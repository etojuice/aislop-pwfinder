#include "bsp.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace pw {

// GoldSrc BSP v30. Lump directory: int32 version @0, then 15 lump_t{int32
// fileofs, int32 filelen}. All reads little-endian. See
// ReHLDS/rehlds/public/rehlds/bspfile.h.
namespace {

constexpr int HLBSP_VERSION = 30;
constexpr int HEADER_LUMPS  = 15;

enum {
    LUMP_ENTITIES = 0, LUMP_PLANES = 1, LUMP_TEXTURES = 2, LUMP_VERTEXES = 3,
    LUMP_VISIBILITY = 4, LUMP_NODES = 5, LUMP_TEXINFO = 6, LUMP_FACES = 7,
    LUMP_LIGHTING = 8, LUMP_CLIPNODES = 9, LUMP_LEAFS = 10, LUMP_MARKSURFACES = 11,
    LUMP_EDGES = 12, LUMP_SURFEDGES = 13, LUMP_MODELS = 14,
};

// Record sizes on disk.
constexpr int SZ_PLANE = 20, SZ_CLIPNODE = 8, SZ_MODEL = 64, SZ_FACE = 20,
              SZ_EDGE = 4, SZ_SURFEDGE = 4, SZ_VERTEX = 12, SZ_TEXINFO = 40;

// Little-endian scalar reads. Host is required to be little-endian (checked in
// LoadBsp); GoldSrc data is always little-endian on disk.
inline int32_t  rd_i32(const uint8_t* p) { int32_t  v; std::memcpy(&v, p, 4); return v; }
inline int16_t  rd_i16(const uint8_t* p) { int16_t  v; std::memcpy(&v, p, 2); return v; }
inline uint16_t rd_u16(const uint8_t* p) { uint16_t v; std::memcpy(&v, p, 2); return v; }
inline float    rd_f32(const uint8_t* p) { float    v; std::memcpy(&v, p, 4); return v; }

bool little_endian_host() {
    uint16_t x = 1;
    return *reinterpret_cast<const uint8_t*>(&x) == 1;
}

} // namespace

bool LoadBsp(const std::string& path, Map& out, std::string& err) {
    if (!little_endian_host()) { err = "host is not little-endian"; return false; }

    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) { err = "cannot open file: " + path; return false; }
    std::streamsize sz = f.tellg();
    if (sz < 4 + HEADER_LUMPS * 8) { err = "file too small to be a BSP"; return false; }
    f.seekg(0);
    std::vector<uint8_t> buf(static_cast<size_t>(sz));
    if (!f.read(reinterpret_cast<char*>(buf.data()), sz)) { err = "read failed"; return false; }
    const uint8_t* base = buf.data();
    const size_t   flen = buf.size();

    out.version = rd_i32(base);
    if (out.version != HLBSP_VERSION) {
        char b[96];
        std::snprintf(b, sizeof b, "unsupported BSP version %d (need %d)", out.version, HLBSP_VERSION);
        err = b;
        return false;
    }

    struct Lump { int ofs, len; };
    Lump lumps[HEADER_LUMPS];
    for (int i = 0; i < HEADER_LUMPS; ++i) {
        lumps[i].ofs = rd_i32(base + 4 + i * 8);
        lumps[i].len = rd_i32(base + 4 + i * 8 + 4);
        if (lumps[i].ofs < 0 || lumps[i].len < 0 ||
            static_cast<size_t>(lumps[i].ofs) + lumps[i].len > flen) {
            char b[96];
            std::snprintf(b, sizeof b, "lump %d out of bounds (ofs %d len %d, file %zu)",
                          i, lumps[i].ofs, lumps[i].len, flen);
            err = b;
            return false;
        }
    }

    auto need_multiple = [&](int lump, int rec, const char* name) -> bool {
        if (lumps[lump].len % rec != 0) {
            err = std::string("funny lump size: ") + name;
            return false;
        }
        return true;
    };

    // --- planes ---
    if (!need_multiple(LUMP_PLANES, SZ_PLANE, "planes")) return false;
    {
        int n = lumps[LUMP_PLANES].len / SZ_PLANE;
        const uint8_t* p = base + lumps[LUMP_PLANES].ofs;
        out.planes.resize(n);
        for (int i = 0; i < n; ++i, p += SZ_PLANE) {
            mplane_t& pl = out.planes[i];
            pl.normal[0] = rd_f32(p + 0);
            pl.normal[1] = rd_f32(p + 4);
            pl.normal[2] = rd_f32(p + 8);
            pl.dist      = rd_f32(p + 12);
            pl.type      = static_cast<unsigned char>(rd_i32(p + 16));
            pl.signbits  = 0;
            for (int k = 0; k < 3; ++k)
                if (pl.normal[k] < 0.0f) pl.signbits |= (1u << k);
            pl.pad[0] = pl.pad[1] = 0;
        }
    }

    // --- clipnodes ---
    if (!need_multiple(LUMP_CLIPNODES, SZ_CLIPNODE, "clipnodes")) return false;
    {
        int n = lumps[LUMP_CLIPNODES].len / SZ_CLIPNODE;
        const uint8_t* p = base + lumps[LUMP_CLIPNODES].ofs;
        out.clipnodes.resize(n);
        for (int i = 0; i < n; ++i, p += SZ_CLIPNODE) {
            out.clipnodes[i].planenum    = rd_i32(p + 0);
            out.clipnodes[i].children[0] = rd_i16(p + 4);
            out.clipnodes[i].children[1] = rd_i16(p + 6);
        }
    }

    // --- models ---
    if (!need_multiple(LUMP_MODELS, SZ_MODEL, "models")) return false;
    {
        int n = lumps[LUMP_MODELS].len / SZ_MODEL;
        const uint8_t* p = base + lumps[LUMP_MODELS].ofs;
        out.models.resize(n);
        for (int i = 0; i < n; ++i, p += SZ_MODEL) {
            BModel& m = out.models[i];
            for (int k = 0; k < 3; ++k) m.mins[k]   = rd_f32(p + 0 + k * 4);
            for (int k = 0; k < 3; ++k) m.maxs[k]   = rd_f32(p + 12 + k * 4);
            for (int k = 0; k < 3; ++k) m.origin[k] = rd_f32(p + 24 + k * 4);
            for (int k = 0; k < MAX_MAP_HULLS; ++k) m.headnode[k] = rd_i32(p + 36 + k * 4);
            m.visleafs  = rd_i32(p + 52);
            m.firstface = rd_i32(p + 56);
            m.numfaces  = rd_i32(p + 60);
        }
    }

    // --- faces ---
    if (!need_multiple(LUMP_FACES, SZ_FACE, "faces")) return false;
    {
        int n = lumps[LUMP_FACES].len / SZ_FACE;
        const uint8_t* p = base + lumps[LUMP_FACES].ofs;
        out.faces.resize(n);
        for (int i = 0; i < n; ++i, p += SZ_FACE) {
            BFace& fc = out.faces[i];
            fc.planenum  = rd_i16(p + 0);
            fc.side      = rd_i16(p + 2);
            fc.firstedge = rd_i32(p + 4);
            fc.numedges  = rd_i16(p + 8);
            fc.texinfo   = rd_i16(p + 10);
        }
    }

    // --- edges ---
    if (!need_multiple(LUMP_EDGES, SZ_EDGE, "edges")) return false;
    {
        int n = lumps[LUMP_EDGES].len / SZ_EDGE;
        const uint8_t* p = base + lumps[LUMP_EDGES].ofs;
        out.edges.resize(n);
        for (int i = 0; i < n; ++i, p += SZ_EDGE) {
            out.edges[i].v[0] = rd_u16(p + 0);
            out.edges[i].v[1] = rd_u16(p + 2);
        }
    }

    // --- surfedges ---
    if (!need_multiple(LUMP_SURFEDGES, SZ_SURFEDGE, "surfedges")) return false;
    {
        int n = lumps[LUMP_SURFEDGES].len / SZ_SURFEDGE;
        const uint8_t* p = base + lumps[LUMP_SURFEDGES].ofs;
        out.surfedges.resize(n);
        for (int i = 0; i < n; ++i, p += SZ_SURFEDGE)
            out.surfedges[i] = rd_i32(p);
    }

    // --- vertices ---
    if (!need_multiple(LUMP_VERTEXES, SZ_VERTEX, "vertices")) return false;
    {
        int n = lumps[LUMP_VERTEXES].len / SZ_VERTEX;
        const uint8_t* p = base + lumps[LUMP_VERTEXES].ofs;
        out.vertices.resize(n);
        for (int i = 0; i < n; ++i, p += SZ_VERTEX) {
            out.vertices[i][0] = rd_f32(p + 0);
            out.vertices[i][1] = rd_f32(p + 4);
            out.vertices[i][2] = rd_f32(p + 8);
        }
    }

    // --- texinfo -> per-texinfo miptex index ---
    std::vector<int> texinfo_miptex;
    if (!need_multiple(LUMP_TEXINFO, SZ_TEXINFO, "texinfo")) return false;
    {
        int n = lumps[LUMP_TEXINFO].len / SZ_TEXINFO;
        const uint8_t* p = base + lumps[LUMP_TEXINFO].ofs;
        texinfo_miptex.resize(n);
        for (int i = 0; i < n; ++i, p += SZ_TEXINFO)
            texinfo_miptex[i] = rd_i32(p + 32);
    }

    // --- textures (miptex names) ---
    {
        const int tl_ofs = lumps[LUMP_TEXTURES].ofs;
        const int tl_len = lumps[LUMP_TEXTURES].len;
        if (tl_len >= 4) {
            const uint8_t* tl = base + tl_ofs;
            int nummip = rd_i32(tl);
            if (nummip > 0 && static_cast<size_t>(4 + nummip * 4) <= static_cast<size_t>(tl_len)) {
                out.texnames.resize(nummip);
                for (int i = 0; i < nummip; ++i) {
                    int dofs = rd_i32(tl + 4 + i * 4);
                    std::string name;
                    if (dofs >= 0 && dofs + 16 <= tl_len) {
                        const char* np = reinterpret_cast<const char*>(tl + dofs);
                        char tmp[17]; std::memcpy(tmp, np, 16); tmp[16] = 0;
                        name = tmp;
                        for (char& c : name) c = static_cast<char>(std::tolower((unsigned char)c));
                    }
                    out.texnames[i] = name;
                }
            }
        }
    }

    // Map each face -> texture-name index (via texinfo.miptex).
    out.face_tex.assign(out.faces.size(), -1);
    for (size_t i = 0; i < out.faces.size(); ++i) {
        int ti = out.faces[i].texinfo;
        if (ti >= 0 && ti < (int)texinfo_miptex.size()) {
            int mip = texinfo_miptex[ti];
            if (mip >= 0 && mip < (int)out.texnames.size())
                out.face_tex[i] = mip;
        }
    }

    // --- entities (raw ASCII) ---
    {
        const char* e = reinterpret_cast<const char*>(base + lumps[LUMP_ENTITIES].ofs);
        int len = lumps[LUMP_ENTITIES].len;
        // Trim at first NUL if present.
        int real = 0;
        while (real < len && e[real] != '\0') ++real;
        out.entities.assign(e, e + real);
    }

    return true;
}
} // namespace pw
