#include "candidates.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_map>

namespace pw {

namespace {

using Poly = std::vector<std::array<float,3>>;

constexpr float FLOOR_NZ   = 0.7f;    // walkable-up threshold (matches engine)
constexpr float WALL_NZ    = 0.1f;    // |nz| below this == vertical wall
constexpr float MIN_SEAM   = 2.0f;    // ignore seams shorter than this
constexpr float Z_MATCH    = 2.0f;    // floor-height vs wall-z-range slack
constexpr float XY_MARGIN  = 1.0f;    // bbox overlap slack

// Reconstruct a face's world-space vertex loop.
Poly FacePoly(const Map& map, int fi) {
    Poly p;
    const BFace& f = map.faces[fi];
    p.reserve(f.numedges);
    for (int i = 0; i < f.numedges; ++i) {
        int sidx = f.firstedge + i;
        if (sidx < 0 || sidx >= (int)map.surfedges.size()) return {};
        int se = map.surfedges[sidx];
        int v;
        if (se >= 0) { if (se >= (int)map.edges.size()) return {}; v = map.edges[se].v[0]; }
        else         { if (-se >= (int)map.edges.size()) return {}; v = map.edges[-se].v[1]; }
        if (v < 0 || v >= (int)map.vertices.size()) return {};
        p.push_back(map.vertices[v]);
    }
    return p;
}

// Effective outward normal of a face (respecting side).
std::array<float,3> FaceNormal(const Map& map, int fi) {
    const BFace& f = map.faces[fi];
    const mplane_t& pl = map.planes[f.planenum];
    float s = f.side ? -1.0f : 1.0f;
    return { pl.normal[0] * s, pl.normal[1] * s, pl.normal[2] * s };
}

struct Box2 { float xmin, ymin, xmax, ymax; };

Box2 PolyBox2(const Poly& p) {
    Box2 b{ 1e30f, 1e30f, -1e30f, -1e30f };
    for (auto& v : p) {
        b.xmin = std::min(b.xmin, v[0]); b.ymin = std::min(b.ymin, v[1]);
        b.xmax = std::max(b.xmax, v[0]); b.ymax = std::max(b.ymax, v[1]);
    }
    return b;
}

// Clip the infinite line { nn . (x,y) = dd } to a 2D box, returning the segment
// [out_a, out_b] (z filled by caller). Returns false if no/degenerate overlap.
bool ClipLineToBox(float nx, float ny, float dd, const Box2& b,
                   std::array<float,2>& oa, std::array<float,2>& ob) {
    float L2 = nx * nx + ny * ny;
    if (L2 < 1e-8f) return false;
    // point closest to origin on the line, and unit direction along it
    float p0x = nx * dd / L2, p0y = ny * dd / L2;
    float dx = -ny, dy = nx;
    float dl = std::sqrt(dx * dx + dy * dy);
    dx /= dl; dy /= dl;

    float tmin = -1e30f, tmax = 1e30f;
    // x-slab
    if (std::fabs(dx) > 1e-8f) {
        float t1 = (b.xmin - p0x) / dx, t2 = (b.xmax - p0x) / dx;
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1); tmax = std::min(tmax, t2);
    } else if (p0x < b.xmin || p0x > b.xmax) return false;
    // y-slab
    if (std::fabs(dy) > 1e-8f) {
        float t1 = (b.ymin - p0y) / dy, t2 = (b.ymax - p0y) / dy;
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1); tmax = std::min(tmax, t2);
    } else if (p0y < b.ymin || p0y > b.ymax) return false;

    if (tmax - tmin < MIN_SEAM) return false;
    oa = { p0x + dx * tmin, p0y + dy * tmin };
    ob = { p0x + dx * tmax, p0y + dy * tmax };
    return true;
}

inline long long CellKey(int ix, int iy) {
    return (static_cast<long long>(ix) << 32) ^ (static_cast<unsigned int>(iy));
}

} // namespace

std::vector<Seam> EnumerateSeams(const Map& map, const WorldModels& wm, bool verbose) {
    std::vector<Seam> seams;

    // face -> model index.
    std::vector<int> face_model(map.faces.size(), -1);
    for (size_t m = 0; m < map.models.size(); ++m) {
        const BModel& bm = map.models[m];
        for (int f = bm.firstface; f < bm.firstface + bm.numfaces; ++f)
            if (f >= 0 && f < (int)map.faces.size()) face_model[f] = (int)m;
    }

    auto tex_skip = [&](int fi) -> bool {
        int t = map.face_tex[fi];
        if (t < 0 || t >= (int)map.texnames.size()) return false;
        const std::string& n = map.texnames[t];
        return n == "sky" || n == "null" || n == "aaatrigger" || n == "clip" ||
               n == "origin" || n == "hint" || n == "skip" || n == "bevel";
    };

    // Collect floor and wall faces from solid models only.
    struct FloorF { int fi, model; float z; Box2 box; Poly poly; };
    struct WallF  { int fi, model; float wz0, wz1; Box2 box; float nx, ny, dd_at0; float outn[3]; };
    std::vector<FloorF> floors;
    std::vector<WallF>  walls;

    for (int fi = 0; fi < (int)map.faces.size(); ++fi) {
        int m = face_model[fi];
        if (m < 0 || m >= (int)wm.model_solid.size() || !wm.model_solid[m]) continue;
        if (tex_skip(fi)) continue;
        Poly p = FacePoly(map, fi);
        if (p.size() < 3) continue;
        std::array<float,3> n = FaceNormal(map, fi);

        // world-space offset for brush-entity faces
        std::array<float,3> org = wm.model_origin[m];
        for (auto& v : p) { v[0] += org[0]; v[1] += org[1]; v[2] += org[2]; }

        if (n[2] >= FLOOR_NZ) {
            float zsum = 0; for (auto& v : p) zsum += v[2];
            FloorF ff; ff.fi = fi; ff.model = m; ff.z = zsum / (float)p.size();
            ff.box = PolyBox2(p); ff.poly = std::move(p);
            floors.push_back(std::move(ff));
        } else if (std::fabs(n[2]) < WALL_NZ) {
            float z0 = 1e30f, z1 = -1e30f;
            for (auto& v : p) { z0 = std::min(z0, v[2]); z1 = std::max(z1, v[2]); }
            WallF wf; wf.fi = fi; wf.model = m; wf.wz0 = z0; wf.wz1 = z1;
            wf.box = PolyBox2(p);
            const mplane_t& pl = map.planes[map.faces[fi].planenum];
            // raw plane in world space (shifted by entity origin along normal)
            float d = pl.dist + pl.normal[0]*org[0] + pl.normal[1]*org[1] + pl.normal[2]*org[2];
            wf.nx = pl.normal[0]; wf.ny = pl.normal[1];
            wf.dd_at0 = d;  // nn.(x,y) = d - nz*z ; nz~0 so ~= d
            wf.outn[0] = n[0]; wf.outn[1] = n[1]; wf.outn[2] = 0.0f;
            walls.push_back(wf);
        }
    }

    // ---- Method A: convex horizontal floor edges ----
    for (const FloorF& ff : floors) {
        // floor centroid (XY) for outward direction
        float cx = 0, cy = 0;
        for (auto& v : ff.poly) { cx += v[0]; cy += v[1]; }
        cx /= (float)ff.poly.size(); cy /= (float)ff.poly.size();

        size_t nv = ff.poly.size();
        for (size_t i = 0; i < nv; ++i) {
            const auto& A = ff.poly[i];
            const auto& B = ff.poly[(i + 1) % nv];
            if (std::fabs(A[2] - B[2]) > 0.1f) continue;   // not horizontal
            float ex = B[0] - A[0], ey = B[1] - A[1];
            float elen = std::sqrt(ex * ex + ey * ey);
            if (elen < MIN_SEAM) continue;
            ex /= elen; ey /= elen;
            // outward perpendicular (away from centroid)
            float px = ey, py = -ex;
            float mx = (A[0] + B[0]) * 0.5f - cx, my = (A[1] + B[1]) * 0.5f - cy;
            if (px * mx + py * my < 0) { px = -px; py = -py; }

            Seam s;
            s.a = { A[0], A[1], ff.z };
            s.b = { B[0], B[1], ff.z };
            s.z = ff.z;
            s.outn = { px, py, 0.0f };
            s.floor_model = ff.model;
            s.wall_model = -1;
            s.method = 'A';
            seams.push_back(s);
        }
    }

    // ---- Method B: floor plane x wall plane ----
    // Spatial hash walls by XY cell.
    constexpr float CELL = 128.0f;
    std::unordered_map<long long, std::vector<int>> grid;
    for (int wi = 0; wi < (int)walls.size(); ++wi) {
        const WallF& w = walls[wi];
        int ix0 = (int)std::floor(w.box.xmin / CELL), ix1 = (int)std::floor(w.box.xmax / CELL);
        int iy0 = (int)std::floor(w.box.ymin / CELL), iy1 = (int)std::floor(w.box.ymax / CELL);
        for (int ix = ix0; ix <= ix1; ++ix)
            for (int iy = iy0; iy <= iy1; ++iy)
                grid[CellKey(ix, iy)].push_back(wi);
    }

    for (const FloorF& ff : floors) {
        int ix0 = (int)std::floor(ff.box.xmin / CELL), ix1 = (int)std::floor(ff.box.xmax / CELL);
        int iy0 = (int)std::floor(ff.box.ymin / CELL), iy1 = (int)std::floor(ff.box.ymax / CELL);
        std::vector<int> seen;
        for (int ix = ix0; ix <= ix1; ++ix) {
            for (int iy = iy0; iy <= iy1; ++iy) {
                auto it = grid.find(CellKey(ix, iy));
                if (it == grid.end()) continue;
                for (int wi : it->second) seen.push_back(wi);
            }
        }
        std::sort(seen.begin(), seen.end());
        seen.erase(std::unique(seen.begin(), seen.end()), seen.end());

        for (int wi : seen) {
            const WallF& w = walls[wi];
            // model-index rule: floor model index <= wall model index
            if (ff.model > w.model) continue;
            // wall must span the floor height
            if (ff.z < w.wz0 - Z_MATCH || ff.z > w.wz1 + Z_MATCH) continue;
            // XY bbox overlap (floor vs wall) with slack
            if (w.box.xmax + XY_MARGIN < ff.box.xmin || w.box.xmin - XY_MARGIN > ff.box.xmax) continue;
            if (w.box.ymax + XY_MARGIN < ff.box.ymin || w.box.ymin - XY_MARGIN > ff.box.ymax) continue;

            // line: nn.(x,y) = d - nz*z ; walls have nz~0 so use dd_at0.
            float dd = w.dd_at0;
            Box2 clip{
                std::max(ff.box.xmin, w.box.xmin) - XY_MARGIN,
                std::max(ff.box.ymin, w.box.ymin) - XY_MARGIN,
                std::min(ff.box.xmax, w.box.xmax) + XY_MARGIN,
                std::min(ff.box.ymax, w.box.ymax) + XY_MARGIN };
            std::array<float,2> oa, ob;
            if (!ClipLineToBox(w.nx, w.ny, dd, clip, oa, ob)) continue;

            Seam s;
            s.a = { oa[0], oa[1], ff.z };
            s.b = { ob[0], ob[1], ff.z };
            s.z = ff.z;
            s.outn = { w.outn[0], w.outn[1], 0.0f };
            s.floor_model = ff.model;
            s.wall_model = w.model;
            s.method = 'B';
            seams.push_back(s);
        }
    }

    if (verbose) {
        int a = 0, b = 0;
        for (auto& s : seams) (s.method == 'A' ? a : b)++;
        std::fprintf(stderr, "[candidates] floors=%zu walls=%zu seams=%zu (A=%d B=%d)\n",
                     floors.size(), walls.size(), seams.size(), a, b);
    }
    return seams;
}

// ---- FloorIndex (over-void gate) ----
namespace {
bool point_in_poly(float x, float y, const std::vector<std::array<float,2>>& p) {
    bool in = false;
    size_t n = p.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        float xi = p[i][0], yi = p[i][1], xj = p[j][0], yj = p[j][1];
        if (((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12f) + xi))
            in = !in;
    }
    return in;
}
} // namespace

bool FloorIndex::overFloor(float x, float y, float z, float ztol) const {
    int ix = (int)std::floor(x / cell), iy = (int)std::floor(y / cell);
    auto it = grid.find(CellKey(ix, iy));
    if (it == grid.end()) return false;
    for (int fi : it->second) {
        const FloorFace& f = faces[fi];
        if (std::fabs(f.z - z) > ztol) continue;
        if (point_in_poly(x, y, f.poly)) return true;
    }
    return false;
}

FloorIndex BuildFloorIndex(const Map& map, const WorldModels& wm) {
    FloorIndex idx;

    std::vector<int> face_model(map.faces.size(), -1);
    for (size_t m = 0; m < map.models.size(); ++m) {
        const BModel& bm = map.models[m];
        for (int f = bm.firstface; f < bm.firstface + bm.numfaces; ++f)
            if (f >= 0 && f < (int)map.faces.size()) face_model[f] = (int)m;
    }

    for (int fi = 0; fi < (int)map.faces.size(); ++fi) {
        int m = face_model[fi];
        if (m < 0 || m >= (int)wm.model_solid.size() || !wm.model_solid[m]) continue;
        Poly p = FacePoly(map, fi);
        if (p.size() < 3) continue;
        std::array<float,3> n = FaceNormal(map, fi);
        if (n[2] < FLOOR_NZ) continue;  // floors only
        std::array<float,3> org = wm.model_origin[m];
        FloorFace ff;
        float zsum = 0;
        for (auto& v : p) {
            ff.poly.push_back({ v[0] + org[0], v[1] + org[1] });
            zsum += v[2] + org[2];
        }
        ff.z = zsum / (float)p.size();
        ff.model = m;
        idx.faces.push_back(std::move(ff));
    }

    // spatial hash by xy bbox
    for (int i = 0; i < (int)idx.faces.size(); ++i) {
        const FloorFace& f = idx.faces[i];
        float xmin = 1e30f, ymin = 1e30f, xmax = -1e30f, ymax = -1e30f;
        for (auto& v : f.poly) {
            xmin = std::min(xmin, v[0]); ymin = std::min(ymin, v[1]);
            xmax = std::max(xmax, v[0]); ymax = std::max(ymax, v[1]);
        }
        int ix0 = (int)std::floor(xmin / idx.cell), ix1 = (int)std::floor(xmax / idx.cell);
        int iy0 = (int)std::floor(ymin / idx.cell), iy1 = (int)std::floor(ymax / idx.cell);
        for (int ix = ix0; ix <= ix1; ++ix)
            for (int iy = iy0; iy <= iy1; ++iy)
                idx.grid[CellKey(ix, iy)].push_back(i);
    }
    return idx;
}
} // namespace pw
