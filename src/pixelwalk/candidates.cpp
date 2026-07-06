#include "candidates.h"
#include "hull.h"
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <tuple>
#include <unordered_map>

namespace pw {

namespace {

using Poly = std::vector<std::array<float,3>>;

constexpr float FLOOR_NZ   = 0.7f;    // walkable-up threshold (matches engine)
constexpr float SLOPE_FLAT_NZ = 0.99f;// floor normal.z below this == a tilted ramp
constexpr float FLAT_NZ    = 0.9999f; // clipnode-decompile: treat only true flats as flat
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

constexpr float CLIP_BIG = 1048576.0f;
constexpr float CLIP_EPS = 0.05f;
constexpr float CLIP_BOUNDS_PAD = 128.0f;

struct ClipPlane {
    std::array<float,3> n;
    float d;
    int clipnode;
    bool bound;
};

struct ClipFace {
    std::array<float,3> n;
    float d;
    float surface_d;
    Poly poly;
    Box2 box;
    float zmin, zmax;
    int model;
    int usehull;
};

float dot3(const std::array<float,3>& a, const std::array<float,3>& b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

std::array<float,3> add3(const std::array<float,3>& a, const std::array<float,3>& b) {
    return { a[0] + b[0], a[1] + b[1], a[2] + b[2] };
}

std::array<float,3> sub3(const std::array<float,3>& a, const std::array<float,3>& b) {
    return { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
}

std::array<float,3> mul3(const std::array<float,3>& a, float s) {
    return { a[0] * s, a[1] * s, a[2] * s };
}

std::array<float,3> cross3(const std::array<float,3>& a, const std::array<float,3>& b) {
    return {
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    };
}

float len3(const std::array<float,3>& v) {
    return std::sqrt(dot3(v, v));
}

std::array<float,3> norm3(std::array<float,3> v) {
    float l = len3(v);
    if (l < 1e-8f) return {0, 0, 0};
    return { v[0] / l, v[1] / l, v[2] / l };
}

float HullSupportMinDot(const std::array<float,3>& n, int usehull) {
    float d = 0.0f;
    for (int i = 0; i < 3; ++i)
        d += n[i] * (n[i] >= 0.0f ? PLAYER_MINS[usehull][i] : PLAYER_MAXS[usehull][i]);
    return d;
}

float SurfaceDistFromHullPlane(const ClipPlane& pl, int usehull) {
    return pl.d + HullSupportMinDot(pl.n, usehull);
}

float PlaneZAt(const std::array<float,3>& n, float d, float x, float y) {
    if (std::fabs(n[2]) < 1e-6f)
        return 0.0f;
    return (d - n[0] * x - n[1] * y) / n[2];
}

Poly BaseWindingForPlane(const ClipPlane& pl) {
    std::array<float,3> n = norm3(pl.n);
    std::array<float,3> ref = (std::fabs(n[2]) > 0.9f) ? std::array<float,3>{0, 1, 0}
                                                       : std::array<float,3>{0, 0, 1};
    std::array<float,3> u = norm3(cross3(ref, n));
    std::array<float,3> v = norm3(cross3(n, u));
    std::array<float,3> c = mul3(n, pl.d);
    u = mul3(u, CLIP_BIG);
    v = mul3(v, CLIP_BIG);
    return {
        add3(add3(c, u), v),
        add3(sub3(c, v), u),
        sub3(sub3(c, u), v),
        add3(sub3(c, u), v),
    };
}

void CleanPoly(Poly& p) {
    Poly out;
    out.reserve(p.size());
    auto close = [](const std::array<float,3>& a, const std::array<float,3>& b) {
        return len3(sub3(a, b)) < 0.01f;
    };
    for (const auto& v : p) {
        if (out.empty() || !close(out.back(), v))
            out.push_back(v);
    }
    if (out.size() > 1 && close(out.front(), out.back()))
        out.pop_back();
    p.swap(out);
}

Poly ClipPolyInside(const Poly& in, const ClipPlane& pl) {
    if (in.empty()) return {};
    Poly out;
    out.reserve(in.size() + 1);

    auto dist = [&](const std::array<float,3>& p) {
        return dot3(p, pl.n) - pl.d;
    };

    std::array<float,3> prev = in.back();
    float prevd = dist(prev);
    bool prevIn = prevd <= CLIP_EPS;
    for (const auto& cur : in) {
        float curd = dist(cur);
        bool curIn = curd <= CLIP_EPS;
        if (curIn != prevIn) {
            float denom = prevd - curd;
            if (std::fabs(denom) > 1e-8f) {
                float t = prevd / denom;
                out.push_back(add3(prev, mul3(sub3(cur, prev), t)));
            }
        }
        if (curIn)
            out.push_back(cur);
        prev = cur;
        prevd = curd;
        prevIn = curIn;
    }

    CleanPoly(out);
    return out;
}

bool HasClippedWinding(const std::vector<ClipPlane>& planes, int planeIndex) {
    Poly poly = BaseWindingForPlane(planes[planeIndex]);
    for (int i = 0; i < (int)planes.size(); ++i) {
        if (i == planeIndex)
            continue;
        poly = ClipPolyInside(poly, planes[i]);
        if (poly.size() < 3)
            return false;
    }
    return true;
}

void RemoveRedundantPlanes(std::vector<ClipPlane>& planes) {
    const std::vector<ClipPlane> original = planes;
    planes.clear();
    planes.reserve(original.size());
    for (int i = 0; i < (int)original.size(); ++i) {
        if (HasClippedWinding(original, i))
            planes.push_back(original[i]);
    }
}

std::array<float,3> PolyCentroid3(const Poly& p) {
    std::array<float,3> c{0, 0, 0};
    if (p.empty()) return c;
    for (const auto& v : p)
        c = add3(c, v);
    return mul3(c, 1.0f / (float)p.size());
}

void AddBoundsPlanes(std::vector<ClipPlane>& stack, const BModel& bm) {
    for (int axis = 0; axis < 3; ++axis) {
        std::array<float,3> npos{0, 0, 0};
        npos[axis] = 1.0f;
        stack.push_back({ npos, bm.maxs[axis] + CLIP_BOUNDS_PAD, -1, true });

        std::array<float,3> nneg{0, 0, 0};
        nneg[axis] = -1.0f;
        stack.push_back({ nneg, -(bm.mins[axis] - CLIP_BOUNDS_PAD), -1, true });
    }
}

ClipPlane MakeClipPlane(const Map& map, int clipnode, bool front) {
    const dclipnode_t& cn = map.clipnodes[clipnode];
    const mplane_t& pl = map.planes[cn.planenum];
    float s = front ? -1.0f : 1.0f;
    return { { pl.normal[0] * s, pl.normal[1] * s, pl.normal[2] * s },
             pl.dist * s, clipnode, false };
}

void DecompileClipNode_R(const Map& map, int node, std::vector<ClipPlane>& stack,
                         std::vector<std::vector<ClipPlane>>& leaves, int depth) {
    if (node < 0) {
        if (node == CONTENTS_SOLID)
            leaves.push_back(stack);
        return;
    }
    if (node >= (int)map.clipnodes.size() || depth > (int)map.clipnodes.size())
        return;

    const dclipnode_t& cn = map.clipnodes[node];
    if (cn.planenum < 0 || cn.planenum >= (int)map.planes.size())
        return;

    for (int side = 0; side < 2; ++side) {
        bool front = (side == 0);
        stack.push_back(MakeClipPlane(map, node, front));
        DecompileClipNode_R(map, cn.children[side], stack, leaves, depth + 1);
        stack.pop_back();
    }
}

bool BuildExteriorFace(const std::vector<ClipPlane>& planes, int planeIndex,
                       const hull_t& hull, Poly& poly) {
    poly = BaseWindingForPlane(planes[planeIndex]);
    for (int i = 0; i < (int)planes.size(); ++i) {
        if (i == planeIndex) continue;
        poly = ClipPolyInside(poly, planes[i]);
        if (poly.size() < 3) return false;
    }
    CleanPoly(poly);
    if (poly.size() < 3) return false;

    std::array<float,3> c = PolyCentroid3(poly);
    std::array<float,3> outside = add3(c, mul3(planes[planeIndex].n, 0.25f));
    vec3_t p = { outside[0], outside[1], outside[2] };
    return PM_HullPointContents(&hull, hull.firstclipnode, p) != CONTENTS_SOLID;
}

void AddWorldClipFace(std::vector<ClipFace>& out, const ClipPlane& pl, Poly poly,
                      int model, int usehull, const std::array<float,3>& origin) {
    for (auto& v : poly)
        v = add3(v, origin);

    ClipFace f;
    f.n = pl.n;
    f.d = pl.d + dot3(pl.n, origin);
    f.surface_d = SurfaceDistFromHullPlane(pl, usehull) + dot3(pl.n, origin);
    f.poly = std::move(poly);
    f.box = PolyBox2(f.poly);
    f.zmin = 1e30f;
    f.zmax = -1e30f;
    for (const auto& v : f.poly) {
        f.zmin = std::min(f.zmin, v[2]);
        f.zmax = std::max(f.zmax, v[2]);
    }
    f.model = model;
    f.usehull = usehull;
    out.push_back(std::move(f));
}

void CollectClipFacesForHull(const Map& map, const WorldModels& wm, int model, int usehull,
                             std::vector<ClipFace>& floors, std::vector<ClipFace>& walls) {
    int hi = HullIndexForUsehull(usehull);
    const BModel& bm = map.models[model];
    int root = bm.headnode[hi];
    if (root < 0 || root >= (int)map.clipnodes.size())
        return;

    hull_t hull{};
    hull.clipnodes = map.clipnodes.data();
    hull.planes = map.planes.data();
    hull.firstclipnode = root;
    hull.lastclipnode = (int)map.clipnodes.size() - 1;

    std::vector<ClipPlane> stack;
    AddBoundsPlanes(stack, bm);
    std::vector<std::vector<ClipPlane>> leaves;
    DecompileClipNode_R(map, root, stack, leaves, 0);

    std::array<float,3> origin = wm.model_origin[model];
    for (const auto& leaf : leaves) {
        std::vector<ClipPlane> planes = leaf;
        RemoveRedundantPlanes(planes);
        for (int i = 0; i < (int)planes.size(); ++i) {
            const ClipPlane& pl = planes[i];
            if (pl.bound)
                continue;

            bool floor = pl.n[2] > 0.0f;
            bool wall = !floor && std::fabs(pl.n[2]) < WALL_NZ;
            if (!floor && !wall)
                continue;

            Poly poly;
            if (!BuildExteriorFace(planes, i, hull, poly))
                continue;

            if (floor)
                AddWorldClipFace(floors, pl, std::move(poly), model, usehull, origin);
            else
                AddWorldClipFace(walls, pl, std::move(poly), model, usehull, origin);
        }
    }
}

long long qkey(float v, float scale) {
    return (long long)std::lround(v * scale);
}

std::string SeamKey(const Seam& s) {
    long long ax = qkey(s.a[0], 8.0f), ay = qkey(s.a[1], 8.0f);
    long long bx = qkey(s.b[0], 8.0f), by = qkey(s.b[1], 8.0f);
    if (std::tie(bx, by) < std::tie(ax, ay)) {
        std::swap(ax, bx);
        std::swap(ay, by);
    }
    return std::to_string(s.floor_model) + ":" + std::to_string(s.wall_model) + ":" +
           std::to_string((int)s.method) + ":" +
           std::to_string(s.slope ? 1 : 0) + ":" +
           std::to_string(s.slide ? 1 : 0) + ":" +
           std::to_string(qkey(s.z, 8.0f)) + ":" +
           std::to_string(qkey(s.outn[0], 100.0f)) + ":" +
           std::to_string(qkey(s.outn[1], 100.0f)) + ":" +
           std::to_string(qkey(s.fn[0], 100.0f)) + ":" +
           std::to_string(qkey(s.fn[1], 100.0f)) + ":" +
           std::to_string(qkey(s.fn[2], 100.0f)) + ":" +
           std::to_string(qkey(s.fd, 8.0f)) + ":" +
           std::to_string(ax) + ":" + std::to_string(ay) + ":" +
           std::to_string(bx) + ":" + std::to_string(by);
}

void MergeDuplicateSeams(std::vector<Seam>& seams) {
    std::unordered_map<std::string, size_t> idx;
    std::vector<Seam> out;
    out.reserve(seams.size());
    for (const Seam& s : seams) {
        std::string key = SeamKey(s);
        auto it = idx.find(key);
        if (it == idx.end()) {
            idx.emplace(std::move(key), out.size());
            out.push_back(s);
        } else {
            out[it->second].usehull_mask |= s.usehull_mask;
        }
    }
    seams.swap(out);
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
    struct FloorF { int fi, model; float z; Box2 box; Poly poly;
                    float fn[3]; float fd; bool slope; };
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
            ff.box = PolyBox2(p);
            // World-space effective floor plane (side-flipped so fn.z>0), shifted by
            // the brush-entity origin: fn.x = (fd - dot(fn,x))... used for per-xy z.
            const mplane_t& fpl = map.planes[map.faces[fi].planenum];
            float fs = map.faces[fi].side ? -1.0f : 1.0f;
            ff.fn[0] = n[0]; ff.fn[1] = n[1]; ff.fn[2] = n[2];
            ff.fd = fpl.dist * fs + (n[0]*org[0] + n[1]*org[1] + n[2]*org[2]);
            ff.slope = (n[2] < SLOPE_FLAT_NZ);   // tilted ramp (not near-flat)
            ff.poly = std::move(p);
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
            s.fn = { ff.fn[0], ff.fn[1], ff.fn[2] };
            s.fd = ff.fd; s.slope = ff.slope;
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
            // Pixelwalk needs a cross-model tie where the floor model wins
            // _PM_PlayerTrace's strict-fraction comparison. Same-model seams are
            // ordinary clipnode tree geometry and only add expensive noise.
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
            s.fn = { ff.fn[0], ff.fn[1], ff.fn[2] };
            s.fd = ff.fd; s.slope = ff.slope;
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

std::vector<Seam> EnumerateClipnodeSeams(const Map& map, const WorldModels& wm, bool verbose) {
    std::vector<ClipFace> floors;
    std::vector<ClipFace> walls;

    for (int model = 0; model < (int)map.models.size(); ++model) {
        if (model >= (int)wm.model_solid.size() || !wm.model_solid[model])
            continue;
        if (model >= (int)wm.model_origin.size())
            continue;

        CollectClipFacesForHull(map, wm, model, 0, floors, walls);
        CollectClipFacesForHull(map, wm, model, 1, floors, walls);
    }

    std::vector<Seam> seams;
    constexpr float CELL = 128.0f;
    std::unordered_map<long long, std::vector<int>> grid;
    for (int wi = 0; wi < (int)walls.size(); ++wi) {
        const ClipFace& w = walls[wi];
        int ix0 = (int)std::floor(w.box.xmin / CELL), ix1 = (int)std::floor(w.box.xmax / CELL);
        int iy0 = (int)std::floor(w.box.ymin / CELL), iy1 = (int)std::floor(w.box.ymax / CELL);
        for (int ix = ix0; ix <= ix1; ++ix)
            for (int iy = iy0; iy <= iy1; ++iy)
                grid[CellKey(ix, iy)].push_back(wi);
    }

    for (const ClipFace& ff : floors) {
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

        bool flat = ff.n[2] >= FLAT_NZ;
        bool floorWalk = ff.n[2] > FLOOR_NZ && ff.n[2] < FLAT_NZ;
        bool slide = ff.n[2] > 0.0f && ff.n[2] < FLOOR_NZ;
        if (!flat && !floorWalk && !slide)
            continue;

        for (int wi : seen) {
            const ClipFace& w = walls[wi];
            if (ff.usehull != w.usehull) continue;
            if (flat) {
                if (ff.model >= w.model) continue;
            } else {
                if (ff.model > w.model) continue;
            }
            if (w.box.xmax + XY_MARGIN < ff.box.xmin || w.box.xmin - XY_MARGIN > ff.box.xmax) continue;
            if (w.box.ymax + XY_MARGIN < ff.box.ymin || w.box.ymin - XY_MARGIN > ff.box.ymax) continue;

            Box2 clip{
                std::max(ff.box.xmin, w.box.xmin) - XY_MARGIN,
                std::max(ff.box.ymin, w.box.ymin) - XY_MARGIN,
                std::min(ff.box.xmax, w.box.xmax) + XY_MARGIN,
                std::min(ff.box.ymax, w.box.ymax) + XY_MARGIN };
            std::array<float,2> oa, ob;
            if (!ClipLineToBox(w.n[0], w.n[1], w.d, clip, oa, ob)) continue;

            float clipZa = PlaneZAt(ff.n, ff.d, oa[0], oa[1]);
            float clipZb = PlaneZAt(ff.n, ff.d, ob[0], ob[1]);
            float clipZmin = std::min(clipZa, clipZb);
            float clipZmax = std::max(clipZa, clipZb);
            if (clipZmax < w.zmin - Z_MATCH || clipZmin > w.zmax + Z_MATCH) continue;

            float surfZa = PlaneZAt(ff.n, ff.surface_d, oa[0], oa[1]);
            float surfZb = PlaneZAt(ff.n, ff.surface_d, ob[0], ob[1]);

            Seam s;
            s.a = { oa[0], oa[1], surfZa };
            s.b = { ob[0], ob[1], surfZb };
            s.z = (surfZa + surfZb) * 0.5f;
            std::array<float,3> outn = norm3({ w.n[0], w.n[1], 0.0f });
            s.outn = { outn[0], outn[1], 0.0f };
            s.floor_model = ff.model;
            s.wall_model = w.model;
            s.method = 'C';
            s.usehull_mask = 1 << ff.usehull;
            s.fn = ff.n;
            s.fd = ff.surface_d;
            s.slope = !flat;
            s.slide = slide;
            seams.push_back(s);
        }
    }

    MergeDuplicateSeams(seams);

    if (verbose) {
        std::fprintf(stderr, "[clipnode-candidates] floors=%zu walls=%zu seams=%zu\n",
                     floors.size(), walls.size(), seams.size());
    }
    return seams;
}

std::vector<Seam> EnumerateClipnodeDecompileSeams(const Map& map, const WorldModels& wm, bool verbose) {
    std::vector<Seam> clip = EnumerateClipnodeSeams(map, wm, verbose);
    if (clip.empty())
        return EnumerateSeams(map, wm, verbose);

    if (verbose) {
        std::fprintf(stderr, "[clipnode-decompile-candidates] total=%zu\n", clip.size());
    }
    return clip;
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
