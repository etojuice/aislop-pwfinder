#include "candidates.h"
#include "clipdecomp.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_map>

namespace pw {

namespace {
using V3 = std::array<float,3>;
constexpr float FLOOR_NZ      = 0.7f;    // walkable-up threshold (matches engine)
constexpr float SLOPE_FLAT_NZ = 0.99f;   // floor normal.z below this == a tilted ramp
constexpr float WALL_NZ       = 0.1f;    // |nz| below this == vertical wall
constexpr float MIN_SEAM      = 2.0f;    // ignore edges/segments shorter than this
constexpr float Z_MATCH       = 2.0f;    // floor-height vs wall-z-range slack
constexpr float XY_MARGIN     = 1.0f;    // bbox overlap slack

struct Box2 { float xmin, ymin, xmax, ymax; };
// A clip-brush FLOOR face (expanded, world-space): outward plane (n,d) + winding.
struct CFloor { int model; V3 n; float d; Box2 box; std::vector<V3> w; };
// A clip-brush WALL face (expanded, world-space): horizontal plane line + z-span.
struct CWall  { int model; float nx, ny, dd; float z0, z1; Box2 box; V3 outn; };

Box2 BoxOf(const std::vector<V3>& w) {
    Box2 b{ 1e30f, 1e30f, -1e30f, -1e30f };
    for (const auto& v : w) {
        b.xmin = std::min(b.xmin, v[0]); b.ymin = std::min(b.ymin, v[1]);
        b.xmax = std::max(b.xmax, v[0]); b.ymax = std::max(b.ymax, v[1]);
    }
    return b;
}
inline long long CellKey(int ix, int iy) {
    return (static_cast<long long>(ix) << 32) ^ static_cast<unsigned int>(iy);
}

// Clip the infinite line { (nx,ny).(x,y) = dd } to box b -> segment [oa,ob].
bool ClipLineToBox(float nx, float ny, float dd, const Box2& b,
                   std::array<float,2>& oa, std::array<float,2>& ob) {
    float L2 = nx*nx + ny*ny;
    if (L2 < 1e-8f) return false;
    float p0x = nx*dd/L2, p0y = ny*dd/L2, dx = -ny, dy = nx;
    float dl = std::sqrt(dx*dx + dy*dy); dx /= dl; dy /= dl;
    float tmin = -1e30f, tmax = 1e30f;
    if (std::fabs(dx) > 1e-8f) { float t1=(b.xmin-p0x)/dx, t2=(b.xmax-p0x)/dx; if(t1>t2)std::swap(t1,t2); tmin=std::max(tmin,t1); tmax=std::min(tmax,t2); }
    else if (p0x < b.xmin || p0x > b.xmax) return false;
    if (std::fabs(dy) > 1e-8f) { float t1=(b.ymin-p0y)/dy, t2=(b.ymax-p0y)/dy; if(t1>t2)std::swap(t1,t2); tmin=std::max(tmin,t1); tmax=std::min(tmax,t2); }
    else if (p0y < b.ymin || p0y > b.ymax) return false;
    if (tmax - tmin < MIN_SEAM) return false;
    oa = { p0x + dx*tmin, p0y + dy*tmin };
    ob = { p0x + dx*tmax, p0y + dy*tmax };
    return true;
}
} // namespace

// Enumerate candidate seams from CLIPNODE-decompiled collision brushes, per solid
// model and player stance (hull 1 standing / hull 3 duck), in the expanded frame:
//   (A) FLOOR-FACE EDGES over the void / against a wall  (ramp edges, ledges), and
//   (B) FLOOR x WALL crossings                           (continuous floor + a
//       separate wall brush, e.g. a func_wall on a flat floor).
std::vector<Seam> EnumerateSeams(const Map& map, const WorldModels& wm, bool verbose) {
    std::vector<Seam> seams;
    std::vector<CFloor> floors[2];
    std::vector<CWall>  walls[2];

    auto floorZ = [](const CFloor& f, float x, float y) {
        return (f.d - f.n[0]*x - f.n[1]*y) / f.n[2];
    };

    int nEdge = 0;
    for (size_t m = 0; m < map.models.size(); ++m) {
        if (m >= wm.model_solid.size() || !wm.model_solid[m]) continue;
        const BModel& bm = map.models[m];
        V3 org = wm.model_origin[m];

        for (int h : {1, 3}) {                         // player hulls only (2 unused)
            int uh = (h == 1) ? 0 : 1;                 // 0 standing, 1 duck
            std::vector<DecompBrush> brushes = DecompileClipHull(
                map.clipnodes.data(), (int)map.clipnodes.size(), map.planes.data(),
                bm.headnode[h], bm.mins, bm.maxs);

            for (const DecompBrush& b : brushes) {
                if (b.contents != CONTENTS_SOLID) continue;
                for (const DecompFace& face : b.faces) {
                    if (face.w.size() < 3) continue;
                    std::vector<V3> w; w.reserve(face.w.size());
                    for (const auto& v : face.w)
                        w.push_back({ v[0]+org[0], v[1]+org[1], v[2]+org[2] });

                    if (face.n[2] >= FLOOR_NZ) {        // ----- FLOOR face -----
                        CFloor cf; cf.model = (int)m; cf.n = face.n;
                        cf.d = face.d + face.n[0]*org[0] + face.n[1]*org[1] + face.n[2]*org[2];
                        cf.box = BoxOf(w); cf.w = w;

                        // (A) floor-face edges over the void / against a wall.
                        size_t nv = w.size();
                        float cx = 0, cy = 0;
                        for (const auto& v : w) { cx += v[0]; cy += v[1]; }
                        cx /= (float)nv; cy /= (float)nv;
                        for (size_t i = 0; i < nv; ++i) {
                            const V3& A = w[i]; const V3& B = w[(i+1)%nv];
                            float ex = B[0]-A[0], ey = B[1]-A[1];
                            float el = std::sqrt(ex*ex + ey*ey);
                            if (el < MIN_SEAM) continue;
                            ex /= el; ey /= el;
                            float px = ey, py = -ex;   // edge perpendicular
                            float mx = (A[0]+B[0])*0.5f - cx, my = (A[1]+B[1])*0.5f - cy;
                            if (px*mx + py*my < 0) { px = -px; py = -py; }   // away from centroid
                            // exposed-edge prune: skip if solid just OUTWARD + below the
                            // plane (floor continues -> interior edge). Same-model check.
                            float smx = (A[0]+B[0])*0.5f + px, smy = (A[1]+B[1])*0.5f + py;
                            float lp[3] = { smx-org[0], smy-org[1], floorZ(cf, smx, smy) - 1.0f - org[2] };
                            bool interior = false;
                            for (const DecompBrush& b2 : brushes)
                                if (b2.contents == CONTENTS_SOLID && PointInBrush(b2, lp, 0.0f)) { interior = true; break; }
                            if (interior) continue;
                            Seam s;
                            s.a = A; s.b = B; s.outn = { px, py, 0.0f };
                            s.floor_model = (int)m; s.wall_model = -1;
                            s.fn = face.n; s.fd = cf.d; s.slope = face.n[2] < SLOPE_FLAT_NZ;
                            s.usehull = uh;
                            seams.push_back(s); ++nEdge;
                        }
                        floors[uh].push_back(std::move(cf));
                    } else if (std::fabs(face.n[2]) < WALL_NZ) {   // ----- WALL face -----
                        CWall wl; wl.model = (int)m; wl.nx = face.n[0]; wl.ny = face.n[1];
                        wl.dd = face.d + face.n[0]*org[0] + face.n[1]*org[1];   // nz~0
                        float z0 = 1e30f, z1 = -1e30f;
                        for (const auto& v : w) { z0 = std::min(z0, v[2]); z1 = std::max(z1, v[2]); }
                        wl.z0 = z0; wl.z1 = z1; wl.box = BoxOf(w);
                        wl.outn = { face.n[0], face.n[1], 0.0f };
                        walls[uh].push_back(std::move(wl));
                    }
                }
            }
        }
    }

    // (B) FLOOR (flat only) x WALL crossings: a wall standing on a continuous floor
    // has no floor EDGE, so pair the floor plane with overlapping wall lines.
    int nB = 0;
    constexpr float CELL = 128.0f;
    for (int uh = 0; uh < 2; ++uh) {
        std::unordered_map<long long, std::vector<int>> grid;
        for (int wi = 0; wi < (int)walls[uh].size(); ++wi) {
            const CWall& wl = walls[uh][wi];
            int ix0 = (int)std::floor(wl.box.xmin/CELL), ix1 = (int)std::floor(wl.box.xmax/CELL);
            int iy0 = (int)std::floor(wl.box.ymin/CELL), iy1 = (int)std::floor(wl.box.ymax/CELL);
            for (int ix = ix0; ix <= ix1; ++ix) for (int iy = iy0; iy <= iy1; ++iy)
                grid[CellKey(ix, iy)].push_back(wi);
        }
        for (const CFloor& f : floors[uh]) {
            if (f.n[2] < SLOPE_FLAT_NZ) continue;      // flat floors only (ramps: use edges)
            int ix0 = (int)std::floor(f.box.xmin/CELL), ix1 = (int)std::floor(f.box.xmax/CELL);
            int iy0 = (int)std::floor(f.box.ymin/CELL), iy1 = (int)std::floor(f.box.ymax/CELL);
            std::vector<int> seen;
            for (int ix = ix0; ix <= ix1; ++ix) for (int iy = iy0; iy <= iy1; ++iy) {
                auto it = grid.find(CellKey(ix, iy));
                if (it != grid.end()) for (int wi : it->second) seen.push_back(wi);
            }
            std::sort(seen.begin(), seen.end());
            seen.erase(std::unique(seen.begin(), seen.end()), seen.end());
            float fz = f.d / f.n[2];                   // flat floor height
            for (int wi : seen) {
                const CWall& wl = walls[uh][wi];
                if (f.model > wl.model) continue;      // model-index prune (floor <= wall)
                if (fz < wl.z0 - Z_MATCH || fz > wl.z1 + Z_MATCH) continue;
                if (wl.box.xmax + XY_MARGIN < f.box.xmin || wl.box.xmin - XY_MARGIN > f.box.xmax) continue;
                if (wl.box.ymax + XY_MARGIN < f.box.ymin || wl.box.ymin - XY_MARGIN > f.box.ymax) continue;
                Box2 clip{ std::max(f.box.xmin, wl.box.xmin) - XY_MARGIN,
                           std::max(f.box.ymin, wl.box.ymin) - XY_MARGIN,
                           std::min(f.box.xmax, wl.box.xmax) + XY_MARGIN,
                           std::min(f.box.ymax, wl.box.ymax) + XY_MARGIN };
                std::array<float,2> oa, ob;
                if (!ClipLineToBox(wl.nx, wl.ny, wl.dd, clip, oa, ob)) continue;
                Seam s;
                s.a = { oa[0], oa[1], fz }; s.b = { ob[0], ob[1], fz };
                s.outn = wl.outn; s.floor_model = f.model; s.wall_model = wl.model;
                s.fn = f.n; s.fd = f.d; s.slope = false; s.usehull = uh;
                seams.push_back(s); ++nB;
            }
        }
    }

    if (verbose)
        std::fprintf(stderr, "[candidates] clip-brush seams=%zu (edges=%d, floorxwall=%d)\n",
                     seams.size(), nEdge, nB);
    return seams;
}
} // namespace pw
