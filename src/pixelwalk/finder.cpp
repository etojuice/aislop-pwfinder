#include "finder.h"
#include "pmove.h"
#include "trace.h"
#include "hull.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>

namespace pw {

namespace {

constexpr float PW_EPS = 0.03125f;    // engine DIST_EPSILON
// Per-frame gravity dip (sv_gravity 800, 100 fps): 800*0.5*0.01. Kept for SimDrive's
// hang bookkeeping. CATCH_MIN = min frames the hull must stay caught on the pixel.
constexpr float HANG_VZ = -4.0f;
constexpr int   CATCH_MIN = 3;

// Height of the floor face's plane at (x,y): z = (fd - fn.x*x - fn.y*y)/fn.z.
// Flat floors -> constant; slopes -> varies along the seam.
float floorZAt(const Seam& s, float x, float y) {
    float nz = s.fn[2];
    if (std::fabs(nz) < 1e-4f) return s.z;
    return (s.fd - s.fn[0]*x - s.fn[1]*y) / nz;
}

// Multi-model point solidity at world point p (order-insensitive; same for both
// clip-tree orderings). Used only for cheap sanity checks.
int PointContentsMulti(const std::vector<physent_t>& phys, int usehull, const vec3_t p) {
    const int hi = HullIndexForUsehull(usehull);
    for (const physent_t& pe : phys) {
        if (!pe.model) continue;
        const hull_t* hull = &pe.model->hulls[hi];
        vec3_t local;
        for (int k = 0; k < 3; ++k)
            local[k] = p[k] - (hull->clip_mins[k] - PLAYER_MINS[usehull][k] + pe.origin[k]);
        if (PM_HullPointContents(hull, hull->firstclipnode, local) == CONTENTS_SOLID)
            return CONTENTS_SOLID;
    }
    return CONTENTS_EMPTY;
}

// Ground pre-gate: teleport to `p` (no shift) and run ONE movement frame with NO
// movement keys (pure gravity), then report whether the hull settled onto ground.
// A real pixelwalk spot is over the void: it stays airborne because a straight-down
// categorize can't see the horizontal-only pixel. A legitimate stand settles onto
// real floor within the frame and is discarded. Run BEFORE the movement sim.
bool SettlesOnGround(const std::vector<physent_t>& phys, int usehull, const vec3_t p) {
    PmContext pm;
    pm.usehull = usehull;
    pm.numphysent = (int)phys.size();
    pm.physents = phys.data();
    pm.dist_epsilon = PW_EPS;
    pm.frametime = 0.01f;                 // 100 fps
    pm.gravity = 1.0f;
    pm.friction = 1.0f;
    pm.movetype = MOVETYPE_WALK;
    pm.waterlevel = 0;
    pm.waterjumptime = 0.0f;
    pm.dead = 0;
    pm.onground = -1;                     // start airborne (over the void)
    pm.maxspeed = 250.0f;
    pm.mv_gravity = 800.0f; pm.mv_bounce = 1.0f;
    pm.mv_accelerate = 5.0f; pm.mv_airaccelerate = 10.0f;
    pm.mv_stopspeed = 75.0f; pm.mv_friction = 4.0f; pm.mv_edgefriction = 2.0f;
    pm.mv_stepsize = 18.0f; pm.mv_maxvelocity = 2000.0f;
    pm.cmd_forwardmove = 0.0f; pm.cmd_sidemove = 0.0f;   // NO movement keys: pure gravity
    pm.angles[0] = 0.0f; pm.angles[1] = 0.0f; pm.angles[2] = 0.0f;
    VectorCopy(p, pm.origin);
    VectorClear(pm.velocity);
    VectorClear(pm.basevelocity);
    PM_PlayerMoveFrame(&pm);             // one pure-gravity frame; categorizes internally
    return pm.onground != -1;
}

// --- SIM: airborne, falling player doing in-air movement ALONG the seam (the
// way the exploit is actually walked: the player hovers over the void at the wall
// and slides sideways while gravity keeps dipping them into the floor plane). A
// pixelwalk slides far while barely dropping; a clean map just falls. Reports how
// far the hull moved horizontally and how far it dropped over `nframes`. ---
struct SimOut { float moved; float dropped; vec3_t endpos; int hang_frames;
               int catch_frames; vec3_t catch_pos;
               int catch_floor_ent; int catch_wall_ent; };   // physents that won on the
                                                             // catch frame (-1 none) -> models

SimOut SimDrive(const std::vector<physent_t>& phys, int usehull, const vec3_t start,
                const vec3_t moveDir, float eps, int nframes) {
    PmContext pm;
    pm.usehull = usehull;
    pm.numphysent = (int)phys.size();
    pm.physents = phys.data();
    pm.dist_epsilon = eps;
    pm.frametime = 0.01f;                 // 100 fps (cmd.msec = 10)
    pm.gravity = 1.0f;
    pm.friction = 1.0f;
    pm.movetype = MOVETYPE_WALK;
    pm.waterlevel = 0;
    pm.waterjumptime = 0.0f;
    pm.dead = 0;
    pm.onground = -1;                     // airborne (over the void)
    // Movement cvars (the CS defaults the user runs with).
    pm.maxspeed = 250.0f;
    pm.mv_gravity = 800.0f; pm.mv_bounce = 1.0f;
    pm.mv_accelerate = 5.0f; pm.mv_airaccelerate = 10.0f;
    pm.mv_stopspeed = 75.0f; pm.mv_friction = 4.0f; pm.mv_edgefriction = 2.0f;
    pm.mv_stepsize = 18.0f; pm.mv_maxvelocity = 2000.0f;
    pm.cmd_forwardmove = 400.0f;         // cl_forwardspeed, +forward held
    pm.cmd_sidemove = 0.0f;
    // Aim so +forward drives along moveDir (level view).
    float yaw = std::atan2(moveDir[1], moveDir[0]) * (180.0f / 3.14159265358979323846f);
    pm.angles[0] = 0.0f; pm.angles[1] = yaw; pm.angles[2] = 0.0f;

    VectorCopy(start, pm.origin);
    VectorClear(pm.velocity);
    VectorClear(pm.basevelocity);

    int hang = 0, caught = 0;
    int cfe = -1, cwe = -1;                       // floor/wall physents on the catch frame
    vec3_t catch_pos; VectorClear(catch_pos);
    for (int f = 0; f < nframes; ++f) {
        PM_PlayerMoveFrame(&pm);
        bool free_space = PointContentsMulti(phys, usehull, pm.origin) != CONTENTS_SOLID;
        // A "hang" frame: airborne, in FREE SPACE, but the floor plane clipped the
        // fall so end-of-frame vertical velocity is still just the half-gravity dip
        // (~-4). Falling accumulates it (-8, -16, ...). The free-space check rejects
        // a STUCK/embedded hull: when PM_FlyMove goes allsolid it zeroes velocity,
        // which also reads as vz==-4 after the gravity fixup — but that's an illegal
        // stuck position, not a pixelwalk.
        if (pm.onground == -1 && std::fabs(pm.velocity[2] - HANG_VZ) < 0.5f && free_space)
            hang++;
        // A "catch" frame (slope-agnostic generalization of hang): PM_FlyMove clipped
        // a FLOOR/slope plane this frame (bit0, normal.z>0.7 — includes ramps) while
        // the hull ends airborne (categorize's straight-down 2u trace misses the
        // pixel) and not embedded. On a flat pixel this coincides with hang; on a
        // slope the hull slides while staying caught, so vz != -4 and only this fires.
        if ((pm.flymove_blocked & 0x01) && pm.onground == -1 && free_space) {
            caught++;
            VectorCopy(pm.origin, catch_pos);
            // Record the models that actually won this catch frame. The floor bit
            // guarantees a floor ent; the wall is the non-floor plane pressed (keep
            // the last valid one across catch frames).
            cfe = pm.flymove_floor_ent;
            if (pm.flymove_wall_ent >= 0) cwe = pm.flymove_wall_ent;
        }
    }

    SimOut o;
    VectorCopy(pm.origin, o.endpos);
    float dx = pm.origin[0] - start[0], dy = pm.origin[1] - start[1];
    o.moved = std::sqrt(dx*dx + dy*dy);
    o.dropped = start[2] - pm.origin[2];
    o.hang_frames = hang;
    o.catch_frames = caught;
    VectorCopy(catch_pos, o.catch_pos);
    o.catch_floor_ent = cfe;
    o.catch_wall_ent = cwe;
    return o;
}

// quantize a position+stance into a dedup key
long long PosKey(const std::array<float,3>& p, int usehull) {
    long long x = (long long)std::lround(p[0] / 2.0f);
    long long y = (long long)std::lround(p[1] / 2.0f);
    long long z = (long long)std::lround(p[2] / 2.0f);
    return (((x & 0x1FFFFF) << 43) ^ ((y & 0x1FFFFF) << 22) ^ ((z & 0x1FFFFF) << 1) ^ usehull);
}

// Detect pixelwalks along one clip-brush FLOOR-FACE EDGE seam (expanded frame).
// The seam is already at hull-center height for its stance, so there is no +feet,
// no z-scan, and no wall re-derivation: place the hull center directly on the
// expanded floor plane, sub-pixel sweep across the edge, drive over it, and apply
// the catch + pixel-dependence test.
void ProcessSeam(const Seam& seam, const WorldModels& wm, const FinderConfig& cfg,
                 std::vector<Find>& out, std::unordered_set<long long>& seen) {
    int usehull = seam.usehull;                       // per-stance seam
    if (usehull == 0 && !cfg.standing) return;
    if (usehull == 1 && !cfg.duck)     return;

    vec3_t a = { seam.a[0], seam.a[1], seam.a[2] };
    vec3_t b = { seam.b[0], seam.b[1], seam.b[2] };
    vec3_t outn = { seam.outn[0], seam.outn[1], 0.0f };
    float onl = std::sqrt(outn[0]*outn[0] + outn[1]*outn[1]);
    if (onl < 1e-4f) return;
    outn[0] /= onl; outn[1] /= onl;

    vec3_t dir = { b[0]-a[0], b[1]-a[1], 0.0f };
    float len = std::sqrt(dir[0]*dir[0] + dir[1]*dir[1]);
    if (len < 1e-3f) return;
    dir[0] /= len; dir[1] /= len;

    // Physents near this seam. The seam z is already the hull-center height; cover
    // the expanded-plane range along the edge plus a margin.
    float za = floorZAt(seam, a[0], a[1]), zb = floorZAt(seam, b[0], b[1]);
    float zlo = std::min(za, zb), zhi = std::max(za, zb);
    float qmins[3] = { std::min(a[0],b[0]) - 64, std::min(a[1],b[1]) - 64, zlo - 96 };
    float qmaxs[3] = { std::max(a[0],b[0]) + 64, std::max(a[1],b[1]) + 64, zhi + 96 };
    std::vector<physent_t> phys;
    SelectPhysents(wm, qmins, qmaxs, phys);

    PmContext pm;
    pm.numphysent = (int)phys.size();
    pm.physents = phys.data();
    pm.usehull = usehull;
    auto modelOf = [&phys](int e){ return (e >= 0 && e < (int)phys.size()) ? phys[e].info : -1; };

    const int   NF   = 15;
    const float BAND = 0.2f;                           // sub-pixel sweep across the edge
    vec3_t md = { -outn[0], -outn[1], 0.0f };          // +forward over the edge (into void/wall)

    for (float t = 0.0f; t <= len + 1e-3f; t += cfg.along_step) {
        float tt = std::min(t, len);
        for (float o = -BAND; o <= BAND + 1e-6f; o += cfg.grid) {
            float x = a[0] + dir[0]*tt + outn[0]*o;
            float y = a[1] + dir[1]*tt + outn[1]*o;
            vec3_t startC = { x, y, floorZAt(seam, x, y) };   // hull center ON the expanded plane
            if (PointContentsMulti(phys, usehull, startC) == CONTENTS_SOLID) continue;
            // Categorize pre-gate: a real pixelwalk start is over the void (airborne);
            // if a pure-gravity frame settles it onto real floor it's a legit stand.
            if (!cfg.skip_categorize && SettlesOnGround(phys, usehull, startC)) continue;

            // Cheap pre-filter: a real catch fires within the first frames, so a
            // 3-frame sim rejects the vast majority of samples that just fall before
            // paying for the two full sims.
            SimOut e1 = SimDrive(phys, usehull, startC, md, PW_EPS, 3);
            if (e1.catch_frames < 1) continue;
            // Catch + pixel-dependence: with the pixel the floor plane keeps clipping
            // the fall (caught); without it the hull cannot stand (falls / embeds).
            SimOut ea = SimDrive(phys, usehull, startC, md, PW_EPS, NF);
            if (ea.catch_frames < CATCH_MIN) continue;
            SimOut eb = SimDrive(phys, usehull, startC, md, 0.0f, NF);
            if (eb.catch_frames >= CATCH_MIN) continue;       // caught both ways -> real, reject

            Find f;
            f.pos = { startC[0], startC[1], startC[2] };
            f.by_walk = true;
            f.by_slope = seam.slope;
            f.advanced = (float)ea.catch_frames;
            // Models from the actual winning catch; fall back to the seam's floor model.
            int cfm = modelOf(ea.catch_floor_ent), cwm = modelOf(ea.catch_wall_ent);
            f.floor_model = (cfm >= 0) ? cfm : seam.floor_model;
            f.wall_model  = (cwm >= 0) ? cwm : seam.wall_model;
            // Model-index rule: the floor wins the seam only if floor_model < wall_model
            // (a coincident-plane tie goes to the lower index). Drop wall <= floor.
            if (f.floor_model >= 0 && f.wall_model >= 0 && f.wall_model <= f.floor_model) continue;

            long long key = PosKey(f.pos, usehull);
            if (!seen.insert(key).second) continue;
            f.usehull = usehull;
            f.approach = { -outn[0], -outn[1], 0.0f };
            f.floor_normal = { seam.fn[0], seam.fn[1], seam.fn[2] };
            f.floor_dist = seam.fd;
            out.push_back(f);
            break;                                            // one find per along-sample
        }
    }
}

} // namespace

void TraceAt(const WorldModels& wm, const float origin[3], float yaw, int usehull,
             int frames, float dist_epsilon, float init_vz) {
    float q0[3] = { origin[0]-256, origin[1]-256, origin[2]-256 };
    float q1[3] = { origin[0]+256, origin[1]+256, origin[2]+256 };
    std::vector<physent_t> phys;
    SelectPhysents(wm, q0, q1, phys);

    PmContext pm;
    pm.usehull = usehull; pm.numphysent = (int)phys.size(); pm.physents = phys.data();
    pm.dist_epsilon = dist_epsilon; pm.frametime = 0.01f;         // 100 fps
    pm.gravity = 1.0f; pm.friction = 1.0f; pm.movetype = MOVETYPE_WALK;
    pm.waterlevel = 0; pm.waterjumptime = 0.0f; pm.dead = 0; pm.onground = -1;
    pm.maxspeed = 250.0f; pm.mv_gravity = 800.0f; pm.mv_bounce = 1.0f;
    pm.mv_accelerate = 5.0f; pm.mv_airaccelerate = 10.0f; pm.mv_stopspeed = 75.0f;
    pm.mv_friction = 4.0f; pm.mv_edgefriction = 2.0f; pm.mv_stepsize = 18.0f; pm.mv_maxvelocity = 2000.0f;
    pm.cmd_forwardmove = 400.0f; pm.cmd_sidemove = 0.0f;
    pm.angles[0] = 0.0f; pm.angles[1] = yaw; pm.angles[2] = 0.0f;
    VectorCopy(origin, pm.origin);
    VectorClear(pm.velocity); VectorClear(pm.basevelocity);
    pm.velocity[2] = init_vz;                 // seed a jump (init_vz = sqrt(2*800*45)=268.33)

    std::printf("[trace] origin (%.3f, %.3f, %.3f)  yaw %.1f  stance %s  eps %.5f  +forward  vz0=%.2f  (%d physents)\n",
                origin[0], origin[1], origin[2], yaw, usehull == 0 ? "standing" : "duck",
                dist_epsilon, init_vz, (int)phys.size());
    PmContext probe = pm;                     // report starting onground without snapping
    PM_CategorizePosition(&probe);
    auto pc = [&](const vec3_t o) { return PointContentsMulti(phys, usehull, o) == CONTENTS_SOLID ? 'S' : 'E'; };
    std::printf("  frame  origin                              velocity                     onground  pc  cF  winFloor(model)  winWall(model)\n");
    std::printf("  init   (%9.3f,%9.3f,%9.3f)  (%8.1f,%8.1f,%8.1f)   %2d       %c   %c\n",
                pm.origin[0], pm.origin[1], pm.origin[2],
                pm.velocity[0], pm.velocity[1], pm.velocity[2], probe.onground, pc(pm.origin), '-');
    auto model_of = [&](int e){ return (e >= 0 && e < (int)phys.size()) ? phys[e].info : -1; };

    for (int f = 0; f < frames; ++f) {
        PM_PlayerMoveFrame(&pm);
        int fe = pm.flymove_floor_ent, we = pm.flymove_wall_ent;
        std::printf("  %4d   (%9.3f,%9.3f,%9.3f)  (%8.1f,%8.1f,%8.1f)   %2d       %c   %c   ent%d(*%d)      ent%d(*%d)\n",
                    f, pm.origin[0], pm.origin[1], pm.origin[2],
                    pm.velocity[0], pm.velocity[1], pm.velocity[2], pm.onground, pc(pm.origin),
                    (pm.flymove_blocked & 0x01) ? 'Y' : '.', fe, model_of(fe), we, model_of(we));
    }
}

std::vector<Find> RunFinder(const Map& map, const WorldModels& wm,
                            const std::vector<Seam>& seams, const FinderConfig& cfg) {
    (void)map;
    int nthreads = cfg.threads;
    if (nthreads <= 0) nthreads = (int)std::thread::hardware_concurrency();
    if (nthreads <= 0) nthreads = 1;
    nthreads = std::min<int>(nthreads, std::max<size_t>(1, seams.size()));

    std::vector<std::vector<Find>> partial(nthreads);
    std::vector<std::thread> pool;

    auto worker = [&](int tid) {
        std::unordered_set<long long> seen;
        std::vector<Find>& out = partial[tid];
        for (size_t i = tid; i < seams.size(); i += nthreads)
            ProcessSeam(seams[i], wm, cfg, out, seen);
    };

    if (nthreads == 1) {
        worker(0);
    } else {
        for (int t = 0; t < nthreads; ++t) pool.emplace_back(worker, t);
        for (auto& th : pool) th.join();
    }

    // Merge partials, deduping across threads and OR-ing method flags.
    std::vector<Find> finds;
    std::unordered_map<long long, size_t> idx;
    for (auto& pv : partial) {
        for (Find& f : pv) {
            long long key = PosKey(f.pos, f.usehull);
            auto it = idx.find(key);
            if (it == idx.end()) {
                idx[key] = finds.size();
                finds.push_back(f);
            } else {
                Find& g = finds[it->second];
                g.by_walk  |= f.by_walk;
                g.by_slope |= f.by_slope;
                if (f.advanced > g.advanced) g.advanced = f.advanced;
            }
        }
    }

    // Cluster nearby hits (dense sub-pixel samples on the same seam) into distinct
    // spots so the report is a usable list of locations.
    if (!cfg.zones) {
        const float CELL = 64.0f;
        std::unordered_map<long long, size_t> cidx;
        std::vector<Find> clustered;
        for (Find& f : finds) {
            long long kx = (long long)std::lround(f.pos[0] / CELL);
            long long ky = (long long)std::lround(f.pos[1] / CELL);
            long long kz = (long long)std::lround(f.pos[2] / CELL);
            long long key = (((kx & 0x1FFFFF) << 43) ^ ((ky & 0x1FFFFF) << 22) ^
                             ((kz & 0x1FFFFF) << 1) ^ f.usehull);
            auto it = cidx.find(key);
            if (it == cidx.end()) {
                cidx[key] = clustered.size();
                clustered.push_back(f);
            } else {
                Find& g = clustered[it->second];
                g.cluster_size++;
                g.by_walk |= f.by_walk; g.by_slope |= f.by_slope;
                if (f.advanced > g.advanced) {   // keep the strongest representative
                    g.pos = f.pos; g.advanced = f.advanced;
                    g.floor_normal = f.floor_normal; g.floor_dist = f.floor_dist;
                    g.approach = f.approach; g.floor_model = f.floor_model;
                    g.wall_model = f.wall_model;
                }
            }
        }
        finds.swap(clustered);
    } else {
        // --zones: group contiguous COLLINEAR finds into from->to spans. Finds on
        // one wall already share approach (wall-normal dir), the floor plane
        // (normal+dist; constant even on a ramp, where only z varies), and a
        // constant perpendicular coord — only the along-wall coord varies (~1 per
        // 2u). So bucket by (stance, approach, floor plane), sort by along-coord,
        // and split a new span on a big along-gap or a perpendicular jump.
        const float PERP_TOL = 4.0f;
        auto nonzeroN = [](const std::array<float,3>& n) {
            return n[0] != 0.0f || n[1] != 0.0f || n[2] != 0.0f;
        };
        auto lineKey = [](const Find& f) -> long long {
            long long parts[9] = {
                (long long)f.usehull,
                (long long)std::lround(f.approach[0] * 100.0f),
                (long long)std::lround(f.approach[1] * 100.0f),
                (long long)std::lround(f.floor_normal[0] * 100.0f),
                (long long)std::lround(f.floor_normal[1] * 100.0f),
                (long long)std::lround(f.floor_normal[2] * 100.0f),
                (long long)std::lround(f.floor_dist),
                (long long)f.floor_model,   // different brush entities never merge
                (long long)f.wall_model,
            };
            long long h = 1469598103934665603LL;   // FNV-1a-ish mix
            for (long long v : parts) h = (h ^ (v & 0xFFFFF)) * 1099511628211LL;
            return h;
        };
        std::unordered_map<long long, std::vector<Find>> groups;
        for (Find& f : finds) groups[lineKey(f)].push_back(f);

        std::vector<Find> zones;
        for (auto& kv : groups) {
            std::vector<Find>& g = kv.second;
            if (g.empty()) continue;
            // along-dir = perpendicular to approach in xy (unit, since approach unit)
            float ax = g[0].approach[0], ay = g[0].approach[1];
            float adx = -ay, ady = ax;
            auto along = [&](const Find& f){ return f.pos[0]*adx + f.pos[1]*ady; };
            auto perp  = [&](const Find& f){ return f.pos[0]*ax  + f.pos[1]*ay;  };
            std::sort(g.begin(), g.end(),
                      [&](const Find& p, const Find& q){ return along(p) < along(q); });

            size_t i = 0;
            while (i < g.size()) {
                size_t j = i;
                while (j + 1 < g.size()) {
                    if (along(g[j+1]) - along(g[j]) > cfg.zone_gap) break;
                    if (std::fabs(perp(g[j+1]) - perp(g[j])) > PERP_TOL) break;
                    ++j;
                }
                Find z = g[i];                         // seed: min-along endpoint = from
                z.to = g[j].pos;                        // max-along endpoint = to
                z.length = along(g[j]) - along(g[i]);
                z.cluster_size = (int)(j - i + 1);
                z.advanced = 0.0f;
                bool haveN = nonzeroN(z.floor_normal);
                for (size_t k = i; k <= j; ++k) {
                    z.by_walk |= g[k].by_walk; z.by_slope |= g[k].by_slope;
                    if (g[k].advanced > z.advanced) {   // strongest sample -> representative
                        z.advanced = g[k].advanced;
                        z.approach = g[k].approach; z.floor_model = g[k].floor_model;
                        z.wall_model = g[k].wall_model;
                    }
                    if (!haveN && nonzeroN(g[k].floor_normal)) {   // plane from a probe/slope hit
                        z.floor_normal = g[k].floor_normal; z.floor_dist = g[k].floor_dist;
                        haveN = true;
                    }
                }
                zones.push_back(z);
                i = j + 1;
            }
        }
        finds.swap(zones);
    }

    // Order: stance (standing then duck), then method (sim first), then
    // most-sampled/strongest within each group.
    std::sort(finds.begin(), finds.end(), [](const Find& a, const Find& b) {
        if (a.usehull != b.usehull) return a.usehull < b.usehull;   // standing(0) before duck(1)
        bool as = a.by_walk || a.by_slope;   // has the sim method?
        bool bs = b.by_walk || b.by_slope;
        if (as != bs) return as > bs;                                // sim-detected first
        if (a.cluster_size != b.cluster_size) return a.cluster_size > b.cluster_size;
        return a.advanced > b.advanced;
    });

    // Drop low-confidence spots: a real pixelwalk region trips many adjacent
    // sub-pixel samples, so few hits (small cluster) is usually noise.
    if (cfg.min_samples > 1) {
        std::vector<Find> kept;
        kept.reserve(finds.size());
        for (const Find& f : finds)
            if (f.cluster_size >= cfg.min_samples) kept.push_back(f);
        finds.swap(kept);
    }

    if (cfg.max_finds > 0 && (int)finds.size() > cfg.max_finds)
        finds.resize(cfg.max_finds);
    return finds;
}
} // namespace pw
