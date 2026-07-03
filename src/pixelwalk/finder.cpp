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
constexpr int   HANG_MIN = 3;         // min frames the hull must hang on the pixel
constexpr float FALL_DIFF = 5.0f;     // min extra fall (units) without the pixel vs with it
                                      // — confirms the hang is pixel-dependent, not a wedge
// Per-frame gravity dip (sv_gravity 800, 100 fps): 800*0.5*0.01. While hanging on
// a horizontal pixel plane the floor clips the fall so end-of-frame vz stays here.
constexpr float HANG_VZ = -4.0f;

float feetOffset(int usehull) { return -PLAYER_MINS[usehull][2]; }  // 36 or 18

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

// True if a player placed at `p` is on solid ground (PM_CategorizePosition's 2u
// down-trace finds a walkable plane). A real pixelwalk spot is over the void
// (airborne), so an on-ground start is a legitimate stand, not a pixelwalk.
bool OnGroundAt(const std::vector<physent_t>& phys, int usehull, const vec3_t p) {
    PmContext pm;
    pm.usehull = usehull;
    pm.numphysent = (int)phys.size();
    pm.physents = phys.data();
    pm.dist_epsilon = PW_EPS;
    pm.frametime = 0.01f;
    pm.waterlevel = 0;
    pm.waterjumptime = 0;
    pm.onground = -1;
    VectorCopy(p, pm.origin);
    VectorClear(pm.velocity);
    PM_CategorizePosition(&pm);
    return pm.onground != -1;
}

// --- PROBE: reproduce one real pixelwalk step. From an airborne hull center `c`
// on the void side of the wall clip plane, sweep exactly like a player walking
// into the wall while gravity dips them (horizontal into-wall + small down),
// one player frame at 100 fps (cmd.msec=10, frametime=0.01). ---
pmtrace_t ProbeSweep(PmContext& pm, const vec3_t c, const vec3_t outward, float eps) {
    pm.dist_epsilon = eps;
    const float HORIZ = 250.0f * 0.01f;                 // 250 u/s run * 0.01 = 2.5
    const float DOWN  = 800.0f * 0.5f * 0.01f * 0.01f;  // gravity dip = 0.04 (> DIST_EPSILON)
    vec3_t p1 = { c[0], c[1], c[2] };
    vec3_t p2 = { c[0] - outward[0]*HORIZ, c[1] - outward[1]*HORIZ, c[2] - DOWN };
    return PM_PlayerTrace(&pm, p1, p2, PM_NORMAL, -1);
}
inline bool HitFloor(const pmtrace_t& t) {
    return t.fraction < 1.0f && !t.startsolid && !t.allsolid && t.plane.normal[2] >= 0.7f;
}
// "Blocked by a wall": a real impact that is NOT a floor (vertical/steep). Used
// as the zero-epsilon reference so we only accept a genuine WALL->floor flip, not
// EMPTY->floor (which is just walking off an ordinary ledge into open air).
inline bool HitWall(const pmtrace_t& t) {
    return t.fraction < 1.0f && !t.startsolid && !t.allsolid && t.plane.normal[2] < 0.7f;
}

// --- SIM: airborne, falling player doing in-air movement ALONG the seam (the
// way the exploit is actually walked: the player hovers over the void at the wall
// and slides sideways while gravity keeps dipping them into the floor plane). A
// pixelwalk slides far while barely dropping; a clean map just falls. Reports how
// far the hull moved horizontally and how far it dropped over `nframes`. ---
struct SimOut { float moved; float dropped; vec3_t endpos; int onground_f2; int hang_frames; };

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

    int og2 = -1, hang = 0;
    for (int f = 0; f < nframes; ++f) {
        PM_PlayerMoveFrame(&pm);
        if (f == 1) og2 = pm.onground;   // onground after the 2nd movement frame
        // A "hang" frame: airborne, in FREE SPACE, but the floor plane clipped the
        // fall so end-of-frame vertical velocity is still just the half-gravity dip
        // (~-4). Falling accumulates it (-8, -16, ...). The free-space check rejects
        // a STUCK/embedded hull: when PM_FlyMove goes allsolid it zeroes velocity,
        // which also reads as vz==-4 after the gravity fixup — but that's an illegal
        // stuck position, not a pixelwalk.
        if (pm.onground == -1 && std::fabs(pm.velocity[2] - HANG_VZ) < 0.5f &&
            PointContentsMulti(phys, usehull, pm.origin) != CONTENTS_SOLID)
            hang++;
    }

    SimOut o;
    VectorCopy(pm.origin, o.endpos);
    float dx = pm.origin[0] - start[0], dy = pm.origin[1] - start[1];
    o.moved = std::sqrt(dx*dx + dy*dy);
    o.dropped = start[2] - pm.origin[2];
    o.onground_f2 = og2;
    o.hang_frames = hang;
    return o;
}

// quantize a position+stance into a dedup key
long long PosKey(const std::array<float,3>& p, int usehull) {
    long long x = (long long)std::lround(p[0] / 2.0f);
    long long y = (long long)std::lround(p[1] / 2.0f);
    long long z = (long long)std::lround(p[2] / 2.0f);
    return (((x & 0x1FFFFF) << 43) ^ ((y & 0x1FFFFF) << 22) ^ ((z & 0x1FFFFF) << 1) ^ usehull);
}

// Find the away-from-wall direction and the hull center resting against the wall
// near `base` at height `cz`. The pixelwalk "red square" sits at the wall CLIP
// plane, ~hull-half-width out from the wall surface, so we locate it with a
// horizontal trace into the wall (constant z never touches the floor plane).
// Tries both signs of the candidate seam normal. Returns false if no wall.
bool FindOutwardAndContact(PmContext& pm, const std::vector<physent_t>& phys, int usehull,
                           const vec3_t base, const vec3_t seamOut, float cz,
                           vec3_t outward, vec3_t contact, int& wallModel) {
    pm.usehull = usehull;
    pm.dist_epsilon = PW_EPS;
    for (int sgn = 0; sgn < 2; ++sgn) {
        vec3_t d = { seamOut[0], seamOut[1], 0.0f };
        if (sgn == 1) { d[0] = -d[0]; d[1] = -d[1]; }
        vec3_t far_  = { base[0] + d[0]*24.0f, base[1] + d[1]*24.0f, cz };
        vec3_t near_ = { base[0] - d[0]*8.0f,  base[1] - d[1]*8.0f,  cz };
        if (PointContentsMulti(phys, usehull, far_) == CONTENTS_SOLID) continue;
        pmtrace_t wc = PM_PlayerTrace(&pm, far_, near_, PM_NORMAL, -1);
        if (wc.startsolid || wc.fraction >= 1.0f) continue;
        if (std::fabs(wc.plane.normal[2]) >= 0.3f) continue;   // must be a (near-)vertical wall
        outward[0] = d[0]; outward[1] = d[1]; outward[2] = 0.0f;
        VectorCopy(wc.endpos, contact);
        wallModel = (wc.ent >= 0 && wc.ent < (int)phys.size()) ? phys[wc.ent].info : -1;
        return true;
    }
    return false;
}

void ProcessSeam(const Seam& seam, const WorldModels& wm, const FloorIndex& floors,
                 const FinderConfig& cfg,
                 std::vector<Find>& out, std::unordered_set<long long>& seen) {
    // seam geometry
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

    // Physents near this seam (world + overlapping solid brush ents).
    float qmins[3] = { std::min(a[0],b[0]) - 64, std::min(a[1],b[1]) - 64, seam.z - 96 };
    float qmaxs[3] = { std::max(a[0],b[0]) + 64, std::max(a[1],b[1]) + 64, seam.z + 128 };
    std::vector<physent_t> phys;
    SelectPhysents(wm, qmins, qmaxs, phys);

    PmContext pm;
    pm.numphysent = (int)phys.size();
    pm.physents = phys.data();

    int stances[2]; int ns = 0;
    if (cfg.standing) stances[ns++] = 0;
    if (cfg.duck)     stances[ns++] = 1;

    for (int si = 0; si < ns; ++si) {
        int usehull = stances[si];
        pm.usehull = usehull;
        float feet = feetOffset(usehull);
        float cz = seam.z + feet;   // hull-center height with feet on the seam floor

        for (float t = 0.0f; t <= len + 1e-3f; t += cfg.along_step) {
            float tt = std::min(t, len);
            vec3_t base = { a[0] + dir[0]*tt, a[1] + dir[1]*tt, cz };

            Find f;
            bool got = false;

            // Locate the wall clip plane (the pixelwalk "red square" is here, not
            // at the wall surface) and the correct outward direction.
            vec3_t outward, contact;
            int wallModel = -1;
            if (!FindOutwardAndContact(pm, phys, usehull, base, outn, seam.z + feet + 0.5f,
                                       outward, contact, wallModel))
                continue;

            // ---- PROBE: epsilon-differential of one real pixelwalk step. A find
            // is where the move resolves to a FLOOR with the engine epsilon but is
            // BLOCKED BY A WALL without it (a genuine wall->floor flip). Requiring a
            // wall block (not just "not floor") rejects ordinary ledges where the
            // zero-epsilon sweep sails off the edge into open air. ----
            if (cfg.do_probe) {
                float pcz = seam.z + feet;   // hull center at the floor clip plane (feet on floor)
                for (float o = -cfg.band; o <= cfg.band + 1e-6f; o += cfg.grid) {
                    vec3_t c = { contact[0] + outward[0]*o, contact[1] + outward[1]*o, pcz };
                    pmtrace_t te = ProbeSweep(pm, c, outward, PW_EPS);
                    if (!HitFloor(te)) continue;
                    pmtrace_t t0 = ProbeSweep(pm, c, outward, 0.0f);
                    if (!HitWall(t0)) continue;         // must be WALL->floor, not EMPTY->floor
                    f.pos = { c[0], c[1], seam.z + feet };
                    f.floor_normal = { te.plane.normal[0], te.plane.normal[1], te.plane.normal[2] };
                    f.floor_dist = te.plane.dist;
                    f.by_probe = true;
                    got = true;
                    break;
                }
            }

            // ---- SIM SLIDE (airborne at the seam, gravity + in-air move into the
            // wall; a pixelwalk lets the fall's downward velocity get clipped by the
            // floor plane so the hull slides horizontally instead of dropping) ----
            // ---- SIM: full PM_PlayerMove reproduction. Hover at the pixel over the
            // void, airborne, and walk (+forward) along the seam. On a pixelwalk map
            // the floor plane catches the fall each frame so the hull HANGS (small
            // drop); on a clean map it just falls. Detect via the drop differential
            // (epsilon on vs off) so it's intrinsic to one map. ----
            if (cfg.do_walk || cfg.do_fall) {
                const int NF = 15;
                vec3_t md = { -outward[0], -outward[1], 0.0f };   // +forward INTO the wall
                // Sweep the start across the ~1/32 pixel band around the wall clip
                // plane (contact sits ~1 epsilon out on the void side). Runs even if
                // the probe already fired, so a spot both methods agree on is tagged
                // probe+sim.
                for (float o = -0.05f; o <= 0.15f + 1e-6f; o += cfg.grid) {
                    vec3_t startC = { contact[0] + outward[0]*o, contact[1] + outward[1]*o, cz };
                    if (PointContentsMulti(phys, usehull, startC) == CONTENTS_SOLID) continue;
                    // A real pixelwalk start is over the void (airborne). If the
                    // player is already on ground here it's a legitimate stand -
                    // skip the sim (unless --skip-categorize-pos disables this).
                    if (!cfg.skip_categorize && OnGroundAt(phys, usehull, startC)) continue;
                    SimOut ea = SimDrive(phys, usehull, startC, md, PW_EPS, NF);
                    // If the player has landed on ground by the end of the 2nd frame
                    // it settled on a real floor (legitimate stand), not a pixelwalk.
                    if (!cfg.skip_categorize && ea.onground_f2 != -1) continue;
                    // Pixelwalk requires BOTH:
                    //  (1) the hull HANGS with the pixel — vz pinned at ~-4 (floor
                    //      clips the fall) for several frames; AND
                    //  (2) that hang is PIXEL-DEPENDENT — without the epsilon the hull
                    //      actually falls (>= FALL_DIFF more). This rejects geometric
                    //      wedges/stuck corners that stay put with or without the pixel.
                    if (ea.hang_frames >= HANG_MIN) {
                        SimOut eb = SimDrive(phys, usehull, startC, md, 0.0f, NF);
                        if (eb.dropped - ea.dropped > FALL_DIFF) {
                            if (!f.by_probe) f.pos = { startC[0], startC[1], seam.z + feet };
                            f.by_walk = true;
                            if (ea.hang_frames > f.advanced) f.advanced = (float)ea.hang_frames;
                            got = true;
                            break;
                        }
                    }
                }
            }

            if (!got) continue;

            // Over-void gate: a genuine pixelwalk stand is over the void (no real
            // floor face under the hull center). Benign concave corners have real
            // floor beneath and are rejected here.
            if (floors.overFloor(f.pos[0], f.pos[1], seam.z, 3.0f)) continue;

            long long key = PosKey(f.pos, usehull);
            if (!seen.insert(key).second) continue;

            f.usehull = usehull;
            f.approach = { -outward[0], -outward[1], 0.0f };
            f.floor_model = seam.floor_model;
            f.wall_model = (wallModel >= 0) ? wallModel : seam.wall_model;
            out.push_back(f);
        }
    }
}

} // namespace

void TraceAt(const WorldModels& wm, const float origin[3], float yaw, int usehull,
             int frames, float dist_epsilon) {
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

    std::printf("[trace] origin (%.3f, %.3f, %.3f)  yaw %.1f  stance %s  eps %.5f  +forward  (%d physents)\n",
                origin[0], origin[1], origin[2], yaw, usehull == 0 ? "standing" : "duck",
                dist_epsilon, (int)phys.size());
    PmContext probe = pm;                     // report starting onground without snapping
    PM_CategorizePosition(&probe);
    auto pc = [&](const vec3_t o) { return PointContentsMulti(phys, usehull, o) == CONTENTS_SOLID ? 'S' : 'E'; };
    std::printf("  frame  origin                              velocity                     onground  pc(E/S)\n");
    std::printf("  init   (%9.3f,%9.3f,%9.3f)  (%8.1f,%8.1f,%8.1f)   %2d       %c\n",
                pm.origin[0], pm.origin[1], pm.origin[2],
                pm.velocity[0], pm.velocity[1], pm.velocity[2], probe.onground, pc(pm.origin));
    for (int f = 0; f < frames; ++f) {
        PM_PlayerMoveFrame(&pm);
        std::printf("  %4d   (%9.3f,%9.3f,%9.3f)  (%8.1f,%8.1f,%8.1f)   %2d       %c\n",
                    f, pm.origin[0], pm.origin[1], pm.origin[2],
                    pm.velocity[0], pm.velocity[1], pm.velocity[2], pm.onground, pc(pm.origin));
    }
}

std::vector<Find> RunFinder(const Map& map, const WorldModels& wm, const FloorIndex& floors,
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
            ProcessSeam(seams[i], wm, floors, cfg, out, seen);
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
                g.by_probe |= f.by_probe;
                g.by_walk  |= f.by_walk;
                g.by_fall  |= f.by_fall;
                if (f.by_probe && (g.floor_normal[0]==0&&g.floor_normal[1]==0&&g.floor_normal[2]==0)) {
                    g.floor_normal = f.floor_normal; g.floor_dist = f.floor_dist;
                }
                if (f.advanced > g.advanced) g.advanced = f.advanced;
                if (f.fall_N && !g.fall_N) g.fall_N = f.fall_N;
            }
        }
    }

    // Cluster nearby hits (dense sub-pixel samples on the same seam) into distinct
    // spots so the report is a usable list of locations.
    {
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
                g.by_probe |= f.by_probe; g.by_walk |= f.by_walk; g.by_fall |= f.by_fall;
                if (f.advanced > g.advanced) {   // keep the strongest representative
                    g.pos = f.pos; g.advanced = f.advanced;
                    g.floor_normal = f.floor_normal; g.floor_dist = f.floor_dist;
                    g.approach = f.approach; g.floor_model = f.floor_model;
                    g.wall_model = f.wall_model; g.fall_N = f.fall_N;
                }
            }
        }
        finds.swap(clustered);
    }

    // Order: stance (standing then duck), then method (sim first), then
    // most-sampled/strongest within each group.
    std::sort(finds.begin(), finds.end(), [](const Find& a, const Find& b) {
        if (a.usehull != b.usehull) return a.usehull < b.usehull;   // standing(0) before duck(1)
        bool as = a.by_walk || a.by_fall, bs = b.by_walk || b.by_fall;   // has the sim method?
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
