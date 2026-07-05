// trace.h - swept hull trace (PM_RecursiveHullCheck) + multi-physent player
// trace (_PM_PlayerTrace). This is the order-sensitive core that separates a
// pixelwalk map from a clean one.
#pragma once
#include "engine_types.h"

namespace pw {

// Trace flags (pm_defs.h subset).
enum {
    PM_NORMAL       = 0x00,
    PM_STUDIO_IGNORE= 0x01,
    PM_STUDIO_BOX   = 0x02,
    PM_GLASS_IGNORE = 0x04,
    PM_WORLD_ONLY   = 0x08,
};

// The engine's global `pmove` is a per-frame singleton; to trace on many threads
// we pass an explicit context instead. Field names mirror pmove_t so the ported
// code reads the same.
struct PmContext {
    // --- trace inputs ---
    int usehull = 0;                     // 0 standing (hull1), 1 duck (hull3)
    int numphysent = 0;
    const physent_t* physents = nullptr; // [0] = world, then brush ents by model index
    float dist_epsilon = 0.03125f;       // set 0 for the epsilon-differential oracle

    // --- movement/sim state (pmove.cpp) ---
    vec3_t origin{};
    vec3_t velocity{};
    vec3_t basevelocity{};
    vec3_t angles{};                     // view angles (pitch,yaw,roll)
    vec3_t forward{}, right{}, up{};     // derived from angles each frame
    float  cmd_forwardmove = 0.0f;       // cl_forwardspeed when +forward held
    float  cmd_sidemove = 0.0f;
    float  frametime = 0.0f;
    float  gravity = 1.0f;               // entity gravity multiplier
    float  friction = 1.0f;              // player friction multiplier
    int    onground = -1;                // -1 airborne, else physent index underfoot
    int    flymove_blocked = 0;          // OR of PM_FlyMove blocked flags this frame
                                         //   (bit0=floor/slope normal.z>0.7, bit1=wall)
    int    flymove_floor_ent = -1;       // physent index of the FLOOR plane clipped this
                                         //   frame (-1 none). phys[ent].info = model index.
                                         //   Sourced as a find's floor_model at the catch.
    int    flymove_wall_ent = -1;        // physent index of the non-floor (wall/steep)
                                         //   plane clipped this frame (-1 none) -> wall_model.
    int    movetype = 3;                 // MOVETYPE_WALK
    int    waterlevel = 0;
    float  waterjumptime = 0.0f;
    int    dead = 0;
    // movevars (server cvars)
    float  maxspeed = 250.0f;            // player class cap (wishspeed clamp)
    float  mv_gravity = 800.0f;          // sv_gravity
    float  mv_bounce = 1.0f;             // sv_bounce
    float  mv_accelerate = 5.0f;         // sv_accelerate
    float  mv_airaccelerate = 10.0f;     // sv_airaccelerate
    float  mv_stopspeed = 75.0f;         // sv_stopspeed
    float  mv_friction = 4.0f;           // sv_friction
    float  mv_edgefriction = 2.0f;       // edgefriction
    float  mv_stepsize = 18.0f;          // sv_stepsize
    float  mv_maxvelocity = 2000.0f;     // sv_maxvelocity
};

// PM_RecursiveHullCheck - pmovetst.cpp:614. Returns 1 (continue) / 0 (impact,
// plane+fraction+endpos written). `dist_epsilon` is normally 0.03125f.
int PM_RecursiveHullCheck(const hull_t* hull, int num, float p1f, float p2f,
                          const vec3_t p1, const vec3_t p2, pmtrace_t* trace,
                          float dist_epsilon);

// PM_PlayerTrace - pmovetst.cpp:356/558. Traces `start`->`end` against all
// physents (world first), keeping the strictly-nearest hit.
pmtrace_t PM_PlayerTrace(const PmContext* pm, const vec3_t start, const vec3_t end,
                         int traceFlags, int ignore_pe);
} // namespace pw
