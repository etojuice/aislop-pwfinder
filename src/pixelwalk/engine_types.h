// engine_types.h - faithful in-memory mirrors of the GoldSrc/ReHLDS collision
// structs, so the ported trace/movement code reads line-for-line like the engine.
//
// These are NOT the on-disk BSP layouts (see bsp.cpp for those); they are the
// runtime shapes from:
//   ReHLDS/rehlds/public/rehlds/model.h  (mplane_t, hull_t)
//   ReHLDS/rehlds/public/rehlds/bspfile.h (dclipnode_t, CONTENTS_*, MAX_MAP_HULLS)
//   ReHLDS/rehlds/common/pmtrace.h        (pmtrace_t, pmplane_t)
//   ReGameDLL_CS/regamedll/pm_shared/pm_defs.h (physent_t)
#pragma once
#include "vec.h"

namespace pw {

#define MAX_MAP_HULLS 4

// Leaf contents (negative clipnode children). bspfile.h / bsptypes.h.
enum {
    CONTENTS_EMPTY = -1,
    CONTENTS_SOLID = -2,
    CONTENTS_WATER = -3,
    CONTENTS_SLIME = -4,
    CONTENTS_LAVA  = -5,
    CONTENTS_SKY   = -6,
};

// physent solid types (const.h). We only ever build SOLID_BSP world/brush ents.
enum {
    SOLID_NOT     = 0,
    SOLID_TRIGGER = 1,
    SOLID_BBOX    = 2,
    SOLID_SLIDEBOX= 3,
    SOLID_BSP     = 4,
};

// mplane_t - model.h:63. `type` (<3 == axis-aligned fast path) is copied from
// disk; signbits recomputed on load (unused by the trace's dist test, kept for
// fidelity).
struct mplane_t {
    vec3_t        normal;
    float         dist;
    unsigned char type;
    unsigned char signbits;
    unsigned char pad[2];
};

// dclipnode_t - bspfile.h:123. children[] < 0 are CONTENTS_* leaves; children on
// disk are int16, but in memory we keep the engine's `short`.
struct dclipnode_t {
    int   planenum;
    short children[2];
};

// hull_t - model.h:206. clipnodes/planes point at the map-global shared arrays;
// firstclipnode = this (model,hull)'s root; lastclipnode = numclipnodes-1.
struct hull_t {
    const dclipnode_t *clipnodes;
    const mplane_t    *planes;
    int   firstclipnode;
    int   lastclipnode;
    vec3_t clip_mins, clip_maxs;
};

// Lite brush model: only the collision hulls the finder needs.
struct model_t {
    hull_t hulls[MAX_MAP_HULLS];
    bool   has_hull[MAX_MAP_HULLS]; // false when headnode was a leaf/empty
};

// pmplane_t / pmtrace_t - pmtrace.h. qboolean == int.
struct pmplane_t {
    vec3_t normal;
    float  dist;
};

struct pmtrace_t {
    int      allsolid;
    int      startsolid;
    int      inopen, inwater;
    float    fraction;
    vec3_t   endpos;
    pmplane_t plane;
    int      ent;        // index of the physent hit (0 = world), -1 = none
    vec3_t   deltavelocity;
    int      hitgroup;
};

// physent_t - pm_defs.h (subset). For pixelwalk we only handle static BSP world +
// brush entities, so studiomodel/box paths are omitted.
struct physent_t {
    const model_t *model;   // brush model (null => skip)
    vec3_t origin;
    vec3_t angles;          // asserted zero for the maps we handle
    int    solid;           // SOLID_BSP for anything we trace
    int    skin;
    int    rendermode;
    int    info;            // original model index (for reporting)
};

// player_mins/player_maxs - ReHLDS/rehlds/engine/pmove.cpp:36. Indexed by
// pmove->usehull. usehull 0 standing (32x32x72), 1 duck (32x32x36),
// 2 point, 3 large. C++17 inline => single definition across TUs.
inline constexpr float PLAYER_MINS[MAX_MAP_HULLS][3] = {
    { -16.0f, -16.0f, -36.0f },
    { -16.0f, -16.0f, -18.0f },
    {   0.0f,   0.0f,   0.0f },
    { -32.0f, -32.0f, -32.0f },
};
inline constexpr float PLAYER_MAXS[MAX_MAP_HULLS][3] = {
    { 16.0f, 16.0f, 36.0f },
    { 16.0f, 16.0f, 18.0f },
    {  0.0f,  0.0f,  0.0f },
    { 32.0f, 32.0f, 32.0f },
};

// usehull -> model hull index. pmovetst.cpp:405 switch.
//   usehull 0 -> hulls[1], 1 -> hulls[3], 2 -> hulls[0], 3 -> hulls[2].
static inline int HullIndexForUsehull(int usehull) {
    switch (usehull) {
        case 1: return 3;
        case 2: return 0;
        case 3: return 2;
        default: return 1;
    }
}
} // namespace pw
