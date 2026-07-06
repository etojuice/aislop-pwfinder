#include "hull.h"

namespace pw {

// PM_HullPointContents - faithful port of ReHLDS/rehlds/engine/pmovetst.cpp:104.
// The engine Sys_Error()s on an out-of-range node; a scanner must not abort on a
// malformed map, so we defensively treat an over-range index as SOLID.
int PM_HullPointContents(const hull_t* hull, int num, const vec3_t p) {
    if (hull->firstclipnode >= hull->lastclipnode)
        return CONTENTS_EMPTY;

    while (num >= 0) {
        if (num > hull->lastclipnode)
            return CONTENTS_SOLID;               // defensive (engine: Sys_Error)
        const dclipnode_t* node  = &hull->clipnodes[num];
        const mplane_t*    plane = &hull->planes[node->planenum];

        float d;
        if (plane->type >= 3)
            d = DotProduct(p, plane->normal) - plane->dist;
        else
            d = p[plane->type] - plane->dist;

        num = (d >= 0.0f) ? node->children[0] : node->children[1];
    }
    return num;   // negative => CONTENTS_*
}

// Per-hull-index clip boxes baked by the compiler (model.cpp:1102-1136).
// Index 0 unused (point/node tree, not built here).
static const float kClipMins[MAX_MAP_HULLS][3] = {
    {   0,   0,   0 },
    { -16, -16, -36 },   // hull 1: standing
    { -32, -32, -32 },   // hull 2: large
    { -16, -16, -18 },   // hull 3: duck
};
static const float kClipMaxs[MAX_MAP_HULLS][3] = {
    {  0,  0,  0 },
    { 16, 16, 36 },
    { 32, 32, 32 },
    { 16, 16, 18 },
};

std::vector<model_t> BuildModels(const Map& map) {
    std::vector<model_t> models(map.models.size());
    const int lastclip = static_cast<int>(map.clipnodes.size()) - 1;

    for (size_t mi = 0; mi < map.models.size(); ++mi) {
        const BModel& bm = map.models[mi];
        model_t& mo = models[mi];

        for (int h = 0; h < MAX_MAP_HULLS; ++h) {
            hull_t& hull = mo.hulls[h];
            hull.clipnodes     = map.clipnodes.data();
            hull.planes        = map.planes.data();
            hull.firstclipnode = bm.headnode[h];
            hull.lastclipnode  = lastclip;
            for (int k = 0; k < 3; ++k) {
                hull.clip_mins[k] = kClipMins[h][k];
                hull.clip_maxs[k] = kClipMaxs[h][k];
            }
            // Hull 0 is the visible node tree (not built); hulls 1..3 are real
            // iff their root is a genuine clipnode index. A negative root means
            // the whole submodel is empty for that hull (handled by the num<0
            // branch in the trace, so we still leave firstclipnode as-is).
            mo.has_hull[h] = (h != 0) && (bm.headnode[h] >= 0);
        }
    }
    return models;
}
} // namespace pw
