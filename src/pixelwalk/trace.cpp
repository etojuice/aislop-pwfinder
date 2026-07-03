#include "trace.h"
#include "hull.h"
#include <cmath>
#include <cstring>

namespace pw {

// ---- angle math (mathlib.cpp), for rotated SOLID_BSP brush entities.
// AngleVectors lives in vec.h; the transpose (used to un-rotate the hit plane)
// is here. ----
namespace {
constexpr float DEG2RAD = 3.14159265358979323846f * 2.0f / 360.0f;

void AngleVectorsTranspose(const vec3_t angles, vec3_t forward, vec3_t right, vec3_t up) {
    float a, sr, sp, sy, cr, cp, cy;
    a = angles[1] * DEG2RAD; sy = std::sin(a); cy = std::cos(a);
    a = angles[0] * DEG2RAD; sp = std::sin(a); cp = std::cos(a);
    a = angles[2] * DEG2RAD; sr = std::sin(a); cr = std::cos(a);
    forward[0] = cp * cy;
    forward[1] = sr * sp * cy - cr * sy;
    forward[2] = cr * sp * cy + sr * sy;
    right[0] = cp * sy;
    right[1] = sr * sp * sy + cr * cy;
    right[2] = cr * sp * sy - sr * cy;
    up[0] = -sp;
    up[1] = sr * cp;
    up[2] = cr * cp;
}
} // namespace

// PM_RecursiveHullCheck - faithful port of pmovetst.cpp:614 (default variant),
// with DIST_EPSILON exposed as a parameter for the epsilon-differential oracle.
int PM_RecursiveHullCheck(const hull_t* hull, int num, float p1f, float p2f,
                          const vec3_t p1, const vec3_t p2, pmtrace_t* trace,
                          float DIST_EPSILON) {
    if (num < 0) {
        if (num == CONTENTS_SOLID) {
            trace->startsolid = 1;
        } else {
            trace->allsolid = 0;
            if (num == CONTENTS_EMPTY) trace->inopen = 1;
            else                       trace->inwater = 1;
        }
        return 1;
    }

    if (hull->firstclipnode >= hull->lastclipnode) {
        trace->allsolid = 0;
        trace->inopen = 1;
        return 1;
    }

    const dclipnode_t* node  = &hull->clipnodes[num];
    const mplane_t*    plane = &hull->planes[node->planenum];

    float t1, t2;
    if (plane->type >= 3) {
        t1 = DotProduct(p1, plane->normal) - plane->dist;
        t2 = DotProduct(p2, plane->normal) - plane->dist;
    } else {
        t1 = p1[plane->type] - plane->dist;
        t2 = p2[plane->type] - plane->dist;
    }

    if (t1 >= 0.0f && t2 >= 0.0f)
        return PM_RecursiveHullCheck(hull, node->children[0], p1f, p2f, p1, p2, trace, DIST_EPSILON);

    float midf;
    if (t1 >= 0.0f) {
        midf = t1 - DIST_EPSILON;
    } else {
        if (t2 < 0.0f)
            return PM_RecursiveHullCheck(hull, node->children[1], p1f, p2f, p1, p2, trace, DIST_EPSILON);
        midf = t1 + DIST_EPSILON;
    }
    midf = midf / (t1 - t2);
    if (midf >= 0.0f) {
        if (midf > 1.0f) midf = 1.0f;
    } else {
        midf = 0.0f;
    }

    float pdif = p2f - p1f;
    float frac = pdif * midf + p1f;
    vec3_t mid;
    mid[0] = (p2[0] - p1[0]) * midf + p1[0];
    mid[1] = (p2[1] - p1[1]) * midf + p1[1];
    mid[2] = (p2[2] - p1[2]) * midf + p1[2];

    int side = (t1 >= 0.0f) ? 0 : 1;

    if (!PM_RecursiveHullCheck(hull, node->children[side], p1f, frac, p1, mid, trace, DIST_EPSILON))
        return 0;

    if (PM_HullPointContents(hull, node->children[side ^ 1], mid) != CONTENTS_SOLID)
        return PM_RecursiveHullCheck(hull, node->children[side ^ 1], frac, p2f, mid, p2, trace, DIST_EPSILON);

    if (trace->allsolid)
        return 0;

    if (side) {
        trace->plane.normal[0] = -plane->normal[0];
        trace->plane.normal[1] = -plane->normal[1];
        trace->plane.normal[2] = -plane->normal[2];
        trace->plane.dist = -plane->dist;
    } else {
        trace->plane.normal[0] = plane->normal[0];
        trace->plane.normal[1] = plane->normal[1];
        trace->plane.normal[2] = plane->normal[2];
        trace->plane.dist = plane->dist;
    }

    // Back up until `mid` is out of solid (or we cross fraction 0). The engine
    // does `midf = (float)(midf - 0.05)` — the 0.05 is a double literal, so the
    // subtraction is done in double then rounded to float; match that exactly.
    while (PM_HullPointContents(hull, hull->firstclipnode, mid) == CONTENTS_SOLID) {
        midf = (float)((double)midf - 0.05);
        if (midf < 0.0f) {
            trace->fraction = frac;
            VectorCopy(mid, trace->endpos);
            return 0;   // engine: Con_DPrintf("Trace backed up past 0.0")
        }
        frac = pdif * midf + p1f;
        mid[0] = (p2[0] - p1[0]) * midf + p1[0];
        mid[1] = (p2[1] - p1[1]) * midf + p1[1];
        mid[2] = (p2[2] - p1[2]) * midf + p1[2];
    }

    trace->fraction = frac;
    VectorCopy(mid, trace->endpos);
    return 0;
}

// _PM_PlayerTrace - faithful port of pmovetst.cpp:356 for BSP world/brush
// entities only (studio/box paths omitted; the maps we scan have none).
pmtrace_t PM_PlayerTrace(const PmContext* pm, const vec3_t start, const vec3_t end,
                         int traceFlags, int ignore_pe) {
    pmtrace_t trace;
    std::memset(&trace, 0, sizeof trace);
    trace.fraction = 1.0f;
    trace.ent = -1;
    VectorCopy(end, trace.endpos);

    for (int i = 0; i < pm->numphysent; ++i) {
        const physent_t* pe = &pm->physents[i];

        if (i > 0 && (traceFlags & PM_WORLD_ONLY))
            break;
        if (ignore_pe != -1 && i == ignore_pe)
            continue;
        if (pe->model && !pe->solid && pe->skin)
            continue;
        if ((traceFlags & PM_GLASS_IGNORE) && pe->rendermode)
            continue;
        if (!pe->model)
            continue;                       // only BSP models are handled

        const int hi = HullIndexForUsehull(pm->usehull);
        const hull_t* hull = &pe->model->hulls[hi];

        vec3_t offset;
        offset[0] = hull->clip_mins[0] - PLAYER_MINS[pm->usehull][0] + pe->origin[0];
        offset[1] = hull->clip_mins[1] - PLAYER_MINS[pm->usehull][1] + pe->origin[1];
        offset[2] = hull->clip_mins[2] - PLAYER_MINS[pm->usehull][2] + pe->origin[2];

        vec3_t start_l, end_l;
        VectorSubtract(start, offset, start_l);
        VectorSubtract(end, offset, end_l);

        bool rotated = (pe->solid == SOLID_BSP) &&
                       (pe->angles[0] != 0.0f || pe->angles[1] != 0.0f || pe->angles[2] != 0.0f);
        if (rotated) {
            vec3_t f, r, u;
            AngleVectors(pe->angles, f, r, u);
            vec3_t ts = { start_l[0], start_l[1], start_l[2] };
            start_l[0] =  DotProduct(f, ts);
            start_l[1] = -DotProduct(r, ts);
            start_l[2] =  DotProduct(u, ts);
            vec3_t te = { end_l[0], end_l[1], end_l[2] };
            end_l[0] =  DotProduct(f, te);
            end_l[1] = -DotProduct(r, te);
            end_l[2] =  DotProduct(u, te);
        }

        pmtrace_t total;
        std::memset(&total, 0, sizeof total);
        VectorCopy(end, total.endpos);
        total.fraction = 1.0f;
        total.allsolid = 1;

        PM_RecursiveHullCheck(hull, hull->firstclipnode, 0.0f, 1.0f, start_l, end_l, &total,
                              pm->dist_epsilon);

        if (total.allsolid)
            total.startsolid = 1;
        if (total.startsolid)
            total.fraction = 0.0f;

        if (total.fraction != 1.0f) {
            if (rotated) {
                vec3_t f, r, u;
                AngleVectorsTranspose(pe->angles, f, r, u);
                vec3_t tmp = { total.plane.normal[0], total.plane.normal[1], total.plane.normal[2] };
                total.plane.normal[0] = DotProduct(f, tmp);
                total.plane.normal[1] = DotProduct(r, tmp);
                total.plane.normal[2] = DotProduct(u, tmp);
            }
            total.endpos[0] = (end[0] - start[0]) * total.fraction + start[0];
            total.endpos[1] = (end[1] - start[1]) * total.fraction + start[1];
            total.endpos[2] = (end[2] - start[2]) * total.fraction + start[2];
        }

        if (total.fraction < trace.fraction) {
            std::memcpy(&trace, &total, sizeof trace);
            trace.ent = i;
        }
    }

    return trace;
}
} // namespace pw
