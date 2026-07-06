// clipdecomp.h - decompile a BSP clip hull into convex solid brushes with planar
// faces + windings. Reimplements the core of ericw-tools common/decompile.cc
// (clipnode path): a plane-accumulating tree walk that snapshots the outward
// half-space set at each SOLID leaf, then windows each plane into a convex face by
// clipping a base polygon against the others.
//
// We only ever decompile the PLAYER hulls - hull 1 (standing) and hull 3 (duck);
// hull 2 (monster) is unused. It's a per-root call, so the caller picks the hull.
//
// The clipnode planes are the HULL-EXPANDED collision planes (the exact surfaces
// the point trace hits), so faces come out in the per-hull expanded frame - which
// is precisely where a pixelwalk's floor/wall seam lives.
#pragma once
#include <array>
#include <vector>
#include "engine_types.h"

namespace pw {

struct DecompFace {
    std::array<float,3> n;                  // outward plane normal (solid: dot(x,n) <= d)
    float d;                                // outward plane dist
    std::vector<std::array<float,3>> w;     // convex winding, wound CCW about n
};

struct DecompBrush {
    std::vector<DecompFace> faces;
    int contents = 0;                       // CONTENTS_SOLID (-2), water, etc.
};

// Decompile the clip hull rooted at clipnode index `root` (= model.headnode[hull],
// hull in {1,3}) into convex solid brushes. `mins`/`maxs` bound the model; 6 seed
// planes at those bounds (padded) close otherwise-unbounded clip solids. Planes
// stay in the hull's expanded frame. Only non-EMPTY leaves become brushes.
std::vector<DecompBrush> DecompileClipHull(
    const dclipnode_t* clipnodes, int numclipnodes, const mplane_t* planes,
    int root, const float mins[3], const float maxs[3]);

// True if point p is inside brush b (behind every outward face, within eps).
bool PointInBrush(const DecompBrush& b, const float p[3], float eps);
} // namespace pw
