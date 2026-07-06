// candidates.h - pixelwalk candidate seams, enumerated from CLIPNODE-decompiled
// collision brushes (not visual faces), in the per-stance HULL-EXPANDED frame (the
// exact surface the point trace hits) - so a seam already sits at hull-center height
// where the pixelwalk lives. Two kinds: floor-face EDGES over the void / against a
// wall, and flat FLOOR x WALL crossings (a wall on a continuous floor).
#pragma once
#include <array>
#include <vector>
#include "bsp.h"
#include "world.h"

namespace pw {

// A candidate seam: a segment [a,b] in the EXPANDED frame. `outn` is the horizontal
// normal toward the floor interior (player stands on the +outn side and walks toward
// -outn over the edge / into a wall). `fn`/`fd` are the EXPANDED floor plane, so
// floorZAt gives the hull-center height at any xy: z = (fd - fn.x*x - fn.y*y)/fn.z.
struct Seam {
    std::array<float,3> a, b;
    std::array<float,3> outn;
    int  floor_model;
    int  wall_model;   // -1: detector fills the actual pressed wall from the catch
    std::array<float,3> fn{0,0,1};
    float fd = 0.0f;
    bool  slope = false;   // floor normal.z in [FLOOR_NZ, ~0.99): a tilted (ramp) floor
    int  usehull = 0;      // stance this seam was decompiled for: 0 standing (hull 1),
                           //   1 duck (hull 3). Detection tests only this stance.
};

// Enumerates seams by decompiling each solid model's player clip hulls (1 & 3) into
// convex brushes and emitting their floor-face edges + floor x wall crossings.
std::vector<Seam> EnumerateSeams(const Map& map, const WorldModels& wm, bool verbose);
} // namespace pw
