// candidates.h - floor/wall seam enumeration (where pixelwalk can occur).
#pragma once
#include <array>
#include <unordered_map>
#include <vector>
#include "bsp.h"
#include "world.h"

namespace pw {

// A candidate seam: a horizontal segment [a,b] at height z where a floor top
// meets a (near-)vertical wall. `outn` is the wall's outward horizontal normal
// (player stands on the +outn side and walks toward -outn into the wall).
struct Seam {
    std::array<float,3> a, b;
    float z;
    std::array<float,3> outn;
    int  floor_model;
    int  wall_model;   // -1 when unknown (Method A / detector fills actual)
    char method;       // 'A' convex floor edge, 'B' floor-plane x wall-plane,
                       // 'C' clipnode floor-plane x wall-plane
    int  usehull_mask = 0x3; // bit0 = standing, bit1 = duck
    // Floor face's world-space plane (normal + dist), so the finder can compute the
    // real floor height at any xy: z = (fd - fn.x*x - fn.y*y)/fn.z. For flat floors
    // this is a constant (== z); for SLOPES it varies along the seam.
    std::array<float,3> fn{0,0,1};
    float fd = 0.0f;
    bool  slope = false;   // floor normal.z in (0, ~0.99): tilted floor/slide plane
    bool  slide = false;   // slope with 0 < normal.z < FLOOR_NZ (not engine-walkable)
};

// Enumerates seams from solid-model floor/wall faces.
std::vector<Seam> EnumerateSeams(const Map& map, const WorldModels& wm, bool verbose);

// Enumerates seams from the real hull clipnodes. This is hull-specific and
// includes flat, floor-walk slope, and slide-walk slope planes.
std::vector<Seam> EnumerateClipnodeSeams(const Map& map, const WorldModels& wm, bool verbose);

// Clipnode decompile candidate source, falling back to the old face generator if
// clipnode decompilation yields none.
std::vector<Seam> EnumerateClipnodeDecompileSeams(const Map& map, const WorldModels& wm, bool verbose);

// Index of real (unexpanded) solid floor-face polygons, for the over-void gate:
// a genuine pixelwalk stand is over the VOID (no real floor under the hull
// center), unlike a benign concave corner which has floor beneath.
struct FloorFace {
    std::vector<std::array<float,2>> poly;  // world-space xy loop
    float z;
    int   model;
};
struct FloorIndex {
    std::vector<FloorFace> faces;
    std::unordered_map<long long, std::vector<int>> grid;
    float cell = 64.0f;
    // True if (x,y) lies over a solid floor face whose height is within ztol of z.
    bool overFloor(float x, float y, float z, float ztol) const;
};
FloorIndex BuildFloorIndex(const Map& map, const WorldModels& wm);
} // namespace pw
