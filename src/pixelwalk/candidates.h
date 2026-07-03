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
    char method;       // 'A' convex floor edge, 'B' floor-plane x wall-plane
};

// Enumerates seams from solid-model floor/wall faces.
std::vector<Seam> EnumerateSeams(const Map& map, const WorldModels& wm, bool verbose);

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
