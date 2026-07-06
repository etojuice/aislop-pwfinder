// bsp.h - GoldSrc BSP v30 loader (raw lump data → Map).
#pragma once
#include <array>
#include <string>
#include <vector>
#include "engine_types.h"

namespace pw {

// On-disk submodel (dmodel_t). Model 0 = worldspawn; 1..N = brush entities.
struct BModel {
    float mins[3], maxs[3], origin[3];
    int   headnode[MAX_MAP_HULLS];  // [0]=node tree, [1..3]=clipnode roots
    int   visleafs, firstface, numfaces;
};

// On-disk face (dface_t) - only the fields we use.
struct BFace {
    int planenum;   // index into planes
    int side;       // 0 => plane normal as-is, else flipped
    int firstedge;  // index into surfedges
    int numedges;
    int texinfo;    // index into texinfo (-> miptex name)
};

struct BEdge { unsigned short v[2]; };

struct Map {
    int version = 0;
    std::vector<mplane_t>    planes;      // shared by all hulls
    std::vector<dclipnode_t> clipnodes;   // hulls index into this
    std::vector<BModel>      models;
    std::vector<BFace>       faces;
    std::vector<BEdge>       edges;
    std::vector<int>         surfedges;
    std::vector<std::array<float,3>> vertices;
    std::vector<int>         face_tex;     // per-face -> index into texnames (-1 if none)
    std::vector<std::string> texnames;     // miptex names, lowercased
    std::string              entities;     // raw entity lump text
};

// Loads `path` into `out`. Returns false and fills `err` on any problem
// (unreadable, not v30, malformed lump). All numeric reads are little-endian.
bool LoadBsp(const std::string& path, Map& out, std::string& err);
} // namespace pw
