// hull.h - point-contents classification + runtime collision-model build.
#pragma once
#include <vector>
#include "bsp.h"
#include "engine_types.h"

namespace pw {

// PM_HullPointContents - pmovetst.cpp:104. Walks the clipnode tree from `num`
// and returns the CONTENTS_* value of the leaf containing point `p`.
int PM_HullPointContents(const hull_t* hull, int num, const vec3_t p);

// Builds one model_t per BSP submodel, wiring hulls[1..3] to share map's global
// clipnode/plane arrays with per-model roots (headnode) and the fixed per-hull
// clip boxes (Mod_LoadClipnodes, model.cpp:1087). Returned models hold pointers
// INTO `map` — `map` must outlive them. Hull 0 (point/node tree) is not built.
std::vector<model_t> BuildModels(const Map& map);
} // namespace pw
