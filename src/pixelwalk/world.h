// world.h - entity-lump parsing, per-model solidity, and physent construction.
#pragma once
#include <array>
#include <string>
#include <vector>
#include "bsp.h"
#include "engine_types.h"

namespace pw {

// A solid brush entity ready to trace, with its world-space bbox for culling.
struct SolidBrush {
    physent_t pe;
    float wmins[3], wmaxs[3];
};

struct WorldModels {
    physent_t world{};                         // model 0 (SOLID_BSP, origin 0)
    std::vector<SolidBrush> brushes;           // solid brush ents, sorted by model index

    // Per map-model-index metadata (size == map.models.size()):
    std::vector<char>        model_solid;       // 1 if traced as SOLID_BSP
    std::vector<std::string> model_classname;
    std::vector<std::array<float,3>> model_origin;
};

// Parses map.entities, classifies solidity, and wires physents to `models`
// (which must outlive the result).
WorldModels BuildWorld(const Map& map, const std::vector<model_t>& models);

// Fills `out` with the physents to trace near world-space AABB [qmins,qmaxs]:
// always world at index 0, then every solid brush whose (margin-expanded) bbox
// overlaps the query, in model-index order. Mirrors the engine only tracing
// nearby entities. `out` is cleared first.
void SelectPhysents(const WorldModels& wm, const float qmins[3], const float qmaxs[3],
                    std::vector<physent_t>& out);
} // namespace pw
