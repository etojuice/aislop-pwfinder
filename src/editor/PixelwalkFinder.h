#pragma once

#include "bsptypes.h"
#include <vector>

class Bsp;

// One candidate pixelwalk seam: the edge where two perpendicular axis-aligned
// collision-hull faces from two different submodels meet. Computed from the
// clipnode tree; visible-face data is never consulted.
struct PixelwalkSeam
{
	vec3 p1;
	vec3 p2;
	int hull;       // 1 (standing) or 3 (crouching)
	int low_idx;    // smaller submodel index of the pair (0 = worldmodel)
	int high_idx;   // larger submodel index of the pair
};

namespace PixelwalkFinder
{
	// Appends candidate pixelwalk seams for the given player hull to `outSeams`.
	// `hull` must be 1 (standing) or 3 (crouching) — these are the only player
	// hulls in pmove. Reads collision data only (clipnode tree via
	// Bsp::get_model_leaf_volume_cuts), not visible-face lumps.
	void findSeams(Bsp* map, int hull, std::vector<PixelwalkSeam>& outSeams);
}
