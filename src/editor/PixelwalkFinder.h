#pragma once

#include "vectors.h"      // fork's vec3
#include <vector>
#include <string>

// A single detected pixelwalk position, produced by pixelwalk-finder-2's
// "sim" detector (full PM_PlayerMove walk-into-wall reproduction). All vectors
// are in raw GoldSrc map coordinates (Z up); the renderer flips to GL space.
struct PixelwalkResult
{
	vec3  pos;         // resting hull-center origin (= amx_setpos x y z)
	vec3  approach;    // unit horizontal dir into the wall (z=0); the +forward to hold
	float yaw;         // degrees, CS convention (0=+X, 90=+Y), normalized [0,360)
	int   usehull;     // 0 = standing, 1 = duck
	int   hang_frames; // frames the hull hung on the pixel (robustness metric)
	int   samples;     // sub-pixel hits merged into this find
};

namespace PixelwalkFinder
{
	// Runs pixelwalk-finder-2 with --method sim --hull both (all other options
	// default) on the BSP file at bspPath, appending detected positions to
	// `out`. Returns false if the file can't be loaded as BSP v30 (the sim
	// core's loader is v30-only). Detection is engine-faithful 32-bit float,
	// so results match the standalone pwfinder tool exactly.
	bool findPixelwalks(const std::string& bspPath, std::vector<PixelwalkResult>& out);
}
