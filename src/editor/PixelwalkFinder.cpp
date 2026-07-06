#include "PixelwalkFinder.h"

// Only pixelwalk-finder-2 headers are pulled in here (never the fork's Bsp.h /
// bsptypes.h), so the pw:: type/macro world stays isolated from the fork's.
// These are reached by relative path; src/pixelwalk is intentionally NOT on the
// include path (see CMakeLists.txt) so pw's "bsp.h" can't shadow the fork's
// "Bsp.h" and vice-versa.
#include "../pixelwalk/bsp.h"
#include "../pixelwalk/hull.h"
#include "../pixelwalk/world.h"
#include "../pixelwalk/candidates.h"
#include "../pixelwalk/finder.h"

#include <cmath>

bool PixelwalkFinder::findPixelwalks(const std::string& bspPath, std::vector<PixelwalkResult>& out)
{
	// Mirrors pixelwalk-finder-2/src/main.cpp (single clip-brush detector now;
	// --method is a deprecated no-op upstream).
	pw::Map map;
	std::string err;
	if (!pw::LoadBsp(bspPath, map, err))
		return false;   // not a BSP v30 map

	std::vector<pw::model_t> models = pw::BuildModels(map);
	pw::WorldModels wm = pw::BuildWorld(map, models);
	std::vector<pw::Seam> seams = pw::EnumerateSeams(map, wm, false);

	pw::FinderConfig cfg;             // defaults from finder.h
	cfg.standing = true;             // --hull both
	cfg.duck = true;
	cfg.zones = true;                // --zones: group collinear finds into from->to spans
	cfg.min_samples = 2;             // --min-samples=2: keep finds/zones with >=2 sub-pixel hits

	std::vector<pw::Find> finds = pw::RunFinder(map, wm, seams, cfg);

	for (const auto& f : finds)
	{
		PixelwalkResult r;
		r.pos = vec3(f.pos[0], f.pos[1], f.pos[2]);
		r.to = vec3(f.to[0], f.to[1], f.to[2]);
		r.length = f.length;
		r.approach = vec3(f.approach[0], f.approach[1], f.approach[2]);
		r.yaw = std::atan2(f.approach[1], f.approach[0]) * 180.0f / 3.14159265358979f;
		if (r.yaw < 0.0f)
			r.yaw += 360.0f;
		r.usehull = f.usehull;
		r.hang_frames = (int)f.advanced;
		r.samples = f.cluster_size;
		r.floor_model = f.floor_model;
		r.wall_model = f.wall_model;
		out.push_back(r);
	}
	return true;
}
