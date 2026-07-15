// ClipDecompShim.h - NEUTRAL interface between the fork world (Bsp.h/bsptypes.h)
// and the pixelwalk world (pw:: clipdecomp / engine_types.h).
//
// The two worlds must NEVER meet in one translation unit: the fork declares
// CONTENTS_* / MAX_MAP_HULLS as preprocessor macros, while pw declares CONTENTS_*
// as enum values inside `namespace pw` - so if bsptypes.h is seen before
// engine_types.h the preprocessor rewrites pw's `enum { CONTENTS_SOLID = -2 }`
// into `enum { -2 = -2 }` and the build breaks (see the note in PixelwalkFinder.cpp).
//
// This header therefore includes NEITHER world - only primitive/std types. The
// fork side (ClipDecompAdapter.cpp) fills the POD inputs; the pw side
// (ClipDecompShim.cpp) is the sole TU that includes clipdecomp.h.
#pragma once
#include <array>
#include <vector>

struct ClipDecompNode {
	int planenum;
	int children[2];   // < 0 => CONTENTS_* leaf, >= 0 => child clipnode index
};

struct ClipDecompPlane {
	float normal[3];
	float dist;
};

struct ClipDecompOutFace {
	float n[3];                                  // outward normal (solid: dot(x,n) <= d)
	float d;                                     // outward plane dist
	std::vector<std::array<float, 3>> w;         // convex winding, CCW about n
};

struct ClipDecompOutBrush {
	std::vector<ClipDecompOutFace> faces;
	int contents;                                // CONTENTS_SOLID (-2), water, etc.
};

// Runs pw::DecompileClipHull on neutral POD inputs. `root` is model.iHeadnodes[hull];
// `mins`/`maxs` bound the model. Implemented in ClipDecompShim.cpp (pw world only).
std::vector<ClipDecompOutBrush> RunClipDecomp(
	const std::vector<ClipDecompNode>& nodes,
	const std::vector<ClipDecompPlane>& planes,
	int root, const float mins[3], const float maxs[3]);
