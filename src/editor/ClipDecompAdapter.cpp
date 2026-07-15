// ClipDecompAdapter.cpp - fork-WORLD side of the clipdecomp bridge. Reads the
// editor's Bsp (BSPCLIPNODE32/BSPPLANE/BSPMODEL) into neutral POD arrays and hands
// them to RunClipDecomp (ClipDecompShim.cpp, the pw-only TU). Includes NO pw
// headers, so the CONTENTS_*/MAX_MAP_HULLS macro-vs-enum collision cannot happen.
#include "ClipDecompAdapter.h"
#include "Bsp.h"
#include "bsptypes.h"
#include "log.h"

std::vector<ClipDecompOutBrush> DecompileEditorClipHull(Bsp* map, int modelIdx, int hullIdx)
{
	std::vector<ClipDecompOutBrush> empty;

	if (!map || modelIdx < 0 || modelIdx >= map->modelCount)
		return empty;
	if (hullIdx < 1 || hullIdx >= MAX_MAP_HULLS)
		return empty;

	// pw::dclipnode_t children are int16. Standard GoldSrc maps cap clipnodes at
	// 32767; 32-bit-extended maps can exceed it. Fall back to Clipper if so.
	if (map->clipnodeCount > 32767)
	{
		print_log("clipdecomp: {} clipnodes exceeds int16 range; using Clipper for model {} hull {}\n",
			map->clipnodeCount, modelIdx, hullIdx);
		return empty;
	}

	std::vector<ClipDecompNode> nodes((size_t)map->clipnodeCount);
	for (int i = 0; i < map->clipnodeCount; i++)
	{
		const BSPCLIPNODE32& src = map->clipnodes[i];
		for (int c = 0; c < 2; c++)
		{
			if (src.iChildren[c] < -32768 || src.iChildren[c] > 32767)
			{
				print_log("clipdecomp: clipnode child {} out of int16 range; using Clipper for model {} hull {}\n",
					src.iChildren[c], modelIdx, hullIdx);
				return empty;
			}
			nodes[i].children[c] = src.iChildren[c];
		}
		nodes[i].planenum = src.iPlane;
	}

	std::vector<ClipDecompPlane> planes((size_t)map->planeCount);
	for (int i = 0; i < map->planeCount; i++)
	{
		const BSPPLANE& src = map->planes[i];
		planes[i].normal[0] = src.vNormal.x;
		planes[i].normal[1] = src.vNormal.y;
		planes[i].normal[2] = src.vNormal.z;
		planes[i].dist = src.fDist;
	}

	const BSPMODEL& model = map->models[modelIdx];
	int root = model.iHeadnodes[hullIdx];

	float mins[3] = { model.nMins.x, model.nMins.y, model.nMins.z };
	float maxs[3] = { model.nMaxs.x, model.nMaxs.y, model.nMaxs.z };

	return RunClipDecomp(nodes, planes, root, mins, maxs);
}
