// ClipDecompShim.cpp - pw-WORLD ONLY. The single editor TU that includes the
// pixelwalk clipnode decompiler. Includes NO fork headers (Bsp.h/bsptypes.h),
// so the CONTENTS_*/MAX_MAP_HULLS macro-vs-enum collision cannot happen here.
#include "ClipDecompShim.h"
#include "../pixelwalk/clipdecomp.h"

std::vector<ClipDecompOutBrush> RunClipDecomp(
	const std::vector<ClipDecompNode>& nodes,
	const std::vector<ClipDecompPlane>& planes,
	int root, const float mins[3], const float maxs[3])
{
	std::vector<pw::dclipnode_t> cn(nodes.size());
	for (size_t i = 0; i < nodes.size(); i++)
	{
		cn[i].planenum = nodes[i].planenum;
		cn[i].children[0] = (short)nodes[i].children[0];
		cn[i].children[1] = (short)nodes[i].children[1];
	}

	std::vector<pw::mplane_t> pl(planes.size());
	for (size_t i = 0; i < planes.size(); i++)
	{
		pl[i].normal[0] = planes[i].normal[0];
		pl[i].normal[1] = planes[i].normal[1];
		pl[i].normal[2] = planes[i].normal[2];
		pl[i].dist = planes[i].dist;
		pl[i].type = 0;
		pl[i].signbits = 0;
		pl[i].pad[0] = pl[i].pad[1] = 0;
	}

	std::vector<pw::DecompBrush> brushes = pw::DecompileClipHull(
		cn.data(), (int)cn.size(), pl.data(), root, mins, maxs);

	std::vector<ClipDecompOutBrush> out;
	out.reserve(brushes.size());
	for (const pw::DecompBrush& b : brushes)
	{
		ClipDecompOutBrush ob;
		ob.contents = b.contents;
		ob.faces.reserve(b.faces.size());
		for (const pw::DecompFace& f : b.faces)
		{
			ClipDecompOutFace of;
			of.n[0] = f.n[0];
			of.n[1] = f.n[1];
			of.n[2] = f.n[2];
			of.d = f.d;
			of.w = f.w;   // both are std::vector<std::array<float,3>>
			ob.faces.push_back(std::move(of));
		}
		out.push_back(std::move(ob));
	}
	return out;
}
