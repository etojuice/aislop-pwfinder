#include "PixelwalkFinder.h"
#include "Bsp.h"
#include "Clipper.h"
#include "util.h"
#include "log.h"
#include <algorithm>
#include <set>

namespace
{
	// One axis-aligned face polygon extracted from a brush submodel's
	// collision hull. The polygon's vertices share `pos` along `axis`.
	struct AxisAlignedPoly
	{
		int modelIdx;
		int axis;            // 0=X-normal, 1=Y-normal, 2=Z-normal
		int sign;            // +1 if normal points along +axis, -1 otherwise
		float pos;           // value of `axis` coordinate (= plane position along that axis)
		std::vector<vec3> verts;
	};

	float vecAxis(const vec3& v, int axis)
	{
		switch (axis)
		{
		case 0: return v.x;
		case 1: return v.y;
		default: return v.z;
		}
	}

	// Returns true if normal is within tolerance of an axis-aligned direction.
	// Sets out_axis to 0/1/2 and out_sign to +1/-1.
	bool isAxisAligned(const vec3& n, int& out_axis, int& out_sign)
	{
		const float kTol = 1e-3f;
		for (int a = 0; a < 3; a++)
		{
			float v = vecAxis(n, a);
			float other_a = vecAxis(n, (a + 1) % 3);
			float other_b = vecAxis(n, (a + 2) % 3);
			if (std::fabs(other_a) < kTol && std::fabs(other_b) < kTol &&
				std::fabs(std::fabs(v) - 1.0f) < kTol)
			{
				out_axis = a;
				out_sign = (v > 0.0f) ? 1 : -1;
				return true;
			}
		}
		return false;
	}

	// Convex-polygon clip: given an axis-aligned convex polygon (verts share
	// the constant value of `axis_normal`), return the [lo, hi] range along
	// `axis_along` where the line "axis_const = constVal" crosses the polygon.
	// Returns false if no intersection (or the line touches at a single point).
	bool clipPolyToLine(const std::vector<vec3>& verts,
		int axis_along, int axis_const, float constVal,
		float& outLo, float& outHi)
	{
		const float kEps = 1e-4f;
		std::vector<float> crossings;
		int n = (int)verts.size();
		for (int i = 0; i < n; i++)
		{
			const vec3& a = verts[i];
			const vec3& b = verts[(i + 1) % n];
			float au = vecAxis(a, axis_const);
			float bu = vecAxis(b, axis_const);
			float da = au - constVal;
			float db = bu - constVal;
			if (std::fabs(au - bu) < kEps)
			{
				// Edge runs parallel to slice. If it lies on the slice, the entire
				// edge contributes — but for a convex polygon we still pick up its
				// endpoints from the adjacent crossing edges. Skip here.
				continue;
			}
			if ((da >= -kEps && db <= kEps) || (da <= kEps && db >= -kEps))
			{
				if (da * db > kEps * kEps)
					continue;  // both strictly same side
				float t = da / (au - bu);
				if (t < 0.0f) t = 0.0f;
				if (t > 1.0f) t = 1.0f;
				float along = vecAxis(a, axis_along) + t * (vecAxis(b, axis_along) - vecAxis(a, axis_along));
				crossings.push_back(along);
			}
		}
		if (crossings.size() < 2)
			return false;
		std::sort(crossings.begin(), crossings.end());
		outLo = crossings.front();
		outHi = crossings.back();
		return (outHi - outLo) > kEps;
	}

	// Extract every axis-aligned visible face polygon from one submodel's
	// collision hull for the given player hull.
	void extractSubmodelPolys(Bsp* map, int modelIdx, int hull,
		std::vector<AxisAlignedPoly>& out)
	{
		std::vector<NodeVolumeCuts> solidNodes =
			map->get_model_leaf_volume_cuts(modelIdx, hull, CONTENTS_SOLID);
		if (solidNodes.empty())
			return;

		Clipper clipper;
		std::set<long long> dedup;  // quantized (axis, pos*64) per submodel

		for (auto& nvc : solidNodes)
		{
			CMesh mesh = clipper.clip(nvc.cuts);
			for (auto& face : mesh.faces)
			{
				if (!face.visible)
					continue;
				int axis = 0, sign = 0;
				if (!isAxisAligned(face.normal, axis, sign))
					continue;

				// Mirror BspRenderer: collect unique verts via edges, sort planar.
				std::set<int> uniqueVertIdx;
				for (int eIdx : face.edges)
				{
					for (int v = 0; v < 2; v++)
					{
						int vi = mesh.edges[eIdx].verts[v];
						if (mesh.verts[vi].visible)
							uniqueVertIdx.insert(vi);
					}
				}
				std::vector<vec3> faceVerts;
				faceVerts.reserve(uniqueVertIdx.size());
				for (int vi : uniqueVertIdx)
					faceVerts.push_back(mesh.verts[vi].pos);
				if (faceVerts.size() < 3)
					continue;
				faceVerts = getSortedPlanarVerts(faceVerts);
				if (faceVerts.size() < 3)
					continue;

				// Reject polygons whose vertices reach Clipper's max-cube
				// boundary (MAX_DIM=131072). Those are artifacts produced when
				// a SOLID leaf is not fully bounded by its cut planes — e.g.,
				// the worldmodel's "outside the world" region. They are not
				// real collision surfaces and would produce seams stretching
				// to ±131072 along arbitrary axes.
				const float kArtifactThreshold = 65536.0f;
				bool isArtifact = false;
				for (const vec3& v : faceVerts)
				{
					if (std::fabs(v.x) > kArtifactThreshold ||
						std::fabs(v.y) > kArtifactThreshold ||
						std::fabs(v.z) > kArtifactThreshold)
					{
						isArtifact = true;
						break;
					}
				}
				if (isArtifact)
					continue;

				float pos = vecAxis(faceVerts[0], axis);

				// Dedup faces sharing the same axis-aligned plane within this submodel
				// (multi-leaf decompositions can produce coincident exterior faces).
				long long key = ((long long)axis << 60) | (long long)std::llround(pos * 64.0f);
				if (dedup.count(key))
					continue;
				dedup.insert(key);

				AxisAlignedPoly p;
				p.modelIdx = modelIdx;
				p.axis = axis;
				p.sign = sign;
				p.pos = pos;
				p.verts = std::move(faceVerts);
				out.push_back(std::move(p));
			}
		}
	}
}

void PixelwalkFinder::findSeams(Bsp* map, int hull,
	std::vector<PixelwalkSeam>& outSeams)
{
	if (!map)
		return;

	// 1. Per-submodel polygon extraction (worldmodel + every brush entity).
	std::vector<std::vector<AxisAlignedPoly>> polysByModel(map->modelCount);
	size_t totalPolys = 0;
	for (int m = 0; m < map->modelCount; m++)
	{
		extractSubmodelPolys(map, m, hull, polysByModel[m]);
		totalPolys += polysByModel[m].size();
	}
	print_log("pixelwalk: hull {} extracted {} axis-aligned collision polys across {} submodels\n",
		hull, totalPolys, map->modelCount);

	// 2 & 3. Pair submodels and clip seam lines to actual polygon extents.
	std::set<long long> seamDedup;
	size_t startCount = outSeams.size();

	for (int low = 0; low < map->modelCount; low++)
	{
		if (polysByModel[low].empty())
			continue;
		for (int high = low + 1; high < map->modelCount; high++)
		{
			if (polysByModel[high].empty())
				continue;

			auto& A = polysByModel[low];
			auto& B = polysByModel[high];

			// Try ph from A + pv from B, then ph from B + pv from A.
			for (int dir = 0; dir < 2; dir++)
			{
				auto& src_h = (dir == 0) ? A : B;
				auto& src_v = (dir == 0) ? B : A;

				for (auto& ph : src_h)
				{
					if (ph.axis != 2)
						continue;  // need Z-normal (horizontal) plane
					for (auto& pv : src_v)
					{
						if (pv.axis != 0 && pv.axis != 1)
							continue;  // need X- or Y-normal (vertical) plane

						int u_axis = pv.axis;
						int along_axis = (u_axis == 0) ? 1 : 0;
						float z0 = ph.pos;
						float u0 = pv.pos;

						// Range along `along_axis` where ph contains the line at u_axis=u0.
						float h_lo, h_hi;
						if (!clipPolyToLine(ph.verts, along_axis, u_axis, u0, h_lo, h_hi))
							continue;
						// Range along `along_axis` where pv contains the line at z=z0.
						float v_lo, v_hi;
						if (!clipPolyToLine(pv.verts, along_axis, 2, z0, v_lo, v_hi))
							continue;

						float lo = std::max(h_lo, v_lo);
						float hi = std::min(h_hi, v_hi);
						if (hi - lo < 1e-3f)
							continue;

						vec3 p1, p2;
						p1.z = z0; p2.z = z0;
						if (u_axis == 0)
						{
							p1.x = u0; p2.x = u0;
							p1.y = lo; p2.y = hi;
						}
						else
						{
							p1.y = u0; p2.y = u0;
							p1.x = lo; p2.x = hi;
						}

						// Dedup: same submodel pair + identical quantized endpoints.
						long long k1 = (long long)std::llround(lo * 64.0f);
						long long k2 = (long long)std::llround(hi * 64.0f);
						long long zk = (long long)std::llround(z0 * 64.0f);
						long long uk = (long long)std::llround(u0 * 64.0f);
						long long key = ((long long)low * 9301 + high) ^ (k1 << 1) ^ (k2 << 17)
							^ (zk << 33) ^ (uk << 49) ^ ((long long)u_axis << 60);
						if (seamDedup.count(key))
							continue;
						seamDedup.insert(key);

						PixelwalkSeam s;
						s.p1 = p1;
						s.p2 = p2;
						s.hull = hull;
						s.low_idx = low;
						s.high_idx = high;
						outSeams.push_back(s);
					}
				}
			}
		}
	}

	print_log("pixelwalk: hull {} produced {} seam segments\n",
		hull, outSeams.size() - startCount);
}
