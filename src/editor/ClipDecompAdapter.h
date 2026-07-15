// ClipDecompAdapter.h - editor-side bridge that decompiles a clip hull with the
// pixelwalk decompiler and returns NEUTRAL windings (see ClipDecompShim.h). This
// header pulls in NO pw headers, so BspRenderer.cpp can use it without dragging
// the pw:: type/macro world into a fork TU.
#pragma once
#include <vector>
#include "ClipDecompShim.h"

class Bsp;

// Decompile the clip hull for (modelIdx, hullIdx) into neutral DecompBrush windings.
// Returns empty if the model/hull is unusable, or if the map's clipnode indices
// don't fit pw::dclipnode_t's int16 children (32-bit-extended maps) -> caller
// should fall back to the Clipper path.
std::vector<ClipDecompOutBrush> DecompileEditorClipHull(Bsp* map, int modelIdx, int hullIdx);
