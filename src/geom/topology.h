// Topology changing operations: subdivision, UV-aware simplification, remeshing.
#pragma once
#include "rx/options.h"
#include "rx/scene.h"

namespace rx {

// Loop (smooth) or linear (midpoint) subdivision; UVs and other attributes are interpolated
// linearly per face, so UV seams and islands are preserved exactly.
void subdivideMesh(Mesh& m, int levels, Subdivision scheme);

// Quadric simplification that weighs UVs / normals; UV seams are kept (vertices may only slide
// along a seam). Returns achieved triangle count.
size_t simplifyMesh(Mesh& m, float keepRatio, bool lockSeams);

struct RemeshParams {
  float targetEdge = 0;   // <= 0: derived from targetFaces or current mean edge length
  size_t targetFaces = 0;
  int iterations = 5;
};
// Isotropic remeshing (split / collapse / flip / tangential relaxation + projection) that keeps
// UV seams and boundaries fixed and re-derives UVs inside each chart from the source surface.
void remeshMesh(Mesh& m, const RemeshParams& p, float smoothAngle);

}  // namespace rx
