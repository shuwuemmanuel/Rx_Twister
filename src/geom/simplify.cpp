#include "geom/topology.h"
#include "geom/mesh_ops.h"
#include "core/util.h"
#include <meshoptimizer.h>
#include <algorithm>
#include <cmath>

namespace rx {

size_t simplifyMesh(Mesh& m, float keepRatio, bool lockSeams) {
  const size_t tc = m.triangleCount(), vc = m.positions.size();
  if (keepRatio >= 1.0f || tc < 4) return tc;
  size_t target = std::max<size_t>(1, size_t(std::llround(double(tc) * keepRatio))) * 3;

  // attribute stream: uv0 (2) + normal (3)
  const bool uv = m.hasUV(), nrm = m.hasNormals();
  const size_t ac = (uv ? 2 : 0) + (nrm ? 3 : 0);
  std::vector<float> attr(ac * vc);
  std::vector<float> weights;
  if (uv) { weights.push_back(1.0f); weights.push_back(1.0f); }
  if (nrm) { weights.push_back(0.3f); weights.push_back(0.3f); weights.push_back(0.3f); }
  for (size_t v = 0; v < vc && ac; ++v) {
    float* a = &attr[v * ac];
    size_t k = 0;
    if (uv) { a[k++] = m.uv0[v].x; a[k++] = m.uv0[v].y; }
    if (nrm) { a[k++] = m.normals[v].x; a[k++] = m.normals[v].y; a[k++] = m.normals[v].z; }
  }
  std::vector<uint32_t> out(m.indices.size());
  float err = 0;
  size_t n;
#if MESHOPTIMIZER_VERSION >= 210
  std::vector<unsigned char> lock;
  if (lockSeams && uv) {
    // Lock vertices whose position is shared by vertices with different UVs (UV seams).
    auto pid = positionIds(m);
    lock.assign(vc, 0);
    for (size_t v = 0; v < vc; ++v) {
      uint32_t r = pid[v];
      if (r != v && (m.uv0[r].x != m.uv0[v].x || m.uv0[r].y != m.uv0[v].y)) { lock[v] = 1; lock[r] = 1; }
    }
    for (size_t v = 0; v < vc; ++v) if (lock[pid[v]]) lock[v] = 1;
  }
  n = meshopt_simplifyWithAttributes(out.data(), m.indices.data(), m.indices.size(), &m.positions[0].x, vc, sizeof(Vec3),
                                     ac ? attr.data() : nullptr, ac * sizeof(float), ac ? weights.data() : nullptr, ac,
                                     lock.empty() ? nullptr : lock.data(), target, 1.0f, 0, &err);
#else
  (void)lockSeams;  // meshoptimizer < 0.21 keeps attribute seams topologically by itself
  if (ac)
    n = meshopt_simplifyWithAttributes(out.data(), m.indices.data(), m.indices.size(), &m.positions[0].x, vc,
                                       sizeof(Vec3), attr.data(), ac * sizeof(float), weights.data(), ac, target, 1.0f,
                                       0, &err);
  else
    n = meshopt_simplify(out.data(), m.indices.data(), m.indices.size(), &m.positions[0].x, vc, sizeof(Vec3), target,
                         1.0f, 0, &err);
#endif
  if (n > target * 3 / 2)
    logVerbose("simplify '%s': reached %zu of %zu target triangles (topology / seams limit further reduction)",
               m.name.c_str(), n / 3, target / 3);
  out.resize(n);
  m.indices.swap(out);
  compactVertices(m);
  return m.triangleCount();
}

}  // namespace rx
