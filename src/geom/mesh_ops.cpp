#include "geom/mesh_ops.h"
#include "core/par.h"
#include "core/util.h"
#include <meshoptimizer.h>
#include <cmath>
#include <cstring>
#include <numeric>

namespace rx {

template <class T>
static void remapStream(std::vector<T>& v, const std::vector<uint32_t>& remap, size_t newCount) {
  if (v.empty()) return;
  std::vector<T> out(newCount);
  meshopt_remapVertexBuffer(out.data(), v.data(), v.size(), sizeof(T), remap.data());
  v.swap(out);
}

static void applyRemap(Mesh& m, const std::vector<uint32_t>& remap, size_t uniq) {
  const size_t vc = m.positions.size();
  auto fix = [&](auto& v) { if (v.size() != vc) v.clear(); };
  fix(m.normals); fix(m.tangents); fix(m.uv0); fix(m.uv1); fix(m.colors);
  remapStream(m.positions, remap, uniq);
  remapStream(m.normals, remap, uniq);
  remapStream(m.tangents, remap, uniq);
  remapStream(m.uv0, remap, uniq);
  remapStream(m.uv1, remap, uniq);
  remapStream(m.colors, remap, uniq);
  meshopt_remapIndexBuffer(m.indices.data(), m.indices.data(), m.indices.size(), remap.data());
}

// Parallel exact weld for big meshes: hash every vertex, parallel sort by hash, resolve groups.
static void weldMeshParallel(Mesh& m) {
  const size_t vc = m.positions.size();
  auto fix = [&](auto& v) { if (v.size() != vc) v.clear(); };
  fix(m.normals); fix(m.tangents); fix(m.uv0); fix(m.uv1); fix(m.colors);
  auto same = [&](uint32_t a, uint32_t b) {
    if (memcmp(&m.positions[a], &m.positions[b], sizeof(Vec3))) return false;
    if (!m.normals.empty() && memcmp(&m.normals[a], &m.normals[b], sizeof(Vec3))) return false;
    if (!m.tangents.empty() && memcmp(&m.tangents[a], &m.tangents[b], sizeof(Vec4))) return false;
    if (!m.uv0.empty() && memcmp(&m.uv0[a], &m.uv0[b], sizeof(Vec2))) return false;
    if (!m.uv1.empty() && memcmp(&m.uv1[a], &m.uv1[b], sizeof(Vec2))) return false;
    if (!m.colors.empty() && memcmp(&m.colors[a], &m.colors[b], 4)) return false;
    return true;
  };
  struct HV { uint64_t h; uint32_t v; };
  std::vector<HV> hv(vc);
  parallelFor(vc, [&](size_t v) {
    uint64_t h = 1469598103934665603ull;
    auto mix = [&](const void* p, size_t n) {
      const uint8_t* b = static_cast<const uint8_t*>(p);
      for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 1099511628211ull; }
    };
    mix(&m.positions[v], 12);
    if (!m.normals.empty()) mix(&m.normals[v], 12);
    if (!m.tangents.empty()) mix(&m.tangents[v], 16);
    if (!m.uv0.empty()) mix(&m.uv0[v], 8);
    if (!m.uv1.empty()) mix(&m.uv1[v], 8);
    if (!m.colors.empty()) mix(&m.colors[v], 4);
    hv[v] = {h, uint32_t(v)};
  }, 1 << 14);
  parallelSort(hv.begin(), hv.end(), [](const HV& a, const HV& b) { return a.h < b.h || (a.h == b.h && a.v < b.v); });
  std::vector<uint32_t> rep(vc);
  // groups of equal hash are tiny; resolve them in parallel over sorted ranges split at hash boundaries
  parallelRanges(vc, 1 << 16, [&](size_t b, size_t e) {
    while (b > 0 && b < vc && hv[b].h == hv[b - 1].h) ++b;   // start at a group boundary
    while (e < vc && hv[e].h == hv[e - 1].h) ++e;
    for (size_t i = b; i < e;) {
      size_t j = i;
      while (j < e && hv[j].h == hv[i].h) ++j;
      for (size_t k = i; k < j; ++k) {
        uint32_t v = hv[k].v, r = v;
        for (size_t q = i; q < k; ++q) if (rep[hv[q].v] == hv[q].v && same(hv[q].v, v)) { r = hv[q].v; break; }
        rep[v] = r;
      }
      i = j;
    }
  });
  hv.clear(); hv.shrink_to_fit();
  std::vector<uint32_t> newId(vc);
  size_t n = 0;
  for (size_t v = 0; v < vc; ++v) newId[v] = rep[v] == v ? uint32_t(n++) : 0;
  if (n == vc) return;  // nothing to weld
  std::vector<uint32_t> remap(vc);
  parallelFor(vc, [&](size_t v) { remap[v] = newId[rep[v]]; }, 1 << 15);
  auto apply = [&](auto& s) {
    if (s.empty()) return;
    std::remove_reference_t<decltype(s)> out(n);
    parallelFor(vc, [&](size_t v) { if (rep[v] == v) out[remap[v]] = s[v]; }, 1 << 15);
    s.swap(out);
  };
  apply(m.positions); apply(m.normals); apply(m.tangents); apply(m.uv0); apply(m.uv1); apply(m.colors);
  parallelFor(m.indices.size(), [&](size_t i) { m.indices[i] = remap[m.indices[i]]; }, 1 << 15);
}

void weldMesh(Mesh& m) {
  if (m.positions.empty() || m.indices.empty()) return;
  const size_t vc = m.positions.size();
  if (vc > (size_t(1) << 20) && threadLimit() > 1) { weldMeshParallel(m); return; }
  std::vector<meshopt_Stream> streams;
  streams.push_back({m.positions.data(), sizeof(Vec3), sizeof(Vec3)});
  if (m.normals.size() == vc) streams.push_back({m.normals.data(), sizeof(Vec3), sizeof(Vec3)});
  if (m.tangents.size() == vc) streams.push_back({m.tangents.data(), sizeof(Vec4), sizeof(Vec4)});
  if (m.uv0.size() == vc) streams.push_back({m.uv0.data(), sizeof(Vec2), sizeof(Vec2)});
  if (m.uv1.size() == vc) streams.push_back({m.uv1.data(), sizeof(Vec2), sizeof(Vec2)});
  if (m.colors.size() == vc) streams.push_back({m.colors.data(), 4, 4});
  std::vector<uint32_t> remap(vc);
  size_t uniq = meshopt_generateVertexRemapMulti(remap.data(), m.indices.data(), m.indices.size(), vc,
                                                 streams.data(), streams.size());
  applyRemap(m, remap, uniq);
}

void compactVertices(Mesh& m) {
  if (m.positions.empty()) return;
  std::vector<uint32_t> remap(m.positions.size());
  size_t uniq = meshopt_optimizeVertexFetchRemap(remap.data(), m.indices.data(), m.indices.size(), m.positions.size());
  applyRemap(m, remap, uniq);
}

std::vector<uint32_t> positionIds(const Mesh& m) {
  std::vector<uint32_t> remap(m.positions.size());
  if (m.positions.empty()) return remap;
  meshopt_generateVertexRemap(remap.data(), nullptr, m.positions.size(), m.positions.data(), m.positions.size(), sizeof(Vec3));
  // remap gives compact ids; convert to "first vertex with that position" representatives.
  std::vector<uint32_t> first(m.positions.size(), ~0u);
  for (size_t v = 0; v < remap.size(); ++v) {
    uint32_t& f = first[remap[v]];
    if (f == ~0u) f = uint32_t(v);
    remap[v] = f;
  }
  return remap;
}

void unweld(Mesh& m) {
  const size_t n = m.indices.size(), vc = m.positions.size();
  auto ex = [&](auto& v) {
    if (v.size() != vc) { v.clear(); return; }
    std::remove_reference_t<decltype(v)> out(n);
    for (size_t i = 0; i < n; ++i) out[i] = v[m.indices[i]];
    v.swap(out);
  };
  ex(m.positions); ex(m.normals); ex(m.tangents); ex(m.uv0); ex(m.uv1); ex(m.colors);
  std::iota(m.indices.begin(), m.indices.end(), 0u);
}

void computeNormals(Mesh& m, float angleDeg) {
  const size_t vc = m.positions.size(), tc = m.triangleCount();
  if (!vc || !tc) return;
  std::vector<Vec3> fn(tc);
  parallelFor(tc, [&](size_t t) {
    const uint32_t* i = &m.indices[t * 3];
    fn[t] = cross(m.positions[i[1]] - m.positions[i[0]], m.positions[i[2]] - m.positions[i[0]]);  // area weighted
  }, 4096);
  auto pid = positionIds(m);
  if (angleDeg >= 179.0f) {
    std::vector<Vec3> acc(vc);
    for (size_t t = 0; t < tc; ++t)
      for (int k = 0; k < 3; ++k) acc[pid[m.indices[t * 3 + k]]] += fn[t];
    m.normals.resize(vc);
    parallelFor(vc, [&](size_t v) { m.normals[v] = normalize(acc[pid[v]]); }, 8192);
    return;
  }
  // Angle-limited smoothing: CSR position -> faces, normals per corner, then re-weld.
  std::vector<uint32_t> start(vc + 1, 0);
  for (uint32_t i : m.indices) start[pid[i] + 1]++;
  for (size_t v = 0; v < vc; ++v) start[v + 1] += start[v];
  std::vector<uint32_t> faces(m.indices.size()), fill(start.begin(), start.end() - 1);
  for (size_t c = 0; c < m.indices.size(); ++c) faces[fill[pid[m.indices[c]]]++] = uint32_t(c / 3);
  const float cosT = std::cos(angleDeg * kPi / 180.0f);
  std::vector<Vec3> nfn(tc);
  parallelFor(tc, [&](size_t t) { nfn[t] = normalize(fn[t]); }, 8192);
  std::vector<Vec3> cn(m.indices.size());
  parallelFor(m.indices.size(), [&](size_t c) {
    uint32_t p = pid[m.indices[c]];
    Vec3 me = nfn[c / 3], acc{};
    for (uint32_t k = start[p]; k < start[p + 1]; ++k) {
      uint32_t f = faces[k];
      if (dot(nfn[f], me) >= cosT) acc += fn[f];
    }
    cn[c] = normalize(acc);
  }, 4096);
  unweld(m);
  m.normals.swap(cn);
  weldMesh(m);
}

void computeTangents(Mesh& m) {
  const size_t vc = m.positions.size();
  if (!m.hasUV() || !vc) return;
  if (!m.hasNormals()) computeNormals(m);
  std::vector<Vec3> tan(vc), bit(vc);
  for (size_t t = 0; t < m.triangleCount(); ++t) {
    uint32_t a = m.indices[t * 3], b = m.indices[t * 3 + 1], c = m.indices[t * 3 + 2];
    Vec3 e1 = m.positions[b] - m.positions[a], e2 = m.positions[c] - m.positions[a];
    // bitangent follows +v in OpenGL orientation (v_gl = 1 - v_gltf)
    float du1 = m.uv0[b].x - m.uv0[a].x, dv1 = -(m.uv0[b].y - m.uv0[a].y);
    float du2 = m.uv0[c].x - m.uv0[a].x, dv2 = -(m.uv0[c].y - m.uv0[a].y);
    float det = du1 * dv2 - du2 * dv1;
    if (std::fabs(det) < 1e-20f) continue;
    float r = 1.0f / det;
    Vec3 T = (e1 * dv2 - e2 * dv1) * r, B = (e2 * du1 - e1 * du2) * r;
    // weight by area so tiny triangles do not dominate
    float w = length(cross(e1, e2));
    T = normalize(T) * w; B = normalize(B) * w;
    tan[a] += T; tan[b] += T; tan[c] += T;
    bit[a] += B; bit[b] += B; bit[c] += B;
  }
  m.tangents.resize(vc);
  parallelFor(vc, [&](size_t v) {
    Vec3 n = m.normals[v];
    Vec3 t = tan[v] - n * dot(n, tan[v]);
    if (dot(t, t) < 1e-20f) {  // pick any perpendicular
      t = std::fabs(n.x) < 0.9f ? cross(n, Vec3{1, 0, 0}) : cross(n, Vec3{0, 1, 0});
    }
    t = normalize(t);
    float w = dot(cross(n, t), bit[v]) < 0 ? -1.0f : 1.0f;
    m.tangents[v] = {t.x, t.y, t.z, w};
  }, 8192);
}

void removeDegenerateTriangles(Mesh& m) {
  size_t o = 0;
  for (size_t t = 0; t < m.triangleCount(); ++t) {
    uint32_t a = m.indices[t * 3], b = m.indices[t * 3 + 1], c = m.indices[t * 3 + 2];
    if (a == b || b == c || a == c) continue;
    m.indices[o++] = a; m.indices[o++] = b; m.indices[o++] = c;
  }
  m.indices.resize(o);
}

void optimizeMesh(Mesh& m) {
  if (m.indices.empty()) return;
  removeDegenerateTriangles(m);
  const size_t ic = m.indices.size();
  const size_t block = size_t(3) << 20;  // triangles are re-ordered within ~1M triangle blocks in parallel
  if (ic > block * 2 && threadLimit() > 1) {
    parallelFor((ic + block - 1) / block, [&](size_t b) {
      size_t s = b * block, n = std::min(block, ic - s);
      meshopt_optimizeVertexCache(&m.indices[s], &m.indices[s], n, m.positions.size());
    }, 1);
  } else {
    meshopt_optimizeVertexCache(m.indices.data(), m.indices.data(), ic, m.positions.size());
  }
  compactVertices(m);
}

void transformMesh(Mesh& m, const Mat4& t) {
  if (t.isIdentity()) return;
  // normal matrix = inverse transpose of the upper 3x3
  const float* a = t.m;
  float c00 = a[5] * a[10] - a[6] * a[9], c01 = a[6] * a[8] - a[4] * a[10], c02 = a[4] * a[9] - a[5] * a[8];
  float c10 = a[2] * a[9] - a[1] * a[10], c11 = a[0] * a[10] - a[2] * a[8], c12 = a[1] * a[8] - a[0] * a[9];
  float c20 = a[1] * a[6] - a[2] * a[5], c21 = a[2] * a[4] - a[0] * a[6], c22 = a[0] * a[5] - a[1] * a[4];
  // cXY above is cofactor C(Y,X); inverse transpose = C / det
  const float sg = t.det3() < 0 ? -1.0f : 1.0f;
  auto nrm = [&](Vec3 n) {
    return normalize(Vec3{c00 * n.x + c10 * n.y + c20 * n.z, c01 * n.x + c11 * n.y + c21 * n.z,
                          c02 * n.x + c12 * n.y + c22 * n.z} * sg);
  };
  parallelFor(m.positions.size(), [&](size_t i) {
    m.positions[i] = t.point(m.positions[i]);
    if (i < m.normals.size()) m.normals[i] = nrm(m.normals[i]);
    if (i < m.tangents.size()) {
      Vec3 tv = normalize(t.vector({m.tangents[i].x, m.tangents[i].y, m.tangents[i].z}));
      m.tangents[i] = {tv.x, tv.y, tv.z, m.tangents[i].w};
    }
  }, 8192);
  if (t.det3() < 0) {
    for (size_t i = 0; i + 2 < m.indices.size(); i += 3) std::swap(m.indices[i + 1], m.indices[i + 2]);
    for (auto& tg : m.tangents) tg.w = -tg.w;
  }
}

static void walk(const Scene& s, int n, const Mat4& parent, std::vector<Mat4>& w, std::vector<char>& seen) {
  if (n < 0 || size_t(n) >= s.nodes.size() || seen[n]) return;
  seen[n] = 1;
  w[n] = parent * s.nodes[n].local;
  for (int c : s.nodes[n].children) walk(s, c, w[n], w, seen);
}
std::vector<Mat4> worldMatrices(const Scene& s) {
  std::vector<Mat4> w(s.nodes.size());
  std::vector<char> seen(s.nodes.size(), 0);
  for (int r : s.roots) walk(s, r, Mat4::identity(), w, seen);
  return w;
}

void flattenScene(Scene& s) {
  auto world = worldMatrices(s);
  std::vector<char> reachable(s.nodes.size(), 0);
  {
    std::vector<int> st(s.roots.begin(), s.roots.end());
    while (!st.empty()) {
      int n = st.back(); st.pop_back();
      if (n < 0 || size_t(n) >= s.nodes.size() || reachable[n]) continue;
      reachable[n] = 1;
      for (int c : s.nodes[n].children) st.push_back(c);
    }
  }
  struct Inst { int node, mesh; };
  std::vector<Inst> inst;
  std::vector<int> uses(s.meshes.size(), 0);
  for (size_t n = 0; n < s.nodes.size(); ++n)
    if (reachable[n])
      for (int m : s.nodes[n].meshes)
        if (m >= 0 && size_t(m) < s.meshes.size()) { inst.push_back({int(n), m}); uses[m]++; }
  std::vector<Mesh> out;
  out.reserve(inst.size());
  std::vector<int> seenMesh(s.meshes.size(), 0);
  std::vector<Node> nodes;
  for (auto& in : inst) {
    Mesh m;
    if (++seenMesh[in.mesh] == uses[in.mesh]) m = std::move(s.meshes[in.mesh]);  // last use: move
    else m = s.meshes[in.mesh];
    if (m.name.empty()) m.name = s.nodes[in.node].name;
    transformMesh(m, world[in.node]);
    Node nd;
    nd.name = s.nodes[in.node].name.empty() ? m.name : s.nodes[in.node].name;
    nd.meshes.push_back(int(out.size()));
    out.push_back(std::move(m));
    nodes.push_back(std::move(nd));
  }
  s.meshes.swap(out);
  s.nodes.swap(nodes);
  s.roots.resize(s.nodes.size());
  std::iota(s.roots.begin(), s.roots.end(), 0);
}

void convertAxisAndUnits(Scene& s, UpAxis to, double metersPerUnit) {
  Mat4 r;
  if (s.up != to) {
    if (s.up == UpAxis::Y && to == UpAxis::Z) {  // (x,y,z) -> (x,-z,y)
      r.m[0] = 1; r.m[5] = 0; r.m[6] = 1; r.m[9] = -1; r.m[10] = 0;
    } else {                                      // Z up -> Y up: (x,y,z) -> (x,z,-y)
      r.m[0] = 1; r.m[5] = 0; r.m[6] = -1; r.m[9] = 1; r.m[10] = 0;
    }
  }
  float k = float(s.metersPerUnit / metersPerUnit);
  Mat4 t = Mat4::scale({k, k, k}) * r;
  s.up = to;
  s.metersPerUnit = metersPerUnit;
  if (t.isIdentity()) return;
  Node root;
  root.name = "RxTwister_Root";
  root.local = t;
  root.children = s.roots;
  s.nodes.push_back(root);
  s.roots = {int(s.nodes.size()) - 1};
}

void dropUnusedResources(Scene& s) {
  // materials
  std::vector<int> mUsed(s.materials.size(), 0);
  for (auto& m : s.meshes) if (m.material >= 0 && size_t(m.material) < s.materials.size()) mUsed[m.material] = 1;
  std::vector<int> mMap(s.materials.size(), -1);
  std::vector<Material> mats;
  for (size_t i = 0; i < s.materials.size(); ++i)
    if (mUsed[i]) { mMap[i] = int(mats.size()); mats.push_back(std::move(s.materials[i])); }
  for (auto& m : s.meshes) m.material = (m.material >= 0 && size_t(m.material) < mMap.size()) ? mMap[m.material] : -1;
  s.materials.swap(mats);
  // images
  std::vector<int> iUsed(s.images.size(), 0);
  auto mark = [&](const TextureSlot& t) { if (t.image >= 0 && size_t(t.image) < s.images.size()) iUsed[t.image] = 1; };
  for (auto& m : s.materials) {
    mark(m.baseColorTex); mark(m.normalTex); mark(m.occlusionTex); mark(m.emissiveTex);
    mark(m.metalRoughTex); mark(m.roughnessTex); mark(m.metallicTex);
  }
  std::vector<int> iMap(s.images.size(), -1);
  std::vector<Image> imgs;
  for (size_t i = 0; i < s.images.size(); ++i)
    if (iUsed[i]) { iMap[i] = int(imgs.size()); imgs.push_back(std::move(s.images[i])); }
  auto fix = [&](TextureSlot& t) { t.image = (t.image >= 0 && size_t(t.image) < iMap.size()) ? iMap[t.image] : -1; };
  for (auto& m : s.materials) {
    fix(m.baseColorTex); fix(m.normalTex); fix(m.occlusionTex); fix(m.emissiveTex);
    fix(m.metalRoughTex); fix(m.roughnessTex); fix(m.metallicTex);
  }
  s.images.swap(imgs);
}

}  // namespace rx
