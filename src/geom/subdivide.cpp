#include "geom/topology.h"
#include "geom/mesh_ops.h"
#include "core/par.h"
#include "core/util.h"
#include <algorithm>
#include <cmath>

namespace rx {

namespace {
struct HalfEdge { uint64_t key; uint32_t opp; };
struct SplitEdge { uint64_t key; uint32_t slot; };
inline uint64_t ekey(uint32_t a, uint32_t b) { return a < b ? (uint64_t(a) << 32) | b : (uint64_t(b) << 32) | a; }

void subdivideOnce(Mesh& m, Subdivision scheme) {
  const size_t V = m.positions.size(), T = m.triangleCount();
  const bool loop = scheme == Subdivision::Loop;
  std::vector<uint32_t> pid;
  std::vector<uint64_t> pkeys;  // sorted unique position edges
  std::vector<Vec3> edgePos;    // Loop edge points per position edge
  std::vector<Vec3> newPos;     // indexed by representative vertex

  if (loop) {
    pid = positionIds(m);
    std::vector<HalfEdge> he(T * 3);
    parallelFor(T, [&](size_t t) {
      const uint32_t* i = &m.indices[t * 3];
      for (int k = 0; k < 3; ++k)
        he[t * 3 + k] = {ekey(pid[i[k]], pid[i[(k + 1) % 3]]), pid[i[(k + 2) % 3]]};
    }, 4096);
    parallelSort(he.begin(), he.end(), [](const HalfEdge& a, const HalfEdge& b) { return a.key < b.key; });
    std::vector<Vec3> nsum(V), bsum(V);
    std::vector<uint32_t> val(V, 0), bcnt(V, 0);
    for (size_t i = 0; i < he.size();) {
      size_t j = i;
      while (j < he.size() && he[j].key == he[i].key) ++j;
      uint32_t a = uint32_t(he[i].key >> 32), b = uint32_t(he[i].key);
      Vec3 pa = m.positions[a], pb = m.positions[b];
      size_t cnt = j - i;
      Vec3 ep;
      if (cnt == 2) ep = (pa + pb) * 0.375f + (m.positions[he[i].opp] + m.positions[he[i + 1].opp]) * 0.125f;
      else {
        ep = (pa + pb) * 0.5f;
        bsum[a] += pb; bsum[b] += pa; bcnt[a]++; bcnt[b]++;
      }
      pkeys.push_back(he[i].key);
      edgePos.push_back(ep);
      nsum[a] += pb; nsum[b] += pa; val[a]++; val[b]++;
      i = j;
    }
    he.clear(); he.shrink_to_fit();
    newPos.resize(V);
    parallelFor(V, [&](size_t v) {
      if (pid[v] != v) return;
      Vec3 p = m.positions[v];
      if (bcnt[v] > 0) {
        newPos[v] = bcnt[v] == 2 ? p * 0.75f + bsum[v] * 0.125f : p;  // corners / non-manifold stay
      } else if (val[v] >= 3) {
        float n = float(val[v]);
        float beta = val[v] == 3 ? 3.0f / 16.0f : 3.0f / (8.0f * n);
        newPos[v] = p * (1.0f - n * beta) + nsum[v] * beta;
      } else newPos[v] = p;
    }, 8192);
  }

  // Edge vertices in attribute (split) topology.
  std::vector<SplitEdge> se(T * 3);
  parallelFor(T, [&](size_t t) {
    const uint32_t* i = &m.indices[t * 3];
    for (int k = 0; k < 3; ++k) se[t * 3 + k] = {ekey(i[k], i[(k + 1) % 3]), uint32_t(t * 3 + k)};
  }, 4096);
  parallelSort(se.begin(), se.end(), [](const SplitEdge& a, const SplitEdge& b) { return a.key < b.key; });
  std::vector<uint32_t> edgeVert(T * 3);
  std::vector<uint64_t> newEdges;  // split key per new vertex
  for (size_t i = 0; i < se.size();) {
    size_t j = i;
    uint32_t id = uint32_t(V + newEdges.size());
    while (j < se.size() && se[j].key == se[i].key) edgeVert[se[j++].slot] = id;
    newEdges.push_back(se[i].key);
    i = j;
  }
  se.clear(); se.shrink_to_fit();
  const size_t NV = V + newEdges.size();
  if (NV > 0xFFFFFFF0ull) { logWarn("subdivision exceeds 32-bit index range; stopping"); return; }

  auto grow = [&](auto& v, auto mix) {
    if (v.size() != V) { v.clear(); return; }
    v.resize(NV);
    parallelFor(newEdges.size(), [&](size_t e) {
      uint32_t a = uint32_t(newEdges[e] >> 32), b = uint32_t(newEdges[e]);
      v[V + e] = mix(v[a], v[b]);
    }, 8192);
  };
  grow(m.uv0, [](Vec2 a, Vec2 b) { return (a + b) * 0.5f; });
  grow(m.uv1, [](Vec2 a, Vec2 b) { return (a + b) * 0.5f; });
  grow(m.normals, [](Vec3 a, Vec3 b) { return normalize(a + b); });
  grow(m.tangents, [](Vec4 a, Vec4 b) {
    Vec3 t = normalize(Vec3{a.x + b.x, a.y + b.y, a.z + b.z});
    return Vec4{t.x, t.y, t.z, a.w};
  });
  grow(m.colors, [](Color8 a, Color8 b) {
    return Color8{uint8_t((a.r + b.r + 1) / 2), uint8_t((a.g + b.g + 1) / 2), uint8_t((a.b + b.b + 1) / 2), uint8_t((a.a + b.a + 1) / 2)};
  });
  std::vector<Vec3> pos(NV);
  parallelFor(newEdges.size(), [&](size_t e) {
    uint32_t a = uint32_t(newEdges[e] >> 32), b = uint32_t(newEdges[e]);
    if (loop) {
      uint64_t pk = ekey(pid[a], pid[b]);
      size_t idx = size_t(std::lower_bound(pkeys.begin(), pkeys.end(), pk) - pkeys.begin());
      pos[V + e] = edgePos[idx];
    } else pos[V + e] = (m.positions[a] + m.positions[b]) * 0.5f;
  }, 8192);
  parallelFor(V, [&](size_t v) { pos[v] = loop ? newPos[pid[v]] : m.positions[v]; }, 8192);
  m.positions.swap(pos);

  std::vector<uint32_t> idx(T * 12);
  parallelFor(T, [&](size_t t) {
    uint32_t a = m.indices[t * 3], b = m.indices[t * 3 + 1], c = m.indices[t * 3 + 2];
    uint32_t ab = edgeVert[t * 3], bc = edgeVert[t * 3 + 1], ca = edgeVert[t * 3 + 2];
    uint32_t* o = &idx[t * 12];
    o[0] = a; o[1] = ab; o[2] = ca;
    o[3] = ab; o[4] = b; o[5] = bc;
    o[6] = ca; o[7] = bc; o[8] = c;
    o[9] = ab; o[10] = bc; o[11] = ca;
  }, 4096);
  m.indices.swap(idx);
}
}  // namespace

void subdivideMesh(Mesh& m, int levels, Subdivision scheme) {
  for (int l = 0; l < levels && !m.indices.empty(); ++l) subdivideOnce(m, scheme);
}

}  // namespace rx
