#include "geom/bvh.h"
#include "core/par.h"
#include <algorithm>
#include <numeric>

namespace rx {

Vec3 closestOnTriangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c, float& u, float& v, float& w) {
  Vec3 ab = b - a, ac = c - a, ap = p - a;
  float d1 = dot(ab, ap), d2 = dot(ac, ap);
  if (d1 <= 0 && d2 <= 0) { u = 1; v = 0; w = 0; return a; }
  Vec3 bp = p - b;
  float d3 = dot(ab, bp), d4 = dot(ac, bp);
  if (d3 >= 0 && d4 <= d3) { u = 0; v = 1; w = 0; return b; }
  float vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) { float t = d1 / (d1 - d3); u = 1 - t; v = t; w = 0; return a + ab * t; }
  Vec3 cp = p - c;
  float d5 = dot(ab, cp), d6 = dot(ac, cp);
  if (d6 >= 0 && d5 <= d6) { u = 0; v = 0; w = 1; return c; }
  float vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) { float t = d2 / (d2 - d6); u = 1 - t; v = 0; w = t; return a + ac * t; }
  float va = d3 * d6 - d5 * d4;
  if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
    float t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    u = 0; v = 1 - t; w = t;
    return b + (c - b) * t;
  }
  float den = 1.0f / (va + vb + vc);
  v = vb * den; w = vc * den; u = 1 - v - w;
  return a + ab * v + ac * w;
}

void Bvh::build(const std::vector<Vec3>& positions, const std::vector<uint32_t>& indices) {
  pos_ = &positions; idx_ = &indices;
  const size_t tc = indices.size() / 3;
  nodes_.clear();
  tris_.resize(tc);
  std::iota(tris_.begin(), tris_.end(), 0u);
  if (!tc) return;
  std::vector<Vec3> cent(tc);
  parallelFor(tc, [&](size_t t) {
    cent[t] = (positions[indices[t * 3]] + positions[indices[t * 3 + 1]] + positions[indices[t * 3 + 2]]) * (1.0f / 3);
  }, 8192);
  nodes_.reserve(tc / 2 + 16);
  buildRec(0, uint32_t(tc), cent);
}

uint32_t Bvh::buildRec(uint32_t b, uint32_t e, std::vector<Vec3>& cent) {
  uint32_t id = uint32_t(nodes_.size());
  nodes_.push_back({});
  Aabb box, cb;
  for (uint32_t i = b; i < e; ++i) {
    uint32_t t = tris_[i];
    for (int k = 0; k < 3; ++k) box.add((*pos_)[(*idx_)[t * 3 + k]]);
    cb.add(cent[t]);
  }
  nodes_[id].lo = box.lo; nodes_[id].hi = box.hi;
  if (e - b <= 4) { nodes_[id].start = b; nodes_[id].count = e - b; return id; }
  Vec3 s = cb.size();
  int axis = (s.x > s.y && s.x > s.z) ? 0 : (s.y > s.z ? 1 : 2);
  uint32_t mid = (b + e) / 2;
  auto key = [&](uint32_t t) { const Vec3& c = cent[t]; return axis == 0 ? c.x : axis == 1 ? c.y : c.z; };
  std::nth_element(tris_.begin() + b, tris_.begin() + mid, tris_.begin() + e,
                   [&](uint32_t x, uint32_t y) { return key(x) < key(y); });
  nodes_[id].count = 0;
  buildRec(b, mid, cent);
  uint32_t r = buildRec(mid, e, cent);
  nodes_[id].right = r;
  return id;
}

static inline float boxDist2(Vec3 q, Vec3 lo, Vec3 hi) {
  float dx = std::max({lo.x - q.x, 0.0f, q.x - hi.x});
  float dy = std::max({lo.y - q.y, 0.0f, q.y - hi.y});
  float dz = std::max({lo.z - q.z, 0.0f, q.z - hi.z});
  return dx * dx + dy * dy + dz * dz;
}

bool Bvh::closest(Vec3 q, ClosestHit& hit, float maxDist2) const {
  if (nodes_.empty()) return false;
  hit.dist2 = maxDist2;
  hit.tri = ~0u;
  uint32_t stack[128];
  int sp = 0;
  stack[sp++] = 0;
  const auto& P = *pos_;
  const auto& I = *idx_;
  while (sp) {
    const Node& n = nodes_[stack[--sp]];
    if (boxDist2(q, n.lo, n.hi) >= hit.dist2) continue;
    if (n.count) {
      for (uint32_t i = n.start; i < n.start + n.count; ++i) {
        uint32_t t = tris_[i];
        float u, v, w;
        Vec3 c = closestOnTriangle(q, P[I[t * 3]], P[I[t * 3 + 1]], P[I[t * 3 + 2]], u, v, w);
        Vec3 d = c - q;
        float d2 = dot(d, d);
        if (d2 < hit.dist2) { hit = {t, u, v, w, c, d2}; }
      }
      continue;
    }
    uint32_t l = uint32_t(&n - nodes_.data()) + 1, r = n.right;
    float dl = boxDist2(q, nodes_[l].lo, nodes_[l].hi), dr = boxDist2(q, nodes_[r].lo, nodes_[r].hi);
    if (sp + 2 > 128) continue;
    if (dl < dr) { stack[sp++] = r; stack[sp++] = l; }
    else { stack[sp++] = l; stack[sp++] = r; }
  }
  return hit.tri != ~0u;
}

}  // namespace rx
