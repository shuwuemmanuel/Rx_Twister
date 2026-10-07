// Triangle BVH with closest point queries.
#pragma once
#include <vector>
#include "rx/math.h"

namespace rx {

struct ClosestHit {
  uint32_t tri = ~0u;
  float b0 = 1, b1 = 0, b2 = 0;   // barycentrics of the triangle's 3 corners
  Vec3 p;
  float dist2 = 1e30f;
};

// Closest point on triangle abc to p, with barycentrics (Ericson, RTCD 5.1.5).
Vec3 closestOnTriangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c, float& b0, float& b1, float& b2);

class Bvh {
 public:
  // positions / indices must outlive the BVH.
  void build(const std::vector<Vec3>& positions, const std::vector<uint32_t>& indices);
  bool closest(Vec3 q, ClosestHit& hit, float maxDist2 = 1e30f) const;
  bool empty() const { return nodes_.empty(); }
 private:
  struct Node { Vec3 lo, hi; uint32_t start, count; uint32_t right; };  // count==0 => inner (left = this+1)
  uint32_t buildRec(uint32_t b, uint32_t e, std::vector<Vec3>& cent);
  std::vector<Node> nodes_;
  std::vector<uint32_t> tris_;
  const std::vector<Vec3>* pos_ = nullptr;
  const std::vector<uint32_t>* idx_ = nullptr;
};

}  // namespace rx
