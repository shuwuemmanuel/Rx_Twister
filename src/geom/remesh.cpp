// Isotropic remeshing that preserves UVs.
//
// The mesh is held as welded positions + per-corner UVs, so a UV seam is simply an edge whose two
// faces disagree on corner UVs. Seam and boundary vertices are locked: they may receive new
// vertices (edge splits) but never move or disappear, so every UV island keeps its exact outline.
// Interior vertices are relaxed tangentially, projected back to the source surface and receive a
// UV interpolated from their own (single-chart) one-ring, so texture mapping stays continuous.
#include "geom/topology.h"
#include "geom/bvh.h"
#include "geom/mesh_ops.h"
#include "core/par.h"
#include "core/util.h"
#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace rx {
namespace {

constexpr uint32_t kDead = ~0u;
inline uint64_t ekey(uint32_t a, uint32_t b) { return a < b ? (uint64_t(a) << 32) | b : (uint64_t(b) << 32) | a; }
inline bool sameUV(Vec2 a, Vec2 b) { return std::fabs(a.x - b.x) <= 1e-6f && std::fabs(a.y - b.y) <= 1e-6f; }
inline uint32_t nxt(uint32_t s) { return s - s % 3 + (s % 3 + 1) % 3; }
inline uint32_t prv(uint32_t s) { return s - s % 3 + (s % 3 + 2) % 3; }

struct EdgeRec { uint64_t key; uint32_t slot; };  // slot = corner where the half edge starts

struct Remesher {
  std::vector<Vec3> P;
  std::vector<uint32_t> F;   // 3 per face, F[3f] == kDead for removed faces
  std::vector<Vec2> C;       // per corner uv
  std::vector<uint8_t> lock, boundaryV;
  bool hasUV = false;
  float L = 1, Lhi = 1, Llo = 1;
  const Bvh* ref = nullptr;

  size_t faces() const { return F.size() / 3; }
  bool alive(size_t f) const { return F[f * 3] != kDead; }

  std::vector<EdgeRec> edges() const {
    std::vector<EdgeRec> e;
    e.reserve(F.size());
    for (size_t f = 0; f < faces(); ++f)
      if (alive(f))
        for (uint32_t k = 0; k < 3; ++k) {
          uint32_t s = uint32_t(f * 3 + k);
          e.push_back({ekey(F[s], F[nxt(s)]), s});
        }
    parallelSort(e.begin(), e.end(), [](const EdgeRec& a, const EdgeRec& b) { return a.key < b.key; });
    return e;
  }

  // Is the edge shared by s1/s2 a seam (uv mismatch or inconsistent orientation)?
  bool seam(uint32_t s1, uint32_t s2) const {
    if (F[s1] != F[nxt(s2)] || F[nxt(s1)] != F[s2]) return true;  // flipped neighbour
    if (!hasUV) return false;
    return !sameUV(C[s1], C[nxt(s2)]) || !sameUV(C[nxt(s1)], C[s2]);
  }

  void computeLocks() {
    lock.assign(P.size(), 0);
    boundaryV.assign(P.size(), 0);
    auto E = edges();
    for (size_t i = 0; i < E.size();) {
      size_t j = i;
      while (j < E.size() && E[j].key == E[i].key) ++j;
      uint32_t a = uint32_t(E[i].key >> 32), b = uint32_t(E[i].key);
      if (j - i != 2) { lock[a] = lock[b] = 1; boundaryV[a] = boundaryV[b] = 1; }
      else if (seam(E[i].slot, E[i + 1].slot)) lock[a] = lock[b] = 1;
      i = j;
    }
  }

  // CSR vertex -> corner slots
  void corners(std::vector<uint32_t>& start, std::vector<uint32_t>& list) const {
    start.assign(P.size() + 1, 0);
    for (size_t f = 0; f < faces(); ++f) if (alive(f)) for (int k = 0; k < 3; ++k) start[F[f * 3 + k] + 1]++;
    for (size_t v = 0; v < P.size(); ++v) start[v + 1] += start[v];
    list.resize(start.back());
    std::vector<uint32_t> fill(start.begin(), start.end() - 1);
    for (size_t f = 0; f < faces(); ++f) if (alive(f)) for (int k = 0; k < 3; ++k) list[fill[F[f * 3 + k]]++] = uint32_t(f * 3 + k);
  }

  float len(uint32_t a, uint32_t b) const { return length(P[a] - P[b]); }

  // ------------------------------------------------------------------ split
  size_t splitLong() {
    size_t total = 0;
    for (int round = 0; round < 12; ++round) {
      auto E = edges();
      struct Cand { float l; size_t i, n; };
      std::vector<Cand> cand;
      for (size_t i = 0; i < E.size();) {
        size_t j = i;
        while (j < E.size() && E[j].key == E[i].key) ++j;
        float l = len(uint32_t(E[i].key >> 32), uint32_t(E[i].key));
        if (l > Lhi && j - i <= 2) cand.push_back({l, i, j - i});
        i = j;
      }
      if (cand.empty()) break;
      std::sort(cand.begin(), cand.end(), [](const Cand& a, const Cand& b) { return a.l > b.l; });
      std::vector<uint8_t> dirty(faces(), 0);
      size_t did = 0;
      for (auto& c : cand) {
        uint32_t s[2] = {E[c.i].slot, c.n == 2 ? E[c.i + 1].slot : 0};
        bool skip = false;
        for (size_t k = 0; k < c.n; ++k) skip |= dirty[s[k] / 3] != 0;
        if (skip) continue;
        uint32_t a = uint32_t(E[c.i].key >> 32), b = uint32_t(E[c.i].key);
        bool lockNew = c.n != 2 || seam(s[0], s[1]);
        uint32_t m = uint32_t(P.size());
        P.push_back((P[a] + P[b]) * 0.5f);
        lock.push_back(lockNew ? 1 : 0);
        boundaryV.push_back(c.n != 2 ? 1 : 0);
        for (size_t k = 0; k < c.n; ++k) {
          uint32_t s0 = s[k], s1 = nxt(s0), s2 = prv(s0);
          uint32_t vb = F[s1], vc = F[s2];
          Vec2 um = hasUV ? (C[s0] + C[s1]) * 0.5f : Vec2{};
          Vec2 ub = hasUV ? C[s1] : Vec2{}, uc = hasUV ? C[s2] : Vec2{};
          // existing face becomes (a, m, c); new face (m, b, c)
          F[s1] = m;
          if (hasUV) C[s1] = um;
          F.push_back(m); F.push_back(vb); F.push_back(vc);
          if (hasUV) { C.push_back(um); C.push_back(ub); C.push_back(uc); }
          else C.resize(F.size());
          dirty[s0 / 3] = 1;
          dirty.push_back(1);
        }
        ++did;
      }
      total += did;
      if (!did) break;
    }
    return total;
  }

  // --------------------------------------------------------------- collapse
  size_t collapseShort() {
    size_t total = 0;
    std::vector<uint32_t> start, list, na, nb, nc;
    auto neighbours = [&](uint32_t v, std::vector<uint32_t>& out) {
      out.clear();
      for (uint32_t k = start[v]; k < start[v + 1]; ++k) { out.push_back(F[nxt(list[k])]); out.push_back(F[prv(list[k])]); }
      std::sort(out.begin(), out.end());
      out.erase(std::unique(out.begin(), out.end()), out.end());
    };
    for (int round = 0; round < 24; ++round) {
      corners(start, list);
      auto E = edges();
      struct Cand { float l; size_t i; };
      std::vector<Cand> cand;
      for (size_t i = 0; i < E.size();) {
        size_t j = i;
        while (j < E.size() && E[j].key == E[i].key) ++j;
        uint32_t a = uint32_t(E[i].key >> 32), b = uint32_t(E[i].key);
        float l = len(a, b);
        if (j - i == 2 && l < Llo && !(lock[a] && lock[b])) cand.push_back({l, i});
        i = j;
      }
      if (cand.empty()) break;
      std::sort(cand.begin(), cand.end(), [](const Cand& x, const Cand& y) { return x.l < y.l; });
      std::vector<uint8_t> touched(P.size(), 0);
      size_t did = 0;
      for (auto& c : cand) {
        uint32_t s1 = E[c.i].slot, s2 = E[c.i + 1].slot;
        if (!alive(s1 / 3) || !alive(s2 / 3)) continue;  // stale: removed earlier this round
        uint32_t a = F[s1], b = F[nxt(s1)];
        if (ekey(a, b) != E[c.i].key || ekey(F[s2], F[nxt(s2)]) != E[c.i].key) continue;
        if (touched[a] || touched[b]) continue;
        if (seam(s1, s2)) continue;  // seam edges are kept
        if (lock[a]) { std::swap(a, b); }
        if (lock[a]) continue;
        const bool moveB = !lock[b];
        const Vec3 tp = moveB ? (P[a] + P[b]) * 0.5f : P[b];
        // the two faces being removed
        const uint32_t f1 = s1 / 3, f2 = s2 / 3;
        const uint32_t o1 = F[prv(s1)], o2 = F[prv(s2)];
        if (o1 == o2) continue;
        neighbours(a, na); neighbours(b, nb);
        // link condition: common neighbours must be exactly {o1, o2}
        size_t common = 0;
        for (size_t i = 0, j = 0; i < na.size() && j < nb.size();) {
          if (na[i] < nb[j]) ++i; else if (nb[j] < na[i]) ++j; else { ++common; ++i; ++j; }
        }
        if (common != 2) continue;
        bool ok = true;
        for (uint32_t o : {o1, o2}) {
          neighbours(o, nc);
          if (nc.size() <= (boundaryV[o] ? 2u : 3u)) ok = false;
        }
        if (!ok) continue;
        for (uint32_t n : na) if (n != b && length(P[n] - tp) > Lhi) { ok = false; break; }
        if (ok && moveB) for (uint32_t n : nb) if (n != a && length(P[n] - tp) > Lhi) { ok = false; break; }
        if (!ok) continue;
        // normal flip test for surviving faces around a (and b when it moves)
        auto faceOk = [&](uint32_t v) {
          for (uint32_t k = start[v]; k < start[v + 1]; ++k) {
            uint32_t f = list[k] / 3;
            if (f == f1 || f == f2) continue;
            Vec3 q[3];
            for (int i = 0; i < 3; ++i) {
              uint32_t w = F[f * 3 + i];
              q[i] = (w == a || (w == b && moveB)) ? tp : P[w];
            }
            Vec3 n0 = cross(P[F[f * 3 + 1]] - P[F[f * 3]], P[F[f * 3 + 2]] - P[F[f * 3]]);
            Vec3 n1 = cross(q[1] - q[0], q[2] - q[0]);
            float l0 = length(n0), l1 = length(n1);
            if (l1 < 1e-12f * (Lhi * Lhi) || dot(n0, n1) < 0.3f * l0 * l1) return false;
          }
          return true;
        };
        if (!faceOk(a) || (moveB && !faceOk(b))) continue;
        // ---- apply
        Vec2 ua = hasUV ? C[F[s1] == a ? s1 : nxt(s1)] : Vec2{};
        Vec2 ub = hasUV ? C[F[s1] == b ? s1 : nxt(s1)] : Vec2{};
        Vec2 un = moveB ? (ua + ub) * 0.5f : ub;
        for (uint32_t k = start[a]; k < start[a + 1]; ++k) {
          uint32_t s = list[k];
          if (s / 3 == f1 || s / 3 == f2) continue;
          F[s] = b;
          if (hasUV) C[s] = un;
        }
        if (moveB && hasUV)
          for (uint32_t k = start[b]; k < start[b + 1]; ++k) C[list[k]] = un;
        F[f1 * 3] = kDead; F[f2 * 3] = kDead;
        P[b] = tp;
        touched[a] = touched[b] = 1;
        for (uint32_t n : na) touched[n] = 1;
        for (uint32_t n : nb) touched[n] = 1;
        ++did;
      }
      total += did;
      if (did < 4) break;
    }
    return total;
  }

  // ------------------------------------------------------------------- flip
  size_t flipEdges() {
    auto E = edges();
    std::vector<int> val(P.size(), 0);
    std::unordered_set<uint64_t> keys;
    keys.reserve(E.size());
    for (size_t i = 0; i < E.size();) {
      size_t j = i;
      while (j < E.size() && E[j].key == E[i].key) ++j;
      val[E[i].key >> 32]++; val[uint32_t(E[i].key)]++;
      keys.insert(E[i].key);
      i = j;
    }
    std::vector<uint8_t> dirty(faces(), 0);
    size_t did = 0;
    for (size_t i = 0; i < E.size();) {
      size_t j = i;
      while (j < E.size() && E[j].key == E[i].key) ++j;
      if (j - i != 2) { i = j; continue; }
      uint32_t s1 = E[i].slot, s2 = E[i + 1].slot;
      i = j;
      if (dirty[s1 / 3] || dirty[s2 / 3] || seam(s1, s2)) continue;
      uint32_t a = F[s1], b = F[nxt(s1)], c = F[prv(s1)], d = F[prv(s2)];
      if (c == d || keys.count(ekey(c, d))) continue;
      auto tgt = [&](uint32_t v) { return boundaryV[v] ? 4 : 6; };
      auto dev = [&](int da, int db, int dc, int dd) {
        auto q = [](int x) { return x * x; };
        return q(val[a] + da - tgt(a)) + q(val[b] + db - tgt(b)) + q(val[c] + dc - tgt(c)) + q(val[d] + dd - tgt(d));
      };
      if (dev(-1, -1, 1, 1) >= dev(0, 0, 0, 0)) continue;
      Vec3 n1 = normalize(cross(P[b] - P[a], P[c] - P[a])), n2 = normalize(cross(P[a] - P[b], P[d] - P[b]));
      if (dot(n1, n2) < 0.85f) continue;  // keep creases
      Vec3 m1 = cross(P[d] - P[a], P[c] - P[a]), m2 = cross(P[c] - P[b], P[d] - P[b]);
      Vec3 avg = normalize(n1 + n2);
      if (dot(normalize(m1), avg) < 0.5f || dot(normalize(m2), avg) < 0.5f) continue;
      Vec2 ua{}, ub{}, uc{}, ud{};
      if (hasUV) { ua = C[s1]; ub = C[nxt(s1)]; uc = C[prv(s1)]; ud = C[prv(s2)]; }
      uint32_t f1 = s1 / 3 * 3, f2 = s2 / 3 * 3;
      F[f1] = a; F[f1 + 1] = d; F[f1 + 2] = c;
      F[f2] = b; F[f2 + 1] = c; F[f2 + 2] = d;
      if (hasUV) { C[f1] = ua; C[f1 + 1] = ud; C[f1 + 2] = uc; C[f2] = ub; C[f2 + 1] = uc; C[f2 + 2] = ud; }
      val[a]--; val[b]--; val[c]++; val[d]++;
      keys.insert(ekey(c, d));
      dirty[f1 / 3] = dirty[f2 / 3] = 1;
      ++did;
    }
    return did;
  }

  // ----------------------------------------------------------------- smooth
  void relax(float lambda) {
    std::vector<uint32_t> start, list;
    corners(start, list);
    std::vector<Vec3> fn(faces());
    parallelFor(faces(), [&](size_t f) {
      if (alive(f)) fn[f] = cross(P[F[f * 3 + 1]] - P[F[f * 3]], P[F[f * 3 + 2]] - P[F[f * 3]]);
    }, 4096);
    std::vector<Vec3> np(P);
    std::vector<Vec2> nuv(P.size());
    std::vector<uint8_t> moved(P.size(), 0);
    parallelFor(P.size(), [&](size_t v) {
      if (lock[v] || start[v] == start[v + 1]) return;
      Vec3 q{}, n{};
      float cnt = 0;
      for (uint32_t k = start[v]; k < start[v + 1]; ++k) {
        uint32_t s = list[k];
        q += P[F[nxt(s)]]; q += P[F[prv(s)]]; cnt += 2;
        n += fn[s / 3];
      }
      q *= 1.0f / cnt;
      n = normalize(n);
      Vec3 d = q - P[v];
      d -= n * dot(n, d);
      Vec3 p = P[v] + d * lambda;
      if (ref) {
        ClosestHit h;
        if (ref->closest(p, h, L * L * 4.0f)) p = h.p;
      }
      np[v] = p;
      moved[v] = 1;
      if (hasUV) {  // uv from the closest face of the vertex' own (single chart) fan
        float best = 1e30f;
        Vec2 uv = C[list[start[v]]];
        for (uint32_t k = start[v]; k < start[v + 1]; ++k) {
          uint32_t f = list[k] / 3 * 3;
          float b0, b1, b2;
          Vec3 c = closestOnTriangle(p, P[F[f]], P[F[f + 1]], P[F[f + 2]], b0, b1, b2);
          float d2 = dot(c - p, c - p);
          if (d2 < best) { best = d2; uv = C[f] * b0 + C[f + 1] * b1 + C[f + 2] * b2; }
        }
        nuv[v] = uv;
      }
    }, 2048);
    P.swap(np);
    if (hasUV)
      parallelFor(F.size(), [&](size_t s) { if (F[s - s % 3] != kDead && moved[F[s]]) C[s] = nuv[F[s]]; }, 8192);
  }

  void compact() {
    size_t o = 0;
    for (size_t f = 0; f < faces(); ++f)
      if (alive(f)) {
        for (int k = 0; k < 3; ++k) { F[o * 3 + k] = F[f * 3 + k]; C[o * 3 + k] = C[f * 3 + k]; }
        ++o;
      }
    F.resize(o * 3); C.resize(o * 3);
    std::vector<uint32_t> remap(P.size(), kDead);
    uint32_t n = 0;
    for (auto& v : F) { if (remap[v] == kDead) remap[v] = n++; v = remap[v]; }
    std::vector<Vec3> np(n);
    std::vector<uint8_t> nl(n), nb(n);
    for (size_t v = 0; v < P.size(); ++v)
      if (remap[v] != kDead) { np[remap[v]] = P[v]; nl[remap[v]] = lock[v]; nb[remap[v]] = boundaryV[v]; }
    P.swap(np); lock.swap(nl); boundaryV.swap(nb);
  }
};

}  // namespace

void remeshMesh(Mesh& m, const RemeshParams& p, float smoothAngle) {
  if (m.triangleCount() < 2) return;
  removeDegenerateTriangles(m);
  // area & mean edge length
  double area = 0, elen = 0;
  for (size_t t = 0; t < m.triangleCount(); ++t) {
    Vec3 a = m.positions[m.indices[t * 3]], b = m.positions[m.indices[t * 3 + 1]], c = m.positions[m.indices[t * 3 + 2]];
    area += 0.5 * length(cross(b - a, c - a));
    elen += (length(b - a) + length(c - b) + length(a - c)) / 3.0;
  }
  elen /= double(m.triangleCount());
  float L = p.targetEdge > 0 ? p.targetEdge
          : p.targetFaces ? float(std::sqrt(4.0 * area / (std::sqrt(3.0) * double(p.targetFaces))))
          : float(elen);
  if (!(L > 0)) return;
  // Large reductions: let the quadric simplifier do the heavy lifting first.
  double expectFaces = area / (std::sqrt(3.0) / 4.0 * L * L);
  if (expectFaces * 3.0 < double(m.triangleCount()))
    simplifyMesh(m, float(std::max(expectFaces * 1.5, 4.0) / double(m.triangleCount())), true);

  if (!m.hasUV() && (m.uv1.size() || m.colors.size())) logVerbose("remesh: secondary attributes are dropped");
  // reference surface for projection
  std::vector<Vec3> refP = m.positions;
  std::vector<uint32_t> refI = m.indices;
  Bvh bvh;
  bvh.build(refP, refI);

  Remesher r;
  r.hasUV = m.hasUV();
  r.L = L; r.Lhi = L * 4.0f / 3.0f; r.Llo = L * 4.0f / 5.0f;
  r.ref = &bvh;
  {
    auto pid = positionIds(m);
    std::vector<uint32_t> compactId(m.positions.size(), kDead);
    for (size_t v = 0; v < m.positions.size(); ++v)
      if (pid[v] == v) { compactId[v] = uint32_t(r.P.size()); r.P.push_back(m.positions[v]); }
    r.F.resize(m.indices.size());
    r.C.resize(m.indices.size());
    for (size_t c = 0; c < m.indices.size(); ++c) {
      r.F[c] = compactId[pid[m.indices[c]]];
      if (r.hasUV) r.C[c] = m.uv0[m.indices[c]];
    }
  }
  Timer tm;
  for (int it = 0; it < std::max(1, p.iterations); ++it) {
    r.computeLocks();
    size_t s = r.splitLong();
    size_t c = r.collapseShort();
    r.compact();
    r.computeLocks();
    size_t f = r.flipEdges();
    r.relax(0.5f);
    logVerbose("remesh '%s' it %d: %zu splits, %zu collapses, %zu flips -> %zu faces", m.name.c_str(), it, s, c, f, r.faces());
  }
  // back to an indexed mesh
  Mesh out;
  out.name = m.name;
  out.material = m.material;
  out.positions.resize(r.F.size());
  out.indices.resize(r.F.size());
  if (r.hasUV) out.uv0 = r.C;
  for (size_t c = 0; c < r.F.size(); ++c) { out.positions[c] = r.P[r.F[c]]; out.indices[c] = uint32_t(c); }
  weldMesh(out);
  computeNormals(out, smoothAngle);
  logVerbose("remesh '%s': %zu -> %zu triangles in %.2fs (edge %.4g)", m.name.c_str(), m.triangleCount(), out.triangleCount(), tm.seconds(), L);
  m = std::move(out);
}

}  // namespace rx
