// Tangent space normal map baking: every texel of the (simplified / remeshed) output surface looks
// up the closest point on the original high detail surface and stores its normal in the output
// tangent frame (glTF / OpenGL convention, +Y up).
#include "tex/textures.h"
#include "tex/image_ops.h"
#include "geom/bvh.h"
#include "geom/mesh_ops.h"
#include "core/par.h"
#include "core/util.h"
#include <cmath>
#include <cstring>

namespace rx {

namespace {
struct TriRef { uint32_t mesh, tri; };

void dilate(std::vector<uint8_t>& rgb, std::vector<uint8_t>& mask, int S, int passes) {
  for (int p = 0; p < passes; ++p) {
    std::vector<uint8_t> nrgb(rgb), nmask(mask);
    parallelFor(size_t(S), [&](size_t y) {
      for (int x = 0; x < S; ++x) {
        size_t i = y * S + x;
        if (mask[i]) continue;
        int acc[3] = {0, 0, 0}, n = 0;
        for (int dy = -1; dy <= 1; ++dy)
          for (int dx = -1; dx <= 1; ++dx) {
            int xx = x + dx, yy = int(y) + dy;
            if (xx < 0 || yy < 0 || xx >= S || yy >= S) continue;
            size_t j = size_t(yy) * S + xx;
            if (!mask[j]) continue;
            for (int c = 0; c < 3; ++c) acc[c] += rgb[j * 3 + c];
            ++n;
          }
        if (n) {
          for (int c = 0; c < 3; ++c) nrgb[i * 3 + c] = uint8_t(acc[c] / n);
          nmask[i] = 1;
        }
      }
    }, 16);
    rgb.swap(nrgb); mask.swap(nmask);
  }
}
}  // namespace

void bakeNormalMaps(Scene& s, const Scene& ref, const Options& o) {
  // Merge the reference into one world space triangle soup.
  std::vector<Vec3> RP, RN;
  std::vector<uint32_t> RI;
  for (const Mesh& m0 : ref.meshes) {
    const Mesh* m = &m0;
    Mesh tmp;
    if (!m0.hasNormals()) { tmp.positions = m0.positions; tmp.indices = m0.indices; computeNormals(tmp, 180); m = &tmp; }
    uint32_t base = uint32_t(RP.size());
    RP.insert(RP.end(), m->positions.begin(), m->positions.end());
    RN.insert(RN.end(), m->normals.begin(), m->normals.end());
    for (uint32_t i : m->indices) RI.push_back(base + i);
  }
  if (RI.empty()) return;
  Timer tm;
  Bvh bvh;
  bvh.build(RP, RI);
  Aabb box;
  for (auto& p : RP) box.add(p);
  const float maxD = 0.05f * length(box.size());
  int S = o.normalMapSize;
  if (o.maxTextureSize > 0) S = std::min(S, o.maxTextureSize);
  S = std::max(S, 16);
  const int TS = 64, NT = (S + TS - 1) / TS;
  int baked = 0;

  for (size_t mi = 0; mi < s.materials.size(); ++mi) {
    Material& mat = s.materials[mi];
    if (mat.unlit || (mat.normalTex.valid() && !o.forceNormalMap)) continue;
    std::vector<uint32_t> meshes;
    for (size_t k = 0; k < s.meshes.size(); ++k)
      if (s.meshes[k].material == int(mi) && s.meshes[k].hasUV() && s.meshes[k].triangleCount()) meshes.push_back(uint32_t(k));
    if (meshes.empty()) continue;
    for (uint32_t k : meshes) {
      Mesh& m = s.meshes[k];
      if (!m.hasNormals()) computeNormals(m, o.smoothAngle);
      computeTangents(m);
    }
    // bin triangles into tiles
    std::vector<std::vector<TriRef>> bins(size_t(NT) * NT);
    for (uint32_t k : meshes) {
      const Mesh& m = s.meshes[k];
      for (uint32_t t = 0; t < m.triangleCount(); ++t) {
        float x0 = 1e30f, y0 = 1e30f, x1 = -1e30f, y1 = -1e30f;
        for (int c = 0; c < 3; ++c) {
          Vec2 uv = m.uv0[m.indices[t * 3 + c]];
          x0 = std::min(x0, uv.x * S); x1 = std::max(x1, uv.x * S);
          y0 = std::min(y0, uv.y * S); y1 = std::max(y1, uv.y * S);
        }
        int tx0 = std::clamp(int(std::floor(x0)) / TS, 0, NT - 1), tx1 = std::clamp(int(std::ceil(x1)) / TS, 0, NT - 1);
        int ty0 = std::clamp(int(std::floor(y0)) / TS, 0, NT - 1), ty1 = std::clamp(int(std::ceil(y1)) / TS, 0, NT - 1);
        if (x1 < 0 || y1 < 0 || x0 > S || y0 > S) continue;
        for (int ty = ty0; ty <= ty1; ++ty)
          for (int tx = tx0; tx <= tx1; ++tx) bins[size_t(ty) * NT + tx].push_back({k, t});
      }
    }
    std::vector<uint8_t> rgb(size_t(S) * S * 3, 0), mask(size_t(S) * S, 0);
    parallelFor(bins.size(), [&](size_t b) {
      const int bx = int(b % NT) * TS, by = int(b / NT) * TS;
      const int ex = std::min(S, bx + TS), ey = std::min(S, by + TS);
      for (const TriRef& r : bins[b]) {
        const Mesh& m = s.meshes[r.mesh];
        const uint32_t* I = &m.indices[r.tri * 3];
        Vec2 a = m.uv0[I[0]] * float(S), bb = m.uv0[I[1]] * float(S), c = m.uv0[I[2]] * float(S);
        float area = (bb.x - a.x) * (c.y - a.y) - (bb.y - a.y) * (c.x - a.x);
        if (std::fabs(area) < 1e-12f) continue;
        float inv = 1.0f / area;
        int x0 = std::max(bx, int(std::floor(std::min({a.x, bb.x, c.x})))), x1 = std::min(ex - 1, int(std::ceil(std::max({a.x, bb.x, c.x}))));
        int y0 = std::max(by, int(std::floor(std::min({a.y, bb.y, c.y})))), y1 = std::min(ey - 1, int(std::ceil(std::max({a.y, bb.y, c.y}))));
        for (int y = y0; y <= y1; ++y)
          for (int x = x0; x <= x1; ++x) {
            float px = x + 0.5f, py = y + 0.5f;
            float w0 = ((bb.x - px) * (c.y - py) - (bb.y - py) * (c.x - px)) * inv;
            float w1 = ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) * inv;
            float w2 = 1.0f - w0 - w1;
            const float e = -1e-4f;
            if (w0 < e || w1 < e || w2 < e) continue;
            Vec3 P = m.positions[I[0]] * w0 + m.positions[I[1]] * w1 + m.positions[I[2]] * w2;
            Vec3 N = normalize(m.normals[I[0]] * w0 + m.normals[I[1]] * w1 + m.normals[I[2]] * w2);
            Vec4 t0 = m.tangents[I[0]], t1 = m.tangents[I[1]], t2 = m.tangents[I[2]];
            Vec3 T = Vec3{t0.x, t0.y, t0.z} * w0 + Vec3{t1.x, t1.y, t1.z} * w1 + Vec3{t2.x, t2.y, t2.z} * w2;
            T = normalize(T - N * dot(N, T));
            Vec3 B = cross(N, T) * t0.w;
            Vec3 Nh = N;
            ClosestHit h;
            if (bvh.closest(P, h, maxD * maxD)) {
              const uint32_t* J = &RI[h.tri * 3];
              Nh = normalize(RN[J[0]] * h.b0 + RN[J[1]] * h.b1 + RN[J[2]] * h.b2);
            }
            Vec3 ts{dot(Nh, T), dot(Nh, B), std::max(dot(Nh, N), 0.05f)};
            ts = normalize(ts);
            size_t i = size_t(y) * S + x;
            rgb[i * 3] = uint8_t(ts.x * 127.5f + 127.5f);
            rgb[i * 3 + 1] = uint8_t(ts.y * 127.5f + 127.5f);
            rgb[i * 3 + 2] = uint8_t(ts.z * 127.5f + 127.5f);
            mask[i] = 1;
          }
      }
    }, 1);
    dilate(rgb, mask, S, 8);
    auto px = makePixels(S, S, 128, 128, 255, 255);
    for (size_t i = 0; i < size_t(S) * S; ++i)
      if (mask[i]) memcpy(&px->rgba[i * 4], &rgb[i * 3], 3);
    Image im;
    im.name = (mat.name.empty() ? "material" + std::to_string(mi) : mat.name) + "_baked_normal";
    im.pixels = px;
    im.usage = kUseNormal;
    mat.normalTex = {s.addImage(std::move(im)), 0};
    mat.normalScale = 1.0f;
    ++baked;
  }
  if (baked) logInfo("normal maps: baked %d map(s) at %dpx from %zu reference triangles (%.2fs)", baked, S, RI.size() / 3, tm.seconds());
}

}  // namespace rx
