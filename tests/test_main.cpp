// Rx Twister end-to-end tests. Models are generated procedurally, converted, read back and checked.
#include "rx/rx.h"
#include "core/par.h"
#include "core/util.h"
#include "geom/bvh.h"
#include "geom/mesh_ops.h"
#include "geom/topology.h"
#include "io/io.h"
#include "tex/image_ops.h"
#include "tex/textures.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <nlohmann/json.hpp>
#if defined(RX_HAVE_DRACO)
#include <draco/compression/encode.h>
#include <draco/mesh/triangle_soup_mesh_builder.h>
#endif

using namespace rx;

static int g_fail = 0, g_pass = 0;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); ++g_fail; } else ++g_pass; } while (0)

static fs::path OUT;
static std::string P(const std::string& n) { return pathToUtf8(OUT / n); }

// UV sphere with a seam at u = 0/1 (duplicated column)
static Mesh uvSphere(int seg, int rings, float r = 1.0f) {
  Mesh m;
  m.name = "sphere";
  for (int y = 0; y <= rings; ++y)
    for (int x = 0; x <= seg; ++x) {
      float u = float(x) / seg, v = float(y) / rings;
      float th = u * 2 * kPi, ph = v * kPi;
      Vec3 p{r * std::sin(ph) * std::cos(th), r * std::cos(ph), r * std::sin(ph) * std::sin(th)};
      m.positions.push_back(p);
      m.normals.push_back(normalize(p));
      m.uv0.push_back({u, v});
    }
  for (int y = 0; y < rings; ++y)
    for (int x = 0; x < seg; ++x) {
      uint32_t a = y * (seg + 1) + x, b = a + 1, c = a + seg + 1, d = c + 1;
      if (y != 0) { m.indices.push_back(a); m.indices.push_back(b); m.indices.push_back(c); }
      if (y != rings - 1) { m.indices.push_back(b); m.indices.push_back(d); m.indices.push_back(c); }
    }
  return m;
}

static std::shared_ptr<Pixels> checker(int n, int cells, uint8_t r, uint8_t g, uint8_t b) {
  auto p = makePixels(n, n);
  for (int y = 0; y < n; ++y)
    for (int x = 0; x < n; ++x) {
      bool on = ((x * cells / n) + (y * cells / n)) & 1;
      uint8_t* q = &p->rgba[(size_t(y) * n + x) * 4];
      q[0] = on ? r : 30; q[1] = on ? g : 30; q[2] = on ? b : 30; q[3] = 255;
    }
  return p;
}
static void savePng(const std::shared_ptr<Pixels>& p, const std::string& path) {
  std::vector<uint8_t> out; std::string mime;
  encodeImage(*p, TexFormat::PNG, 90, false, out, mime);
  writeWholeFile(path, out.data(), out.size());
}

// Writes an OBJ by hand (independent of our writer).
static void writeObjFile(const Mesh& m, const std::string& path, const std::string& mtl, const std::string& mat) {
  std::string s;
  if (!mtl.empty()) s += "mtllib " + mtl + "\n";
  char b[128];
  for (auto& p : m.positions) { snprintf(b, sizeof b, "v %.6f %.6f %.6f\n", p.x, p.y, p.z); s += b; }
  for (auto& t : m.uv0) { snprintf(b, sizeof b, "vt %.6f %.6f\n", t.x, 1.0f - t.y); s += b; }
  for (auto& n : m.normals) { snprintf(b, sizeof b, "vn %.6f %.6f %.6f\n", n.x, n.y, n.z); s += b; }
  if (!mat.empty()) s += "usemtl " + mat + "\n";
  for (size_t i = 0; i < m.indices.size(); i += 3) {
    snprintf(b, sizeof b, "f %u/%u/%u %u/%u/%u %u/%u/%u\n", m.indices[i] + 1, m.indices[i] + 1, m.indices[i] + 1,
             m.indices[i + 1] + 1, m.indices[i + 1] + 1, m.indices[i + 1] + 1, m.indices[i + 2] + 1, m.indices[i + 2] + 1, m.indices[i + 2] + 1);
    s += b;
  }
  writeWholeFile(path, (const uint8_t*)s.data(), s.size());
}

static Result run(const std::string& in, const std::string& out, std::function<void(Options&)> f = nullptr) {
  Options o;
  o.input = in; o.output = out; o.quiet = true;
  if (f) f(o);
  Result r = convert(o);
  if (!r.ok) fprintf(stderr, "  convert %s -> %s failed: %s\n", in.c_str(), out.c_str(), r.error.c_str());
  return r;
}
static bool load(const std::string& path, Scene& s) {
  Options o; std::string err;
  bool ok = loadScene(path, s, o, err);
  if (!ok) fprintf(stderr, "  load %s: %s\n", path.c_str(), err.c_str());
  return ok;
}

// UV error: for each vertex of `out`, closest point on `ref` -> interpolated UV vs vertex UV.
static double uvAgreement(const Mesh& ref, const Mesh& out, double tol) {
  Bvh bvh; bvh.build(ref.positions, ref.indices);
  size_t good = 0, total = 0;
  Aabb box; for (auto& p : ref.positions) box.add(p);
  const float poleY = box.hi.y - 0.015f * (box.hi.y - box.lo.y);
  for (size_t v = 0; v < out.positions.size(); ++v) {
    // sphere poles: one position carries every u value, closest point is ambiguous
    if (std::fabs(out.positions[v].y) > poleY) continue;
    ++total;
    ClosestHit h;
    bvh.closest(out.positions[v], h);
    const uint32_t* I = &ref.indices[h.tri * 3];
    Vec2 uv = ref.uv0[I[0]] * h.b0 + ref.uv0[I[1]] * h.b1 + ref.uv0[I[2]] * h.b2;
    float du = std::fabs(uv.x - out.uv0[v].x), dv = std::fabs(uv.y - out.uv0[v].y);
    if (du > 0.5f) du = std::fabs(du - 1.0f);  // seam: u=0 and u=1 are the same place
    if (du < tol && dv < tol) ++good;
  }
  return double(good) / double(std::max<size_t>(1, total));
}

int main(int argc, char** argv) {
  OUT = argc > 1 ? fs::path(argv[1]) : fs::temp_directory_path() / "rx_tests";
  fs::remove_all(OUT);
  fs::create_directories(OUT);
  setLogLevel(LogLevel::Quiet);
  setvbuf(stdout, nullptr, _IONBF, 0);
  Mesh sphere = uvSphere(64, 32);

  // ---------------------------------------------------------------- base asset: OBJ + MTL + PNG
  savePng(checker(512, 8, 220, 60, 40), P("albedo.png"));
  {
    std::string mtl = "newmtl Painted\nKd 1 1 1\nNs 50\nmap_Kd albedo.png\n";
    writeWholeFile(P("sphere.mtl"), (const uint8_t*)mtl.data(), mtl.size());
    writeObjFile(sphere, P("sphere.obj"), "sphere.mtl", "Painted");
  }
  printf("[1] OBJ -> GLB (default WebP embedded)\n");
  {
    Result r = run(P("sphere.obj"), P("sphere.glb"));
    CHECK(r.ok);
    CHECK(r.trianglesOut == sphere.triangleCount());
    Scene s; CHECK(load(P("sphere.glb"), s));
    CHECK(s.images.size() == 1);
    CHECK(s.images.size() == 1 && s.images[0].mime == "image/webp");
    CHECK(s.totalTriangles() == sphere.triangleCount());
    CHECK(s.meshes.size() == 1 && s.meshes[0].hasUV() && s.meshes[0].hasNormals());
    CHECK(s.materials.size() == 1 && s.materials[0].baseColorTex.valid());
    if (!s.meshes.empty()) CHECK(uvAgreement(sphere, s.meshes[0], 1e-4) > 0.999);
  }
  printf("[2] texture resize + PNG + external folder\n");
  {
    Result r = run(P("sphere.obj"), P("ext/sphere.gltf"), [](Options& o) { o.maxTextureSize = 128; o.embedTextures = false; o.texFormat = TexFormat::PNG; });
    CHECK(r.ok);
    CHECK(fs::exists(OUT / "ext/sphere_textures/albedo.png"));
    Scene s; CHECK(load(P("ext/sphere.gltf"), s));
    int w = 0, h = 0;
    CHECK(s.images.size() == 1 && imageSize(s.images[0], w, h) && w == 128 && h == 128);
  }
  printf("[3] GLB -> OBJ / STL / PLY / USDA / USDZ / FBX / DAE / 3MF round trips\n");
  for (const char* ext : {"obj", "stl", "ply", "usda", "usdz", "fbx", "dae", "3mf", "x3d", "3ds"}) {
    std::string out = P(std::string("rt/sphere.") + ext);
    Result r = run(P("sphere.glb"), out);
    CHECK(r.ok);
    CHECK(fs::exists(pathFromUtf8(out)) && fs::file_size(pathFromUtf8(out)) > 1000);
    std::string e = ext;
    if (e == "usda" || e == "usdz" || e == "x3d") continue;  // write-only (usdz verified below)
    Scene s;
    if (!load(out, s)) { CHECK(false); continue; }
    size_t tris = s.totalTriangles();
    if (tris != sphere.triangleCount()) fprintf(stderr, "  %s: %zu triangles\n", ext, tris);
    CHECK(tris == sphere.triangleCount());
    if (e == "obj" || e == "fbx" || e == "dae" || e == "3ds") {
      CHECK(s.images.size() >= 1);
      if (!s.meshes.empty() && s.meshes[0].hasUV()) {
        Mesh flat = s.meshes[0];
        auto world = worldMatrices(s);
        for (size_t n = 0; n < s.nodes.size(); ++n) for (int mi : s.nodes[n].meshes) if (mi == 0) transformMesh(flat, world[n]);
        double agree = uvAgreement(sphere, flat, 2e-3);
        if (agree < 0.99) fprintf(stderr, "  %s UV agreement %.3f\n", ext, agree);
        CHECK(agree > 0.99);
      } else CHECK(false);
    }
  }
  {
    // USDZ: zip with 64 byte aligned members
    std::vector<uint8_t> z; readWholeFile(P("rt/sphere.usdz"), z);
    CHECK(z.size() > 100 && z[0] == 'P' && z[1] == 'K');
    bool aligned = true; size_t files = 0;
    for (size_t p = 0; p + 30 < z.size();) {
      uint32_t sig; memcpy(&sig, &z[p], 4);
      if (sig != 0x04034b50) break;
      uint16_t nl, xl; uint32_t sz; memcpy(&sz, &z[p + 18], 4); memcpy(&nl, &z[p + 26], 2); memcpy(&xl, &z[p + 28], 2);
      size_t data = p + 30 + nl + xl;
      aligned &= data % 64 == 0; ++files;
      p = data + sz;
    }
    CHECK(aligned && files == 2);
  }
  printf("[4] FBX embeds textures\n");
  {
    Scene s; CHECK(load(P("rt/sphere.fbx"), s));
    bool embedded = false;
    for (auto& im : s.images) embedded |= im.sourcePath.empty() && (im.encoded || im.pixels);
    CHECK(embedded);
  }
  printf("[5] subdivision (Loop + linear) keeps UVs\n");
  {
    Mesh m = sphere; weldMesh(m);
    Mesh lin = m;
    subdivideMesh(lin, 2, Subdivision::Linear);
    CHECK(lin.triangleCount() == m.triangleCount() * 16);
    CHECK(uvAgreement(m, lin, 1e-4) > 0.999);   // linear: exact positions + UVs on the old surface
    Mesh lp = m;
    subdivideMesh(lp, 1, Subdivision::Loop);
    CHECK(lp.triangleCount() == m.triangleCount() * 4);
    // Loop: seam vertices with identical positions must stay identical (no cracks)
    auto pid = positionIds(m);
    size_t cracks = 0;
    for (size_t v = 0; v < m.positions.size(); ++v) {
      Vec3 a = lp.positions[v], b = lp.positions[pid[v]];
      if (length(a - b) > 1e-6f) ++cracks;
    }
    CHECK(cracks == 0);
    Result r = run(P("sphere.glb"), P("sub.glb"), [](Options& o) { o.subdivide = 1; });
    CHECK(r.ok && r.trianglesOut == r.trianglesIn * 4);
  }
  printf("[6] unsubdivide (UV aware simplification)\n");
  {
    Result r = run(P("sub.glb"), P("unsub.glb"), [](Options& o) { o.unsubdivide = 1; });
    CHECK(r.ok);
    double ratio = double(r.trianglesOut) / double(r.trianglesIn);
    if (ratio > 0.3 || ratio < 0.2) fprintf(stderr, "  ratio %.3f\n", ratio);
    CHECK(ratio < 0.3 && ratio > 0.2);
    Scene s; CHECK(load(P("unsub.glb"), s));
    if (!s.meshes.empty()) { double a = uvAgreement(sphere, s.meshes[0], 0.01); if (a < 0.97) fprintf(stderr, "  unsub UV agreement %.3f\n", a); CHECK(a > 0.97); }
  }
  printf("[7] remesh preserves UVs\n");
  {
    Mesh m = sphere; weldMesh(m);
    Mesh dense = m; subdivideMesh(dense, 1, Subdivision::Loop);
    Mesh r = dense;
    RemeshParams rp; rp.targetFaces = 3000; rp.iterations = 5;
    remeshMesh(r, rp, 60);
    double ratio = double(r.triangleCount()) / 3000.0;
    fprintf(stderr, "  remesh: %zu -> %zu tris (target 3000)\n", dense.triangleCount(), r.triangleCount());
    CHECK(ratio > 0.5 && ratio < 2.0);
    CHECK(r.hasUV());
    double a = uvAgreement(dense, r, 0.01);
    fprintf(stderr, "  remesh UV agreement %.4f\n", a);
    CHECK(a > 0.97);
    // triangle quality improves: mean min-angle
    auto quality = [](const Mesh& mm) {
      double q = 0;
      for (size_t t = 0; t < mm.triangleCount(); ++t) {
        Vec3 p[3] = {mm.positions[mm.indices[t * 3]], mm.positions[mm.indices[t * 3 + 1]], mm.positions[mm.indices[t * 3 + 2]]};
        float mn = 1e9f;
        for (int k = 0; k < 3; ++k) {
          Vec3 e1 = normalize(p[(k + 1) % 3] - p[k]), e2 = normalize(p[(k + 2) % 3] - p[k]);
          mn = std::min(mn, std::acos(std::clamp(dot(e1, e2), -1.0f, 1.0f)));
        }
        q += mn;
      }
      return q / double(std::max<size_t>(1, mm.triangleCount())) * 180.0 / double(kPi);
    };
    double q0 = quality(dense), q1 = quality(r);
    fprintf(stderr, "  mean min angle %.1f -> %.1f deg\n", q0, q1);
    CHECK(q1 > 35.0);
    Result rr = run(P("sphere.glb"), P("remesh.glb"), [](Options& o) { o.remesh = true; o.remeshFaces = 2000; });
    CHECK(rr.ok);
  }
  printf("[8] normal map: bake after reduction / height from albedo\n");
  {
    {  // bumpy high-poly sphere
      Scene hs;
      Mesh m = sphere; weldMesh(m);
      subdivideMesh(m, 2, Subdivision::Loop);
      for (auto& p : m.positions) {
        Vec3 n = normalize(p);
        float th = std::atan2(n.z, n.x), ph = std::acos(std::clamp(n.y, -1.0f, 1.0f));
        p = n * (1.0f + 0.03f * std::sin(12 * th) * std::sin(12 * ph));
      }
      m.normals.clear();
      computeNormals(m, 180);
      m.material = 0;
      Material mat; mat.name = "bumpy"; hs.materials.push_back(mat);
      hs.meshes.push_back(m);
      Node nd; nd.meshes = {0}; hs.nodes.push_back(nd); hs.roots = {0};
      Options o; o.quiet = true; std::string err;
      CHECK(saveScene(hs, P("bumpy.glb"), o, err));
    }
    Result r = run(P("bumpy.glb"), P("baked.glb"), [](Options& o) { o.unsubdivide = 2; o.normalGen = NormalGen::Auto; o.normalMapSize = 256; });
    CHECK(r.ok);
    Scene s; CHECK(load(P("baked.glb"), s));
    CHECK(s.materials.size() == 1 && s.materials[0].normalTex.valid());
    if (!s.materials.empty() && s.materials[0].normalTex.valid()) {
      auto px = imagePixels(s.images[s.materials[0].normalTex.image]);
      CHECK(px && px->width == 256);
      // mostly "up" (blue) normals with variation
      double z = 0, var = 0;
      for (size_t i = 0; i < size_t(px->width) * px->height; ++i) { z += px->rgba[i * 4 + 2]; var += std::fabs(px->rgba[i * 4] - 128.0); }
      z /= double(px->width) * px->height; var /= double(px->width) * px->height;
      fprintf(stderr, "  baked mean z %.1f, mean |x| %.2f\n", z, var);
      CHECK(z > 200);
      CHECK(var > 3.0);
    }
    Result r2 = run(P("sphere.glb"), P("height.glb"), [](Options& o) { o.normalGen = NormalGen::Height; });
    Scene s2; CHECK(r2.ok && load(P("height.glb"), s2));
    CHECK(s2.materials.size() == 1 && s2.materials[0].normalTex.valid());
    Result r3 = run(P("sphere.glb"), P("nonormal.glb"));
    Scene s3; CHECK(r3.ok && load(P("nonormal.glb"), s3));
    CHECK(s3.materials.size() == 1 && !s3.materials[0].normalTex.valid());
  }
  printf("[9] loose texture binding (name conventions + DX normals + separate roughness/metal)\n");
  {
    fs::create_directories(OUT / "loose/textures");
    writeObjFile(sphere, P("loose/rock.obj"), "", "");
    savePng(checker(256, 4, 200, 180, 150), P("loose/textures/Rock_BaseColor.png"));
    savePng(makePixels(256, 256, 128, 128, 255), P("loose/textures/Rock_Normal_DX.png"));
    savePng(makePixels(256, 256, 180, 180, 180), P("loose/textures/Rock_Roughness.png"));
    savePng(makePixels(128, 128, 20, 20, 20), P("loose/textures/Rock_Metallic.png"));
    savePng(makePixels(256, 256, 250, 250, 250), P("loose/textures/Rock_AO.png"));
    Result r = run(P("loose/rock.obj"), P("loose/rock.glb"));
    CHECK(r.ok);
    Scene s; CHECK(load(P("loose/rock.glb"), s));
    CHECK(s.materials.size() == 1);
    if (s.materials.size() == 1) {
      const Material& m = s.materials[0];
      CHECK(m.baseColorTex.valid() && m.normalTex.valid() && m.metalRoughTex.valid() && m.occlusionTex.valid());
      CHECK(m.occlusionTex.image == m.metalRoughTex.image);  // ORM packed
      if (m.metalRoughTex.valid()) {
        auto px = imagePixels(s.images[m.metalRoughTex.image]);
        CHECK(px && std::abs(px->rgba[1] - 180) < 6 && std::abs(px->rgba[2] - 20) < 6 && std::abs(px->rgba[0] - 250) < 6);
      }
    }
    // explicit --textures folder
    Result r2 = run(P("sphere.obj"), P("loose/explicit.glb"), [](Options& o) { o.textureSources = {P("loose/textures")}; });
    Scene s2; CHECK(r2.ok && load(P("loose/explicit.glb"), s2));
    CHECK(s2.materials.size() == 1 && s2.materials[0].normalTex.valid());
  }
  printf("[10] UDIM tiles -> atlas with UV remap\n");
  {
    fs::create_directories(OUT / "udim");
    Mesh m = sphere;
    for (auto& t : m.uv0) t.x *= 2.0f;  // u in [0,2] -> tiles 1001, 1002
    writeObjFile(m, P("udim/body.obj"), "", "");
    savePng(makePixels(128, 128, 255, 0, 0), P("udim/body_basecolor.1001.png"));
    savePng(makePixels(128, 128, 0, 0, 255), P("udim/body_basecolor.1002.png"));
    Result r = run(P("udim/body.obj"), P("udim/body.glb"), [](Options& o) { o.textureSources = {P("udim")}; });
    CHECK(r.ok);
    Scene s; CHECK(load(P("udim/body.glb"), s));
    CHECK(s.images.size() == 1);
    int w = 0, h = 0;
    if (!s.images.empty()) { imageSize(s.images[0], w, h); CHECK(w == 256 && h == 128); }
    float maxU = 0;
    for (auto& t : s.meshes[0].uv0) maxU = std::max(maxU, t.x);
    CHECK(maxU <= 1.0001f && maxU > 0.9f);
    // left half red (tile 1001), right half blue (tile 1002)
    if (!s.images.empty()) {
      auto px = imagePixels(s.images[0]);
      CHECK(px && px->rgba[(64 * 256 + 10) * 4] > 200 && px->rgba[(64 * 256 + 246) * 4 + 2] > 200);
    }
  }
  printf("[11] material atlas merges textures of many materials\n");
  {
    fs::create_directories(OUT / "atlas");
    std::string mtl;
    std::string obj = "mtllib multi.mtl\n";
    char b[160];
    // three quads with their own material + texture
    for (int i = 0; i < 3; ++i) {
      std::string tn = "tex" + std::to_string(i) + ".png";
      savePng(checker(64, 2, uint8_t(80 * i + 50), 100, 200), P("atlas/" + tn));
      mtl += "newmtl m" + std::to_string(i) + "\nKd 1 1 1\nmap_Kd " + tn + "\n";
      float x = float(i) * 2;
      snprintf(b, sizeof b, "v %g 0 0\nv %g 0 0\nv %g 1 0\nv %g 1 0\n", x, x + 1, x + 1, x); obj += b;
      obj += "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\n";
      int o0 = i * 4 + 1;
      snprintf(b, sizeof b, "usemtl m%d\nf %d/%d %d/%d %d/%d\nf %d/%d %d/%d %d/%d\n", i, o0, o0, o0 + 1, o0 + 1, o0 + 2, o0 + 2, o0, o0, o0 + 2, o0 + 2, o0 + 3, o0 + 3);
      obj += b;
    }
    writeWholeFile(P("atlas/multi.mtl"), (const uint8_t*)mtl.data(), mtl.size());
    writeWholeFile(P("atlas/multi.obj"), (const uint8_t*)obj.data(), obj.size());
    Result r = run(P("atlas/multi.obj"), P("atlas/multi.glb"), [](Options& o) { o.atlas = true; o.mergeMeshes = true; });
    CHECK(r.ok);
    Scene s; CHECK(load(P("atlas/multi.glb"), s));
    CHECK(s.materials.size() == 1 && s.images.size() == 1 && s.meshes.size() == 1);
  }
  printf("[12] meshopt compression + gltf round trip\n");
  {
    Result r = run(P("sub.glb"), P("comp.glb"), [](Options& o) { o.compression = Compression::Meshopt; });
    CHECK(r.ok);
    Scene s; CHECK(load(P("comp.glb"), s));
    CHECK(s.totalTriangles() == r.trianglesOut);
    Result plain = run(P("sub.glb"), P("plain.glb"));
    fprintf(stderr, "  plain %s, meshopt %s\n", humanBytes(double(plain.bytesOut)).c_str(), humanBytes(double(r.bytesOut)).c_str());
    CHECK(r.bytesOut < plain.bytesOut);
  }
  printf("[13] presets / units / axis\n");
  {
    Result r = run(P("sphere.glb"), P("unreal/sphere"), [](Options& o) { o.preset = Preset::Unreal; });
    CHECK(r.ok && extOf(r.output) == "fbx");
    Result z = run(P("sphere.glb"), P("zup.glb"), [](Options& o) { o.setUp = true; o.targetUp = UpAxis::Z; o.setUnits = true; o.targetMetersPerUnit = 0.01; });
    Scene s; CHECK(z.ok && load(P("zup.glb"), s));
    Aabb bb = s.bounds();
    CHECK(std::fabs(bb.size().z - 200.0f) < 1.0f);
  }
  printf("[13b] Draco-compressed glTF input\n");
#if defined(RX_HAVE_DRACO)
  {
    const size_t tc = sphere.triangleCount();
    draco::TriangleSoupMeshBuilder b;
    b.Start(int(tc));
    int pa = b.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
    int ta = b.AddAttribute(draco::GeometryAttribute::TEX_COORD, 2, draco::DT_FLOAT32);
    for (size_t f = 0; f < tc; ++f) {
      const uint32_t* I = &sphere.indices[f * 3];
      b.SetAttributeValuesForFace(pa, draco::FaceIndex(uint32_t(f)), &sphere.positions[I[0]], &sphere.positions[I[1]], &sphere.positions[I[2]]);
      b.SetAttributeValuesForFace(ta, draco::FaceIndex(uint32_t(f)), &sphere.uv0[I[0]], &sphere.uv0[I[1]], &sphere.uv0[I[2]]);
    }
    std::unique_ptr<draco::Mesh> dm = b.Finalize();
    draco::Encoder enc;
    enc.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 16);
    enc.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD, 14);
    draco::EncoderBuffer buf;
    CHECK(enc.EncodeMeshToBuffer(*dm, &buf).ok());
    using nlohmann::json;
    size_t np = dm->num_points();
    json j = {{"asset", {{"version", "2.0"}}}, {"scene", 0}, {"scenes", {{{"nodes", {0}}}}}, {"nodes", {{{"mesh", 0}}}},
              {"extensionsUsed", {"KHR_draco_mesh_compression"}}, {"extensionsRequired", {"KHR_draco_mesh_compression"}},
              {"buffers", {{{"byteLength", buf.size()}}}}, {"bufferViews", {{{"buffer", 0}, {"byteLength", buf.size()}}}},
              {"accessors", {{{"componentType", 5126}, {"count", np}, {"type", "VEC3"}, {"min", {-1, -1, -1}}, {"max", {1, 1, 1}}},
                             {{"componentType", 5126}, {"count", np}, {"type", "VEC2"}},
                             {{"componentType", 5125}, {"count", tc * 3}, {"type", "SCALAR"}}}},
              {"meshes", {{{"primitives", {{{"attributes", {{"POSITION", 0}, {"TEXCOORD_0", 1}}}, {"indices", 2},
                  {"extensions", {{"KHR_draco_mesh_compression", {{"bufferView", 0},
                      {"attributes", {{"POSITION", dm->attribute(pa)->unique_id()}, {"TEXCOORD_0", dm->attribute(ta)->unique_id()}}}}}}}}}}}}}};
    std::string js = j.dump();
    while (js.size() % 4) js += ' ';
    std::vector<uint8_t> glb;
    auto u32 = [&](uint32_t v) { glb.insert(glb.end(), (uint8_t*)&v, (uint8_t*)&v + 4); };
    size_t binLen = (buf.size() + 3) & ~size_t(3);
    u32(0x46546C67); u32(2); u32(uint32_t(12 + 8 + js.size() + 8 + binLen));
    u32(uint32_t(js.size())); u32(0x4E4F534A); glb.insert(glb.end(), js.begin(), js.end());
    u32(uint32_t(binLen)); u32(0x004E4942); glb.insert(glb.end(), buf.data(), buf.data() + buf.size()); glb.resize(glb.size() + binLen - buf.size(), 0);
    writeWholeFile(P("draco.glb"), glb.data(), glb.size());
    Scene s;
    CHECK(load(P("draco.glb"), s));
    CHECK(s.totalTriangles() == tc);
    if (!s.meshes.empty() && s.meshes[0].hasUV()) {
      double a = uvAgreement(sphere, s.meshes[0], 2e-3);
      if (a < 0.99) fprintf(stderr, "  draco UV agreement %.3f\n", a);
      CHECK(a > 0.99);
    } else CHECK(false);
    Result r = run(P("draco.glb"), P("draco_out.glb"));
    CHECK(r.ok && r.trianglesOut == tc);
  }
#else
  printf("  (built without Draco - skipped)\n");
#endif
  printf("[14] Blender bridge\n");
  if (!findBlender("").empty()) {
    Result r = run(P("sphere.glb"), P("blend/sphere.blend"));
    CHECK(r.ok);
    Result back = run(P("blend/sphere.blend"), P("blend/back.glb"));
    CHECK(back.ok && back.trianglesOut == sphere.triangleCount());
    Scene s; CHECK(load(P("blend/back.glb"), s));
    CHECK(s.images.size() == 1);  // packed texture survived
    Result abc = run(P("sphere.glb"), P("blend/sphere.abc"));
    CHECK(abc.ok || abc.error.find("not available in this Blender build") != std::string::npos);
  } else printf("  (Blender not found - skipped)\n");

  printf("\n%d checks passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
