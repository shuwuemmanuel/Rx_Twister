// USD writer: .usda (text) and .usdz (zero-compression zip package, 64 byte aligned members).
// Materials use UsdPreviewSurface, understood by Unity, Unreal, Blender, Omniverse and Apple QuickLook.
#include "io/io.h"
#include "core/par.h"
#include "core/util.h"
#include <charconv>
#include <cstring>
#include <map>
#include <set>
#include <functional>

namespace rx {

namespace {

std::string primName(const std::string& n, const std::string& fallback, std::set<std::string>& used) {
  std::string r;
  for (unsigned char c : n) r += (std::isalnum(c) || c == '_') ? char(c) : '_';
  if (r.empty()) r = fallback;
  if (std::isdigit((unsigned char)r[0])) r = "_" + r;
  std::string b = r;
  for (int k = 1; used.count(r); ++k) r = b + "_" + std::to_string(k);
  used.insert(r);
  return r;
}

inline void putF(std::string& o, float v) {
  char buf[32];
  if (std::fabs(v) < 1e-30f) v = 0;
  o.append(buf, formatFloat(buf, v));
}

template <class F>
void arrayParallel(std::string& o, size_t n, F&& item) {
  const size_t blk = 1 << 15;
  size_t nb = (n + blk - 1) / blk;
  std::vector<std::string> parts(nb);
  parallelFor(nb, [&](size_t b) {
    std::string& s = parts[b];
    for (size_t i = b * blk; i < std::min(n, (b + 1) * blk); ++i) { if (i) s += ", "; item(s, i); }
  }, 1);
  o += "[";
  for (auto& p : parts) o += p;
  o += "]";
}

uint32_t crc32(const uint8_t* d, size_t n) {
  static uint32_t T[256];
  static bool init = false;
  if (!init) { for (uint32_t i = 0; i < 256; ++i) { uint32_t c = i; for (int k = 0; k < 8; ++k) c = c & 1 ? 0xEDB88320u ^ (c >> 1) : c >> 1; T[i] = c; } init = true; }
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i) c = T[(c ^ d[i]) & 0xFF] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

struct ZipEntry { std::string name; const uint8_t* data; size_t size; };

bool writeUsdz(const std::string& path, const std::vector<ZipEntry>& files) {
  FileWriter f(path);
  if (!f.ok()) return false;
  struct Central { std::string name; uint32_t crc, size, offset; };
  std::vector<Central> cd;
  auto w16 = [&](uint16_t v) { f.write(&v, 2); };
  auto w32 = [&](uint32_t v) { f.write(&v, 4); };
  for (auto& e : files) {
    if (e.size > 0xFFFFFFFFull) return false;
    uint32_t crc = crc32(e.data, e.size);
    uint32_t off = uint32_t(f.position());
    size_t headerEnd = f.position() + 30 + e.name.size() + 4;  // + extra field header
    uint16_t pad = uint16_t((64 - headerEnd % 64) % 64);
    w32(0x04034b50); w16(20); w16(0); w16(0); w16(0); w16(0); w32(crc); w32(uint32_t(e.size)); w32(uint32_t(e.size));
    w16(uint16_t(e.name.size())); w16(uint16_t(4 + pad));
    f.writeStr(e.name);
    w16(0x1986); w16(pad); f.pad(pad);  // alignment padding extra field
    f.write(e.data, e.size);
    cd.push_back({e.name, crc, uint32_t(e.size), off});
  }
  uint32_t cdStart = uint32_t(f.position());
  for (auto& c : cd) {
    w32(0x02014b50); w16(20); w16(20); w16(0); w16(0); w16(0); w16(0); w32(c.crc); w32(c.size); w32(c.size);
    w16(uint16_t(c.name.size())); w16(0); w16(0); w16(0); w16(0); w32(0); w32(c.offset);
    f.writeStr(c.name);
  }
  uint32_t cdSize = uint32_t(f.position()) - cdStart;
  w32(0x06054b50); w16(0); w16(0); w16(uint16_t(cd.size())); w16(uint16_t(cd.size())); w32(cdSize); w32(cdStart); w16(0);
  return f.close();
}

}  // namespace

bool writeUsd(const Scene& s, const std::string& path, const WriteSettings& w, bool zip, std::string& err) {
  (void)w;
  fs::path out = pathFromUtf8(path);
  // texture paths
  std::vector<std::string> tex(s.images.size());
  std::vector<ZipEntry> zipFiles;
  if (zip) {
    std::set<std::string> used;
    for (size_t i = 0; i < s.images.size(); ++i) {
      if (!s.images[i].finalData) continue;
      std::string b = sanitizeFileName(s.images[i].name.empty() ? "tex" + std::to_string(i) : stemOf(s.images[i].name));
      std::string n = "textures/" + b + s.images[i].finalExt;
      for (int k = 1; used.count(n); ++k) n = "textures/" + b + "_" + std::to_string(k) + s.images[i].finalExt;
      used.insert(n);
      tex[i] = n;
    }
  } else {
    tex = writeTextureFiles(s, textureDirFor(path), pathToUtf8(out.parent_path()));
  }
  std::string o;
  o.reserve(1 << 20);
  char buf[512];
  snprintf(buf, sizeof buf, "#usda 1.0\n(\n    defaultPrim = \"Root\"\n    metersPerUnit = %g\n    upAxis = \"%s\"\n    doc = \"Rx Twister\"\n)\n\n",
           s.metersPerUnit, s.up == UpAxis::Z ? "Z" : "Y");
  o += buf;
  o += "def Xform \"Root\"\n{\n";
  // materials
  std::vector<std::string> matPath(s.materials.size());
  if (!s.materials.empty()) {
    o += "    def Scope \"Materials\"\n    {\n";
    std::set<std::string> used;
    for (size_t mi = 0; mi < s.materials.size(); ++mi) {
      const Material& m = s.materials[mi];
      std::string name = primName(m.name, "Material" + std::to_string(mi), used);
      std::string mp = "/Root/Materials/" + name;
      matPath[mi] = mp;
      o += "        def Material \"" + name + "\"\n        {\n";
      o += "            token outputs:surface.connect = <" + mp + "/Surface.outputs:surface>\n";
      std::string shaders;
      auto texNode = [&](const char* node, const TextureSlot& t, bool srgb, const char* scale, const char* bias) -> bool {
        if (!t.valid() || tex[t.image].empty()) return false;
        shaders += std::string("            def Shader \"") + node + "\"\n            {\n                uniform token info:id = \"UsdUVTexture\"\n";
        shaders += "                asset inputs:file = @" + tex[t.image] + "@\n";
        shaders += "                float2 inputs:st.connect = <" + mp + "/UV.outputs:result>\n";
        shaders += std::string("                token inputs:sourceColorSpace = \"") + (srgb ? "sRGB" : "raw") + "\"\n";
        shaders += "                token inputs:wrapS = \"repeat\"\n                token inputs:wrapT = \"repeat\"\n";
        if (scale) shaders += std::string("                float4 inputs:scale = ") + scale + "\n";
        if (bias) shaders += std::string("                float4 inputs:bias = ") + bias + "\n";
        shaders += "                float3 outputs:rgb\n                float outputs:r\n                float outputs:g\n                float outputs:b\n                float outputs:a\n            }\n";
        return true;
      };
      std::string surf = "            def Shader \"Surface\"\n            {\n                uniform token info:id = \"UsdPreviewSurface\"\n";
      char sc[160];
      snprintf(sc, sizeof sc, "(%g, %g, %g, %g)", m.baseColor.x, m.baseColor.y, m.baseColor.z, m.baseColor.w);
      if (texNode("BaseColorTex", m.baseColorTex, true, sc, nullptr)) {
        surf += "                color3f inputs:diffuseColor.connect = <" + mp + "/BaseColorTex.outputs:rgb>\n";
        if (m.alphaMode != AlphaMode::Opaque) surf += "                float inputs:opacity.connect = <" + mp + "/BaseColorTex.outputs:a>\n";
      } else {
        snprintf(buf, sizeof buf, "                color3f inputs:diffuseColor = (%g, %g, %g)\n", m.baseColor.x, m.baseColor.y, m.baseColor.z);
        surf += buf;
        if (m.alphaMode != AlphaMode::Opaque) { snprintf(buf, sizeof buf, "                float inputs:opacity = %g\n", m.baseColor.w); surf += buf; }
      }
      if (m.alphaMode == AlphaMode::Mask) { snprintf(buf, sizeof buf, "                float inputs:opacityThreshold = %g\n", m.alphaCutoff); surf += buf; }
      snprintf(sc, sizeof sc, "(1, %g, %g, 1)", m.roughness, m.metallic);
      if (texNode("MetalRoughTex", m.metalRoughTex, false, sc, nullptr)) {
        surf += "                float inputs:roughness.connect = <" + mp + "/MetalRoughTex.outputs:g>\n";
        surf += "                float inputs:metallic.connect = <" + mp + "/MetalRoughTex.outputs:b>\n";
      } else {
        snprintf(buf, sizeof buf, "                float inputs:roughness = %g\n                float inputs:metallic = %g\n", m.roughness, m.metallic);
        surf += buf;
      }
      snprintf(sc, sizeof sc, "(%g, %g, 2, 1)", 2 * m.normalScale, 2 * m.normalScale);
      char bi[96];
      snprintf(bi, sizeof bi, "(%g, %g, -1, 0)", -m.normalScale, -m.normalScale);
      if (texNode("NormalTex", m.normalTex, false, sc, bi)) surf += "                normal3f inputs:normal.connect = <" + mp + "/NormalTex.outputs:rgb>\n";
      if (m.occlusionTex.valid() && m.occlusionTex.image == m.metalRoughTex.image && !tex[m.occlusionTex.image].empty())
        surf += "                float inputs:occlusion.connect = <" + mp + "/MetalRoughTex.outputs:r>\n";
      else if (texNode("OcclusionTex", m.occlusionTex, false, nullptr, nullptr))
        surf += "                float inputs:occlusion.connect = <" + mp + "/OcclusionTex.outputs:r>\n";
      Vec3 e = m.emissive * m.emissiveStrength;
      snprintf(sc, sizeof sc, "(%g, %g, %g, 1)", e.x, e.y, e.z);
      if (texNode("EmissiveTex", m.emissiveTex, true, sc, nullptr)) surf += "                color3f inputs:emissiveColor.connect = <" + mp + "/EmissiveTex.outputs:rgb>\n";
      else if (e.x + e.y + e.z > 0) { snprintf(buf, sizeof buf, "                color3f inputs:emissiveColor = (%g, %g, %g)\n", e.x, e.y, e.z); surf += buf; }
      surf += "                token outputs:surface\n            }\n";
      o += surf;
      o += "            def Shader \"UV\"\n            {\n                uniform token info:id = \"UsdPrimvarReader_float2\"\n"
           "                string inputs:varname = \"st\"\n                float2 outputs:result\n            }\n";
      o += shaders;
      o += "        }\n";
    }
    o += "    }\n";
  }
  // node hierarchy
  std::set<std::string> rootUsed{"Materials"};
  std::vector<char> visited(s.nodes.size(), 0);
  std::function<void(int, int, std::set<std::string>&)> node = [&](int ni, int depth, std::set<std::string>& used) {
    if (ni < 0 || size_t(ni) >= s.nodes.size() || visited[ni]) return;
    visited[ni] = 1;
    const Node& n = s.nodes[ni];
    std::string ind(size_t(depth) * 4, ' ');
    o += ind + "def Xform \"" + primName(n.name, "Node" + std::to_string(ni), used) + "\"\n" + ind + "{\n";
    if (!n.local.isIdentity()) {
      o += ind + "    matrix4d xformOp:transform = (";
      for (int r = 0; r < 4; ++r) {
        snprintf(buf, sizeof buf, "(%.9g, %.9g, %.9g, %.9g)%s", n.local.m[r * 4], n.local.m[r * 4 + 1], n.local.m[r * 4 + 2], n.local.m[r * 4 + 3], r < 3 ? ", " : "");
        o += buf;
      }
      o += ")\n" + ind + "    uniform token[] xformOpOrder = [\"xformOp:transform\"]\n";
    }
    std::set<std::string> childUsed;
    for (int mi : n.meshes) {
      const Mesh& m = s.meshes[mi];
      if (m.indices.empty()) continue;
      const size_t vc = m.positions.size(), tc = m.triangleCount();
      std::string ii = ind + "    ";
      o += ii + "def Mesh \"" + primName(m.name, "Mesh" + std::to_string(mi), childUsed) + "\"";
      if (m.material >= 0 && size_t(m.material) < s.materials.size()) o += " (\n" + ii + "    prepend apiSchemas = [\"MaterialBindingAPI\"]\n" + ii + ")";
      o += "\n" + ii + "{\n";
      Aabb box;
      for (auto& p : m.positions) box.add(p);
      snprintf(buf, sizeof buf, "%s    float3[] extent = [(%g, %g, %g), (%g, %g, %g)]\n", ii.c_str(), box.lo.x, box.lo.y, box.lo.z, box.hi.x, box.hi.y, box.hi.z);
      o += buf;
      o += ii + "    int[] faceVertexCounts = ";
      { std::string threes; threes.reserve(tc * 3 + 2); threes += "["; for (size_t t = 0; t < tc; ++t) { if (t) threes += ", "; threes += '3'; } threes += "]"; o += threes; }
      o += "\n" + ii + "    int[] faceVertexIndices = ";
      arrayParallel(o, m.indices.size(), [&](std::string& x, size_t i) { char b2[16]; auto r = std::to_chars(b2, b2 + 16, m.indices[i]); x.append(b2, r.ptr); });
      o += "\n" + ii + "    point3f[] points = ";
      arrayParallel(o, vc, [&](std::string& x, size_t i) { x += '('; putF(x, m.positions[i].x); x += ", "; putF(x, m.positions[i].y); x += ", "; putF(x, m.positions[i].z); x += ')'; });
      o += "\n";
      if (m.hasNormals()) {
        o += ii + "    normal3f[] normals = ";
        arrayParallel(o, vc, [&](std::string& x, size_t i) { x += '('; putF(x, m.normals[i].x); x += ", "; putF(x, m.normals[i].y); x += ", "; putF(x, m.normals[i].z); x += ')'; });
        o += " (\n" + ii + "        interpolation = \"vertex\"\n" + ii + "    )\n";
      }
      if (m.hasUV()) {
        o += ii + "    texCoord2f[] primvars:st = ";
        arrayParallel(o, vc, [&](std::string& x, size_t i) { x += '('; putF(x, m.uv0[i].x); x += ", "; putF(x, 1.0f - m.uv0[i].y); x += ')'; });
        o += " (\n" + ii + "        interpolation = \"vertex\"\n" + ii + "    )\n";
      }
      if (m.colors.size() == vc) {
        o += ii + "    color3f[] primvars:displayColor = ";
        arrayParallel(o, vc, [&](std::string& x, size_t i) { x += '('; putF(x, m.colors[i].r / 255.f); x += ", "; putF(x, m.colors[i].g / 255.f); x += ", "; putF(x, m.colors[i].b / 255.f); x += ')'; });
        o += " (\n" + ii + "        interpolation = \"vertex\"\n" + ii + "    )\n";
      }
      o += ii + "    uniform token subdivisionScheme = \"none\"\n";
      if (m.material >= 0 && size_t(m.material) < s.materials.size()) {
        if (s.materials[m.material].doubleSided) o += ii + "    uniform bool doubleSided = 1\n";
        o += ii + "    rel material:binding = <" + matPath[m.material] + ">\n";
      }
      o += ii + "}\n";
    }
    for (int c : n.children) node(c, depth + 1, childUsed);
    o += ind + "}\n";
  };
  for (int r : s.roots) node(r, 1, rootUsed);
  o += "}\n";

  if (!zip) {
    if (!writeWholeFile(path, reinterpret_cast<const uint8_t*>(o.data()), o.size())) { err = "cannot write " + path; return false; }
    return true;
  }
  zipFiles.push_back({pathToUtf8(out.stem()) + ".usda", reinterpret_cast<const uint8_t*>(o.data()), o.size()});
  for (size_t i = 0; i < s.images.size(); ++i)
    if (!tex[i].empty()) zipFiles.push_back({tex[i], s.images[i].finalData->data, s.images[i].finalData->size});
  if (!writeUsdz(path, zipFiles)) { err = "cannot write " + path; return false; }
  return true;
}

}  // namespace rx
