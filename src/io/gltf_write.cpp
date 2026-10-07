// Native glTF 2.0 / GLB writer. Vertex data is streamed straight from the meshes (no staging copy).
// Images are embedded as bufferViews (GLB) or written to a texture folder. WebP images use
// EXT_texture_webp (required), meshopt compression uses EXT_meshopt_compression.
// GLB is limited to 4 GB by the spec: larger outputs automatically switch to .gltf + .bin files.
#include "io/io.h"
#include "core/par.h"
#include "core/util.h"
#include <meshoptimizer.h>
#include <nlohmann/json.hpp>
#include <cstring>
#include <map>

namespace rx {
using json = nlohmann::json;

namespace {

struct Segment { const uint8_t* data; size_t size; std::shared_ptr<void> keep; };

struct BufferBuilder {
  std::vector<std::vector<Segment>> parts;  // one per output buffer
  std::vector<size_t> sizes;
  size_t maxPart;
  explicit BufferBuilder(size_t maxPartSize) : maxPart(maxPartSize) { parts.emplace_back(); sizes.push_back(0); }
  // returns (buffer, offset)
  std::pair<size_t, size_t> add(const void* d, size_t n, std::shared_ptr<void> keep = nullptr) {
    if (sizes.back() > 0 && sizes.back() + n > maxPart) { parts.emplace_back(); sizes.push_back(0); }
    size_t b = parts.size() - 1, off = sizes.back();
    parts.back().push_back({static_cast<const uint8_t*>(d), n, std::move(keep)});
    sizes.back() += (n + 3) & ~size_t(3);
    return {b, off};
  }
  size_t total() const { size_t t = 0; for (auto s : sizes) t += s; return t; }
};

template <class T>
std::shared_ptr<std::vector<T>> owned(std::vector<T>&& v) { return std::make_shared<std::vector<T>>(std::move(v)); }

}  // namespace

bool writeGltf(const Scene& s, const std::string& pathIn, const WriteSettings& w, bool binary, std::string& err) {
  const Options& o = *w.opts;
  std::string path = pathIn;
  const bool meshopt = o.compression == Compression::Meshopt;
  // estimate size to decide whether GLB is possible
  size_t estimate = 0;
  for (auto& m : s.meshes) estimate += m.memoryBytes();
  if (w.embedTextures) for (auto& im : s.images) if (im.finalData) estimate += im.finalData->size;
  if (binary && estimate > size_t(4000) << 20) {
    path = pathToUtf8(pathFromUtf8(path).replace_extension(".gltf"));
    logWarn("output is %s - over the 4 GB GLB limit; writing %s with external .bin files instead",
            humanBytes(double(estimate)).c_str(), path.c_str());
    binary = false;
  }
  BufferBuilder bb(binary ? ~size_t(0) >> 1 : size_t(1) << 30);
  // meshopt: main buffer holds compressed data, a second "fallback" buffer only declares sizes
  size_t fallbackSize = 0;

  json j;
  j["asset"] = {{"version", "2.0"}, {"generator", "Rx Twister"}};
  json views = json::array(), accessors = json::array(), meshesJ = json::array(), materialsJ = json::array();
  json imagesJ = json::array(), texturesJ = json::array(), nodesJ = json::array();
  std::vector<std::string> extUsed, extRequired;
  auto useExt = [&](const std::string& e, bool req) {
    if (std::find(extUsed.begin(), extUsed.end(), e) == extUsed.end()) extUsed.push_back(e);
    if (req && std::find(extRequired.begin(), extRequired.end(), e) == extRequired.end()) extRequired.push_back(e);
  };

  // Adds a bufferView; `count`/`elemSize` used for meshopt encoding. target: 34962 vertex / 34963 index
  auto addView = [&](const void* data, size_t bytes, size_t elemSize, int target, std::shared_ptr<void> keep,
                     bool isIndex = false, size_t count = 0) -> int {
    json v;
    if (meshopt && target != 0 && bytes) {
      std::vector<uint8_t> enc;
      std::string mode;
      if (isIndex) {
        enc.resize(meshopt_encodeIndexBufferBound(count, count));
        std::vector<uint32_t> tmp;
        const uint32_t* idx = static_cast<const uint32_t*>(data);
        if (elemSize == 2) { tmp.resize(count); for (size_t i = 0; i < count; ++i) tmp[i] = static_cast<const uint16_t*>(data)[i]; idx = tmp.data(); }
        enc.resize(meshopt_encodeIndexBuffer(enc.data(), enc.size(), idx, count));
        mode = "TRIANGLES";
      } else {
        size_t n = bytes / elemSize;
        enc.resize(meshopt_encodeVertexBufferBound(n, elemSize));
        enc.resize(meshopt_encodeVertexBuffer(enc.data(), enc.size(), data, n, elemSize));
        mode = "ATTRIBUTES";
      }
      auto keepEnc = owned(std::move(enc));
      auto [b, off] = bb.add(keepEnc->data(), keepEnc->size(), keepEnc);
      v["buffer"] = 1;
      v["byteOffset"] = fallbackSize;
      v["byteLength"] = bytes;
      if (!isIndex) v["byteStride"] = elemSize;
      v["extensions"]["EXT_meshopt_compression"] = {{"buffer", 0}, {"byteOffset", off}, {"byteLength", keepEnc->size()},
                                                    {"byteStride", elemSize}, {"count", isIndex ? count : bytes / elemSize},
                                                    {"mode", mode}};
      fallbackSize += (bytes + 3) & ~size_t(3);
      (void)b;
    } else {
      auto [b, off] = bb.add(data, bytes, std::move(keep));
      v["buffer"] = b;
      v["byteOffset"] = off;
      v["byteLength"] = bytes;
      if (target == 34962 && elemSize % 4 == 0) v["byteStride"] = elemSize;
    }
    if (target) v["target"] = target;
    views.push_back(v);
    return int(views.size()) - 1;
  };
  auto addAccessor = [&](int view, int ct, size_t count, const char* type, bool normalized = false) {
    json a = {{"bufferView", view}, {"componentType", ct}, {"count", count}, {"type", type}};
    if (normalized) a["normalized"] = true;
    accessors.push_back(a);
    return int(accessors.size()) - 1;
  };

  // ---- primitives (one per Scene mesh)
  std::vector<json> prims(s.meshes.size());
  for (size_t mi = 0; mi < s.meshes.size(); ++mi) {
    const Mesh& m = s.meshes[mi];
    const size_t vc = m.positions.size();
    if (!vc || m.indices.empty()) continue;
    json attrs;
    {
      int v = addView(m.positions.data(), vc * 12, 12, 34962, nullptr);
      int a = addAccessor(v, 5126, vc, "VEC3");
      Aabb box;
      for (auto& p : m.positions) box.add(p);
      accessors[a]["min"] = {box.lo.x, box.lo.y, box.lo.z};
      accessors[a]["max"] = {box.hi.x, box.hi.y, box.hi.z};
      attrs["POSITION"] = a;
    }
    if (m.hasNormals()) attrs["NORMAL"] = addAccessor(addView(m.normals.data(), vc * 12, 12, 34962, nullptr), 5126, vc, "VEC3");
    if (m.hasTangents()) attrs["TANGENT"] = addAccessor(addView(m.tangents.data(), vc * 16, 16, 34962, nullptr), 5126, vc, "VEC4");
    if (m.hasUV()) attrs["TEXCOORD_0"] = addAccessor(addView(m.uv0.data(), vc * 8, 8, 34962, nullptr), 5126, vc, "VEC2");
    if (m.uv1.size() == vc) attrs["TEXCOORD_1"] = addAccessor(addView(m.uv1.data(), vc * 8, 8, 34962, nullptr), 5126, vc, "VEC2");
    if (m.colors.size() == vc) attrs["COLOR_0"] = addAccessor(addView(m.colors.data(), vc * 4, 4, 34962, nullptr), 5121, vc, "VEC4", true);
    int idxAcc;
    if (vc <= 65535) {
      std::vector<uint16_t> i16(m.indices.size());
      for (size_t i = 0; i < i16.size(); ++i) i16[i] = uint16_t(m.indices[i]);
      auto k = owned(std::move(i16));
      idxAcc = addAccessor(addView(k->data(), k->size() * 2, 2, 34963, k, true, k->size()), 5123, m.indices.size(), "SCALAR");
    } else {
      idxAcc = addAccessor(addView(m.indices.data(), m.indices.size() * 4, 4, 34963, nullptr, true, m.indices.size()), 5125, m.indices.size(), "SCALAR");
    }
    json p = {{"attributes", attrs}, {"indices", idxAcc}, {"mode", 4}};
    if (m.material >= 0 && size_t(m.material) < s.materials.size()) p["material"] = m.material;
    prims[mi] = p;
  }

  // ---- images / textures
  std::vector<std::string> relPaths;
  if (!w.embedTextures) relPaths = writeTextureFiles(s, textureDirFor(path), pathToUtf8(pathFromUtf8(path).parent_path()));
  std::vector<int> texOfImage(s.images.size(), -1);
  for (size_t i = 0; i < s.images.size(); ++i) {
    const Image& im = s.images[i];
    if (!im.finalData) continue;
    json ji;
    ji["name"] = im.name;
    if (w.embedTextures) {
      ji["bufferView"] = addView(im.finalData->data, im.finalData->size, 1, 0, im.finalData->owner);
      ji["mimeType"] = im.finalMime;
    } else {
      ji["uri"] = percentEncodePath(relPaths[i]);
    }
    imagesJ.push_back(ji);
    int imgIdx = int(imagesJ.size()) - 1;
    json t = {{"sampler", 0}};
    if (im.finalMime == "image/webp") { t["extensions"]["EXT_texture_webp"] = {{"source", imgIdx}}; useExt("EXT_texture_webp", true); }
    else if (im.finalMime == "image/ktx2") { t["extensions"]["KHR_texture_basisu"] = {{"source", imgIdx}}; useExt("KHR_texture_basisu", true); }
    else t["source"] = imgIdx;
    texturesJ.push_back(t);
    texOfImage[i] = int(texturesJ.size()) - 1;
  }

  // ---- materials
  for (auto& m : s.materials) {
    json jm;
    if (!m.name.empty()) jm["name"] = m.name;
    auto tex = [&](const TextureSlot& t) -> json {
      if (!t.valid() || size_t(t.image) >= texOfImage.size() || texOfImage[t.image] < 0) return nullptr;
      json r = {{"index", texOfImage[t.image]}};
      if (t.uvSet) r["texCoord"] = t.uvSet;
      return r;
    };
    json pbr;
    pbr["baseColorFactor"] = {m.baseColor.x, m.baseColor.y, m.baseColor.z, m.baseColor.w};
    pbr["metallicFactor"] = m.metallic;
    pbr["roughnessFactor"] = m.roughness;
    if (auto t = tex(m.baseColorTex); !t.is_null()) pbr["baseColorTexture"] = t;
    if (auto t = tex(m.metalRoughTex); !t.is_null()) pbr["metallicRoughnessTexture"] = t;
    jm["pbrMetallicRoughness"] = pbr;
    if (auto t = tex(m.normalTex); !t.is_null()) { if (m.normalScale != 1.0f) t["scale"] = m.normalScale; jm["normalTexture"] = t; }
    if (auto t = tex(m.occlusionTex); !t.is_null()) { if (m.occlusionStrength != 1.0f) t["strength"] = m.occlusionStrength; jm["occlusionTexture"] = t; }
    if (auto t = tex(m.emissiveTex); !t.is_null()) jm["emissiveTexture"] = t;
    if (m.emissive.x + m.emissive.y + m.emissive.z > 0) jm["emissiveFactor"] = {m.emissive.x, m.emissive.y, m.emissive.z};
    if (m.emissiveStrength > 1.0f) { jm["extensions"]["KHR_materials_emissive_strength"] = {{"emissiveStrength", m.emissiveStrength}}; useExt("KHR_materials_emissive_strength", false); }
    if (m.unlit) { jm["extensions"]["KHR_materials_unlit"] = json::object(); useExt("KHR_materials_unlit", false); }
    if (m.alphaMode == AlphaMode::Blend) jm["alphaMode"] = "BLEND";
    else if (m.alphaMode == AlphaMode::Mask) { jm["alphaMode"] = "MASK"; jm["alphaCutoff"] = m.alphaCutoff; }
    if (m.doubleSided) jm["doubleSided"] = true;
    materialsJ.push_back(jm);
  }

  // ---- nodes / meshes (glTF mesh per distinct primitive list)
  std::map<std::vector<int>, int> meshCache;
  for (auto& n : s.nodes) {
    json jn;
    if (!n.name.empty()) jn["name"] = n.name;
    if (!n.local.isIdentity()) { json mtx = json::array(); for (float v : n.local.m) mtx.push_back(v); jn["matrix"] = mtx; }
    std::vector<int> list;
    for (int mi : n.meshes) if (mi >= 0 && size_t(mi) < prims.size() && !prims[mi].is_null()) list.push_back(mi);
    if (!list.empty()) {
      auto it = meshCache.find(list);
      if (it == meshCache.end()) {
        json jm;
        jm["name"] = s.meshes[list[0]].name;
        for (int mi : list) jm["primitives"].push_back(prims[mi]);
        meshesJ.push_back(jm);
        it = meshCache.emplace(list, int(meshesJ.size()) - 1).first;
      }
      jn["mesh"] = it->second;
    }
    if (!n.children.empty()) jn["children"] = n.children;
    nodesJ.push_back(jn);
  }
  j["scene"] = 0;
  j["scenes"] = json::array({json{{"nodes", s.roots}}});
  if (!nodesJ.empty()) j["nodes"] = nodesJ;
  if (!meshesJ.empty()) j["meshes"] = meshesJ;
  if (!materialsJ.empty()) j["materials"] = materialsJ;
  if (!texturesJ.empty()) {
    j["textures"] = texturesJ;
    j["images"] = imagesJ;
    j["samplers"] = json::array({json{{"magFilter", 9729}, {"minFilter", 9987}, {"wrapS", 10497}, {"wrapT", 10497}}});
  }
  if (!views.empty()) j["bufferViews"] = views;
  if (!accessors.empty()) j["accessors"] = accessors;
  if (meshopt) useExt("EXT_meshopt_compression", true);

  // ---- buffers
  json buffersJ = json::array();
  std::vector<std::string> binNames;
  fs::path outP = pathFromUtf8(path);
  for (size_t b = 0; b < bb.parts.size(); ++b) {
    if (bb.sizes[b] == 0 && b > 0) continue;
    json jb = {{"byteLength", bb.sizes[b]}};
    if (!binary) {
      std::string name = pathToUtf8(outP.stem()) + (bb.parts.size() > 1 ? "_" + std::to_string(b) : std::string()) + ".bin";
      jb["uri"] = percentEncodePath(name);
      binNames.push_back(name);
    }
    buffersJ.push_back(jb);
  }
  if (meshopt) {
    if (bb.parts.size() > 1) { err = "meshopt compression with multi-file output is not supported"; return false; }
    buffersJ.push_back({{"byteLength", fallbackSize}, {"extensions", {{"EXT_meshopt_compression", {{"fallback", true}}}}}});
  }
  if (bb.total() > 0 || meshopt) j["buffers"] = buffersJ;
  if (!extUsed.empty()) j["extensionsUsed"] = extUsed;
  if (!extRequired.empty()) j["extensionsRequired"] = extRequired;

  auto writeSegments = [&](FileWriter& f, const std::vector<Segment>& segs) {
    for (auto& sg : segs) { f.write(sg.data, sg.size); f.pad(((sg.size + 3) & ~size_t(3)) - sg.size); }
  };
  std::string js = j.dump(binary ? -1 : 1);
  if (binary) {
    while (js.size() % 4) js += ' ';
    uint64_t binLen = bb.sizes[0];
    uint64_t total = 12 + 8 + js.size() + (binLen ? 8 + binLen : 0);
    if (total > 0xFFFFFFFFull) { err = "GLB exceeds 4 GB"; return false; }
    FileWriter f(path);
    if (!f.ok()) { err = "cannot create " + path; return false; }
    uint32_t hdr[3] = {0x46546C67u, 2u, uint32_t(total)};
    f.write(hdr, 12);
    uint32_t jh[2] = {uint32_t(js.size()), 0x4E4F534Au};
    f.write(jh, 8); f.writeStr(js);
    if (binLen) {
      uint32_t bh[2] = {uint32_t(binLen), 0x004E4942u};
      f.write(bh, 8);
      writeSegments(f, bb.parts[0]);
    }
    if (!f.close()) { err = "write failed (disk full?)"; return false; }
  } else {
    FileWriter f(path);
    if (!f.ok()) { err = "cannot create " + path; return false; }
    f.writeStr(js);
    if (!f.close()) { err = "write failed"; return false; }
    size_t k = 0;
    for (size_t b = 0; b < bb.parts.size(); ++b) {
      if (bb.sizes[b] == 0 && b > 0) continue;
      FileWriter fb(pathToUtf8(outP.parent_path() / pathFromUtf8(binNames[k++])));
      writeSegments(fb, bb.parts[b]);
      if (!fb.close()) { err = "write failed (bin)"; return false; }
    }
  }
  return true;
}

}  // namespace rx
