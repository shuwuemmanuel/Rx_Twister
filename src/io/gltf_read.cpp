// Native glTF 2.0 / GLB reader: memory mapped, zero copy for images, parallel primitive decoding.
// Supports: KHR_mesh_quantization, EXT_meshopt_compression, KHR_draco_mesh_compression (if built),
// EXT_texture_webp, KHR_texture_basisu (pass-through), KHR_materials_unlit / emissive_strength /
// pbrSpecularGlossiness (approximated), sparse accessors, triangle strips / fans.
#include "io/io.h"
#include "core/par.h"
#include "core/util.h"
#include "tex/image_ops.h"
#include <meshoptimizer.h>
#include <nlohmann/json.hpp>
#include <cstring>
#if defined(RX_HAVE_DRACO)
#include <draco/compression/decode.h>
#endif

namespace rx {
using json = nlohmann::json;

namespace {

struct View { const uint8_t* data = nullptr; size_t size = 0; size_t stride = 0; std::shared_ptr<void> keep; };

int compCount(const std::string& t) {
  static const std::pair<const char*, int> k[] = {{"SCALAR", 1}, {"VEC2", 2}, {"VEC3", 3}, {"VEC4", 4}, {"MAT2", 4}, {"MAT3", 9}, {"MAT4", 16}};
  for (auto& [n, c] : k) if (t == n) return c;
  return 0;
}
size_t compSize(int ct) {
  switch (ct) { case 5120: case 5121: return 1; case 5122: case 5123: return 2; case 5125: case 5126: return 4; }
  return 0;
}
inline float readComp(const uint8_t* p, int ct, bool norm) {
  switch (ct) {
    case 5126: { float f; memcpy(&f, p, 4); return f; }
    case 5121: return norm ? *p / 255.0f : float(*p);
    case 5120: { int8_t v = int8_t(*p); return norm ? std::max(v / 127.0f, -1.0f) : float(v); }
    case 5123: { uint16_t v; memcpy(&v, p, 2); return norm ? v / 65535.0f : float(v); }
    case 5122: { int16_t v; memcpy(&v, p, 2); return norm ? std::max(v / 32767.0f, -1.0f) : float(v); }
    case 5125: { uint32_t v; memcpy(&v, p, 4); return float(v); }
  }
  return 0;
}
inline uint32_t readIndex(const uint8_t* p, int ct) {
  switch (ct) {
    case 5121: return *p;
    case 5123: { uint16_t v; memcpy(&v, p, 2); return v; }
    case 5125: { uint32_t v; memcpy(&v, p, 4); return v; }
  }
  return 0;
}

struct Reader {
  json j;
  std::string dir;
  std::shared_ptr<Blob> file;
  std::vector<std::shared_ptr<Blob>> buffers;
  std::vector<View> views;
  std::string err;

  bool loadBuffers(const uint8_t* bin, size_t binSize) {
    if (!j.contains("buffers")) return true;
    for (auto& b : j["buffers"]) {
      std::shared_ptr<Blob> blob;
      if (!b.contains("uri")) {
        if (bin) { blob = std::make_shared<Blob>(); blob->data = bin; blob->size = binSize; blob->owner = file; }
        else blob = std::make_shared<Blob>();  // e.g. meshopt fallback buffer without data
      } else {
        std::string uri = b["uri"];
        if (startsWith(uri, "data:")) {
          auto c = uri.find(',');
          blob = Blob::fromVector(base64Decode(std::string_view(uri).substr(c + 1)));
        } else {
          std::string p = pathToUtf8(pathFromUtf8(dir) / pathFromUtf8(percentDecode(uri)));
          blob = mapFile(p, &err);
          if (!blob) return false;
        }
      }
      buffers.push_back(blob);
    }
    return true;
  }

  bool loadViews() {
    if (!j.contains("bufferViews")) return true;
    for (auto& v : j["bufferViews"]) {
      View view;
      size_t off = v.value("byteOffset", size_t(0)), len = v.value("byteLength", size_t(0));
      view.stride = v.value("byteStride", size_t(0));
      auto ext = v.find("extensions");
      const json* mo = nullptr;
      if (ext != v.end()) {
        if (ext->contains("EXT_meshopt_compression")) mo = &(*ext)["EXT_meshopt_compression"];
        else if (ext->contains("KHR_meshopt_compression")) mo = &(*ext)["KHR_meshopt_compression"];
      }
      if (mo) {
        size_t buf = (*mo)["buffer"], moOff = mo->value("byteOffset", size_t(0)), moLen = (*mo)["byteLength"];
        size_t stride = (*mo)["byteStride"], count = (*mo)["count"];
        std::string mode = (*mo)["mode"], filter = mo->value("filter", std::string("NONE"));
        if (buf >= buffers.size() || moOff + moLen > buffers[buf]->size) { err = "meshopt view out of range"; return false; }
        auto out = std::make_shared<std::vector<uint8_t>>(count * stride);
        const uint8_t* src = buffers[buf]->data + moOff;
        int rc = -1;
        if (mode == "ATTRIBUTES") rc = meshopt_decodeVertexBuffer(out->data(), count, stride, src, moLen);
        else if (mode == "TRIANGLES") rc = meshopt_decodeIndexBuffer(out->data(), count, stride, src, moLen);
        else if (mode == "INDICES") rc = meshopt_decodeIndexSequence(out->data(), count, stride, src, moLen);
        if (rc != 0) { err = "meshopt decode failed"; return false; }
        if (filter == "OCTAHEDRAL") meshopt_decodeFilterOct(out->data(), count, stride);
        else if (filter == "QUATERNION") meshopt_decodeFilterQuat(out->data(), count, stride);
        else if (filter == "EXPONENTIAL") meshopt_decodeFilterExp(out->data(), count, stride);
        view.data = out->data(); view.size = out->size(); view.keep = out;
        if (!view.stride) view.stride = stride;
      } else {
        size_t buf = v.value("buffer", size_t(0));
        if (buf >= buffers.size()) { err = "bufferView references a missing buffer"; return false; }
        if (buffers[buf]->data && off + len > buffers[buf]->size) { err = "bufferView out of range"; return false; }
        view.data = buffers[buf]->data ? buffers[buf]->data + off : nullptr;
        view.size = len;
      }
      views.push_back(view);
    }
    return true;
  }

  // Generic accessor read into floats (comps per element).
  bool readFloats(int acc, int wantComps, std::vector<float>& out) const {
    const json& a = j["accessors"][acc];
    int ct = a["componentType"];
    int nc = compCount(a["type"]);
    size_t count = a["count"];
    bool norm = a.value("normalized", false);
    out.assign(count * wantComps, 0.0f);
    if (a.contains("bufferView")) {
      const View& v = views.at(a["bufferView"].get<size_t>());
      size_t off = a.value("byteOffset", size_t(0));
      size_t es = compSize(ct) * nc;
      size_t stride = v.stride ? v.stride : es;
      if (!v.data || (count && off + (count - 1) * stride + es > v.size)) return false;
      const int k = std::min(nc, wantComps);
      parallelRanges(count, 1 << 15, [&](size_t b, size_t e) {
        for (size_t i = b; i < e; ++i) {
          const uint8_t* p = v.data + off + i * stride;
          for (int c = 0; c < k; ++c) out[i * wantComps + c] = readComp(p + c * compSize(ct), ct, norm);
        }
      });
    }
    if (a.contains("sparse")) {
      const json& sp = a["sparse"];
      size_t n = sp["count"];
      const json& si = sp["indices"]; const json& sv = sp["values"];
      const View& iv = views.at(si["bufferView"].get<size_t>());
      const View& vv = views.at(sv["bufferView"].get<size_t>());
      int ict = si["componentType"];
      size_t ioff = si.value("byteOffset", size_t(0)), voff = sv.value("byteOffset", size_t(0));
      size_t es = compSize(ct) * nc;
      for (size_t i = 0; i < n; ++i) {
        uint32_t idx = readIndex(iv.data + ioff + i * compSize(ict), ict);
        if (idx >= count) continue;
        const uint8_t* p = vv.data + voff + i * es;
        for (int c = 0; c < std::min(nc, wantComps); ++c) out[idx * wantComps + c] = readComp(p + c * compSize(ct), ct, norm);
      }
    }
    return true;
  }

  bool readIndices(int acc, std::vector<uint32_t>& out) const {
    const json& a = j["accessors"][acc];
    int ct = a["componentType"];
    size_t count = a["count"];
    out.resize(count);
    if (!a.contains("bufferView")) { std::fill(out.begin(), out.end(), 0u); return true; }
    const View& v = views.at(a["bufferView"].get<size_t>());
    size_t off = a.value("byteOffset", size_t(0));
    size_t cs = compSize(ct), stride = v.stride ? v.stride : cs;
    if (!v.data || (count && off + (count - 1) * stride + cs > v.size)) return false;
    if (ct == 5125 && stride == 4) { memcpy(out.data(), v.data + off, count * 4); return true; }
    parallelRanges(count, 1 << 16, [&](size_t b, size_t e) {
      for (size_t i = b; i < e; ++i) out[i] = readIndex(v.data + off + i * stride, ct);
    });
    return true;
  }

#if defined(RX_HAVE_DRACO)
  bool readDraco(const json& prim, const json& ext, Mesh& m) const {
    const View& v = views.at(ext["bufferView"].get<size_t>());
    draco::DecoderBuffer db;
    db.Init(reinterpret_cast<const char*>(v.data), v.size);
    draco::Decoder dec;
    auto st = dec.DecodeMeshFromBuffer(&db);
    if (!st.ok()) return false;
    std::unique_ptr<draco::Mesh> dm = std::move(st).value();
    const size_t np = dm->num_points();
    auto attr = [&](const char* name, int comps, float* out, size_t stride) {
      if (!ext["attributes"].contains(name)) return false;
      const draco::PointAttribute* a = dm->GetAttributeByUniqueId(ext["attributes"][name].get<int>());
      if (!a) return false;
      float tmp[4] = {0, 0, 0, 0};
      for (size_t i = 0; i < np; ++i) {
        a->ConvertValue<float>(a->mapped_index(draco::PointIndex(uint32_t(i))), int8_t(comps), tmp);
        memcpy(out + i * stride, tmp, comps * sizeof(float));
      }
      return true;
    };
    m.positions.resize(np);
    attr("POSITION", 3, &m.positions[0].x, 3);
    m.normals.resize(np);
    if (!attr("NORMAL", 3, &m.normals[0].x, 3)) m.normals.clear();
    m.uv0.resize(np);
    if (!attr("TEXCOORD_0", 2, &m.uv0[0].x, 2)) m.uv0.clear();
    m.uv1.resize(np);
    if (!attr("TEXCOORD_1", 2, &m.uv1[0].x, 2)) m.uv1.clear();
    m.indices.resize(size_t(dm->num_faces()) * 3);
    for (uint32_t f = 0; f < dm->num_faces(); ++f) {
      const auto& face = dm->face(draco::FaceIndex(f));
      for (int k = 0; k < 3; ++k) m.indices[f * 3 + k] = face[k].value();
    }
    (void)prim;
    return true;
  }
#endif

  bool readPrimitive(const json& prim, Mesh& m) const {
    const json& at = prim["attributes"];
    if (prim.contains("extensions") && prim["extensions"].contains("KHR_draco_mesh_compression")) {
#if defined(RX_HAVE_DRACO)
      if (!readDraco(prim, prim["extensions"]["KHR_draco_mesh_compression"], m)) return false;
      m.material = prim.value("material", -1);
      return true;
#else
      if (!at.contains("POSITION") || !j["accessors"][at["POSITION"].get<int>()].contains("bufferView")) return false;
#endif
    }
    std::vector<float> f;
    if (!at.contains("POSITION") || !readFloats(at["POSITION"], 3, f)) return false;
    size_t vc = f.size() / 3;
    m.positions.resize(vc); memcpy(m.positions.data(), f.data(), f.size() * 4);
    if (at.contains("NORMAL") && readFloats(at["NORMAL"], 3, f)) { m.normals.resize(vc); memcpy(m.normals.data(), f.data(), vc * 12); }
    if (at.contains("TANGENT") && readFloats(at["TANGENT"], 4, f)) { m.tangents.resize(vc); memcpy(m.tangents.data(), f.data(), vc * 16); }
    if (at.contains("TEXCOORD_0") && readFloats(at["TEXCOORD_0"], 2, f)) { m.uv0.resize(vc); memcpy(m.uv0.data(), f.data(), vc * 8); }
    if (at.contains("TEXCOORD_1") && readFloats(at["TEXCOORD_1"], 2, f)) { m.uv1.resize(vc); memcpy(m.uv1.data(), f.data(), vc * 8); }
    if (at.contains("COLOR_0")) {
      int nc = compCount(j["accessors"][at["COLOR_0"].get<int>()]["type"]);
      if (readFloats(at["COLOR_0"], 4, f)) {
        m.colors.resize(vc);
        for (size_t i = 0; i < vc; ++i) {
          auto q = [](float x) { return uint8_t(std::clamp(x, 0.0f, 1.0f) * 255.0f + 0.5f); };
          m.colors[i] = {q(f[i * 4]), q(f[i * 4 + 1]), q(f[i * 4 + 2]), nc == 4 ? q(f[i * 4 + 3]) : uint8_t(255)};
        }
      }
    }
    std::vector<uint32_t> idx;
    if (prim.contains("indices")) { if (!readIndices(prim["indices"], idx)) return false; }
    else { idx.resize(vc); for (size_t i = 0; i < vc; ++i) idx[i] = uint32_t(i); }
    int mode = prim.value("mode", 4);
    if (mode == 4) m.indices.swap(idx);
    else if (mode == 5) {
      for (size_t i = 2; i < idx.size(); ++i) {
        if (i & 1) { m.indices.push_back(idx[i - 1]); m.indices.push_back(idx[i - 2]); }
        else { m.indices.push_back(idx[i - 2]); m.indices.push_back(idx[i - 1]); }
        m.indices.push_back(idx[i]);
      }
    } else if (mode == 6) {
      for (size_t i = 2; i < idx.size(); ++i) { m.indices.push_back(idx[0]); m.indices.push_back(idx[i - 1]); m.indices.push_back(idx[i]); }
    } else return false;  // points / lines
    m.indices.resize(m.indices.size() / 3 * 3);
    for (auto& i : m.indices) if (i >= vc) i = 0;
    m.material = prim.value("material", -1);
    return true;
  }
};

int textureImage(const json& j, const json& texInfo, int& uvSet) {
  if (!texInfo.is_object() || !texInfo.contains("index")) return -1;
  uvSet = texInfo.value("texCoord", 0);
  int t = texInfo["index"];
  if (!j.contains("textures") || t < 0 || size_t(t) >= j["textures"].size()) return -1;
  const json& tex = j["textures"][t];
  if (tex.contains("extensions")) {
    for (const char* e : {"EXT_texture_webp", "KHR_texture_basisu", "MSFT_texture_dds", "EXT_texture_avif"})
      if (tex["extensions"].contains(e) && tex["extensions"][e].contains("source")) return tex["extensions"][e]["source"];
  }
  return tex.value("source", -1);
}

}  // namespace

bool readGltf(const std::string& path, Scene& s, std::string& err) {
  Reader r;
  r.file = mapFile(path, &err);
  if (!r.file) return false;
  r.dir = pathToUtf8(pathFromUtf8(path).parent_path());
  const uint8_t* d = r.file->data;
  const uint8_t* bin = nullptr;
  size_t binSize = 0;
  try {
    if (r.file->size >= 20 && !memcmp(d, "glTF", 4)) {
      uint32_t ver, len, jlen, jtype;
      memcpy(&ver, d + 4, 4); memcpy(&len, d + 8, 4); memcpy(&jlen, d + 12, 4); memcpy(&jtype, d + 16, 4);
      if (ver != 2) { err = "only glTF 2.0 is supported"; return false; }
      if (jtype != 0x4E4F534A || 20 + size_t(jlen) > r.file->size) { err = "corrupt GLB"; return false; }
      r.j = json::parse(d + 20, d + 20 + jlen);
      size_t off = 20 + jlen;
      off = (off + 3) & ~size_t(3);
      if (off + 8 <= r.file->size) {
        uint32_t blen, btype;
        memcpy(&blen, d + off, 4); memcpy(&btype, d + off + 4, 4);
        if (btype == 0x004E4942) { bin = d + off + 8; binSize = std::min<size_t>(blen, r.file->size - off - 8); }
      }
    } else {
      r.j = json::parse(d, d + r.file->size);
    }
  } catch (const std::exception& e) { err = std::string("JSON: ") + e.what(); return false; }
  const json& j = r.j;
  if (j.contains("extensionsRequired"))
    for (auto& e : j["extensionsRequired"]) {
      std::string n = e;
      static const char* known[] = {"KHR_mesh_quantization", "EXT_meshopt_compression", "KHR_meshopt_compression", "EXT_texture_webp",
                                    "KHR_texture_basisu", "KHR_materials_unlit", "KHR_draco_mesh_compression", "KHR_texture_transform",
                                    "KHR_materials_pbrSpecularGlossiness", "MSFT_texture_dds"};
      bool ok = false;
      for (auto k : known) ok |= n == k;
#if !defined(RX_HAVE_DRACO)
      if (n == "KHR_draco_mesh_compression") { err = "file needs Draco decompression; rebuild with draco"; return false; }
#endif
      if (!ok) logWarn("glTF: required extension %s is not supported; results may be wrong", n.c_str());
    }
  try {
    if (!r.loadBuffers(bin, binSize) || !r.loadViews()) { err = r.err; return false; }

    // images (zero copy views into the mapped file)
    if (j.contains("images"))
      for (auto& ji : j["images"]) {
        Image im;
        im.name = ji.value("name", std::string());
        im.mime = ji.value("mimeType", std::string());
        if (ji.contains("bufferView")) {
          const View& v = r.views.at(ji["bufferView"].get<size_t>());
          auto b = std::make_shared<Blob>();
          b->data = v.data; b->size = v.size; b->owner = v.keep ? v.keep : std::shared_ptr<void>(r.file);
          if (v.keep == nullptr) b->owner = r.buffers.at(j["bufferViews"][ji["bufferView"].get<size_t>()].value("buffer", 0))->owner;
          im.encoded = b;
        } else if (ji.contains("uri")) {
          std::string uri = ji["uri"];
          if (startsWith(uri, "data:")) im.encoded = Blob::fromVector(base64Decode(std::string_view(uri).substr(uri.find(',') + 1)));
          else {
            std::string p = resolveTexturePath(r.dir, uri);
            if (!p.empty()) { im.encoded = mapFile(p); im.sourcePath = p; }
            if (im.name.empty()) im.name = stemOf(percentDecode(uri));
            if (!im.encoded) logWarn("glTF: missing image %s", uri.c_str());
          }
        }
        if (im.encoded) { std::string sm = sniffMime(im.encoded->data, im.encoded->size); if (!sm.empty()) im.mime = sm; }
        if (im.name.empty()) im.name = "image" + std::to_string(s.images.size());
        s.images.push_back(std::move(im));
      }

    // materials
    if (j.contains("materials"))
      for (auto& jm : j["materials"]) {
        Material m;
        m.name = jm.value("name", std::string());
        m.metallic = 1.0f;  // glTF defaults
        m.roughness = 1.0f;
        auto slot = [&](const json& ti, TextureSlot& t) { int uv = 0; int im = textureImage(j, ti, uv); if (im >= 0 && size_t(im) < s.images.size()) t = {im, uv}; };
        if (jm.contains("pbrMetallicRoughness")) {
          const json& p = jm["pbrMetallicRoughness"];
          if (p.contains("baseColorFactor")) { auto f = p["baseColorFactor"]; m.baseColor = {f[0], f[1], f[2], f[3]}; }
          m.metallic = p.value("metallicFactor", 1.0f);
          m.roughness = p.value("roughnessFactor", 1.0f);
          if (p.contains("baseColorTexture")) slot(p["baseColorTexture"], m.baseColorTex);
          if (p.contains("metallicRoughnessTexture")) slot(p["metallicRoughnessTexture"], m.metalRoughTex);
        }
        if (jm.contains("normalTexture")) { slot(jm["normalTexture"], m.normalTex); m.normalScale = jm["normalTexture"].value("scale", 1.0f); }
        if (jm.contains("occlusionTexture")) { slot(jm["occlusionTexture"], m.occlusionTex); m.occlusionStrength = jm["occlusionTexture"].value("strength", 1.0f); }
        if (jm.contains("emissiveTexture")) slot(jm["emissiveTexture"], m.emissiveTex);
        if (jm.contains("emissiveFactor")) { auto f = jm["emissiveFactor"]; m.emissive = {f[0], f[1], f[2]}; }
        std::string am = jm.value("alphaMode", std::string("OPAQUE"));
        m.alphaMode = am == "BLEND" ? AlphaMode::Blend : am == "MASK" ? AlphaMode::Mask : AlphaMode::Opaque;
        m.alphaCutoff = jm.value("alphaCutoff", 0.5f);
        m.doubleSided = jm.value("doubleSided", false);
        if (jm.contains("extensions")) {
          const json& e = jm["extensions"];
          if (e.contains("KHR_materials_unlit")) m.unlit = true;
          if (e.contains("KHR_materials_emissive_strength")) m.emissiveStrength = e["KHR_materials_emissive_strength"].value("emissiveStrength", 1.0f);
          if (e.contains("KHR_materials_pbrSpecularGlossiness")) {
            const json& sg = e["KHR_materials_pbrSpecularGlossiness"];
            if (sg.contains("diffuseFactor")) { auto f = sg["diffuseFactor"]; m.baseColor = {f[0], f[1], f[2], f[3]}; }
            if (sg.contains("diffuseTexture")) slot(sg["diffuseTexture"], m.baseColorTex);
            m.metallic = 0.0f;
            m.roughness = 1.0f - sg.value("glossinessFactor", 1.0f);
          }
        }
        s.materials.push_back(m);
      }

    // meshes: flatten primitives, decode in parallel
    struct PrimRef { size_t mesh, prim; };
    std::vector<PrimRef> prims;
    std::vector<std::vector<int>> meshPrims;
    if (j.contains("meshes"))
      for (size_t mi = 0; mi < j["meshes"].size(); ++mi) {
        meshPrims.emplace_back();
        for (size_t pi = 0; pi < j["meshes"][mi].value("primitives", json::array()).size(); ++pi) {
          meshPrims.back().push_back(int(prims.size()));
          prims.push_back({mi, pi});
        }
      }
    std::vector<Mesh> meshes(prims.size());
    std::vector<char> ok(prims.size(), 0);
    size_t morph = 0;
    parallelFor(prims.size(), [&](size_t i) {
      const json& jm = j["meshes"][prims[i].mesh];
      const json& p = jm["primitives"][prims[i].prim];
      Mesh& m = meshes[i];
      m.name = jm.value("name", std::string("mesh") + std::to_string(prims[i].mesh));
      if (jm["primitives"].size() > 1) m.name += "_" + std::to_string(prims[i].prim);
      ok[i] = r.readPrimitive(p, m) ? 1 : 0;
    }, 1);
    std::vector<int> primToMesh(prims.size(), -1);
    for (size_t i = 0; i < prims.size(); ++i) {
      const json& p = j["meshes"][prims[i].mesh]["primitives"][prims[i].prim];
      if (p.contains("targets")) ++morph;
      if (!ok[i]) { logVerbose("glTF: skipped primitive %zu of mesh %zu (unsupported mode or bad data)", prims[i].prim, prims[i].mesh); continue; }
      if (meshes[i].material >= int(s.materials.size())) meshes[i].material = -1;
      primToMesh[i] = int(s.meshes.size());
      s.meshes.push_back(std::move(meshes[i]));
    }

    // nodes
    if (j.contains("nodes")) {
      for (auto& jn : j["nodes"]) {
        Node n;
        n.name = jn.value("name", std::string());
        if (jn.contains("matrix")) { for (int k = 0; k < 16; ++k) n.local.m[k] = jn["matrix"][k]; }
        else {
          Mat4 t, rmat, sc;
          if (jn.contains("translation")) { auto v = jn["translation"]; t = Mat4::translation({v[0], v[1], v[2]}); }
          if (jn.contains("rotation")) { auto q = jn["rotation"]; rmat = Mat4::fromQuat(q[0], q[1], q[2], q[3]); }
          if (jn.contains("scale")) { auto v = jn["scale"]; sc = Mat4::scale({v[0], v[1], v[2]}); }
          n.local = t * rmat * sc;
        }
        if (jn.contains("mesh")) {
          size_t mi = jn["mesh"];
          if (mi < meshPrims.size()) for (int p : meshPrims[mi]) if (primToMesh[p] >= 0) n.meshes.push_back(primToMesh[p]);
        }
        if (jn.contains("children")) for (auto& c : jn["children"]) n.children.push_back(c);
        s.nodes.push_back(std::move(n));
      }
      int scene = j.value("scene", 0);
      if (j.contains("scenes") && size_t(scene) < j["scenes"].size() && j["scenes"][scene].contains("nodes")) {
        for (auto& n : j["scenes"][scene]["nodes"]) s.roots.push_back(n);
      } else {
        std::vector<char> child(s.nodes.size(), 0);
        for (auto& n : s.nodes) for (int c : n.children) if (c >= 0 && size_t(c) < child.size()) child[c] = 1;
        for (size_t i = 0; i < s.nodes.size(); ++i) if (!child[i]) s.roots.push_back(int(i));
      }
    } else {
      for (size_t i = 0; i < s.meshes.size(); ++i) { Node n; n.name = s.meshes[i].name; n.meshes = {int(i)}; s.nodes.push_back(n); s.roots.push_back(int(i)); }
    }
    if (j.contains("skins") && !j["skins"].empty()) logWarn("glTF: %zu skin(s) are not carried over (static geometry only)", j["skins"].size());
    if (j.contains("animations") && !j["animations"].empty()) logWarn("glTF: %zu animation(s) are not carried over", j["animations"].size());
    if (morph) logWarn("glTF: morph targets on %zu primitive(s) are not carried over", morph);
  } catch (const std::exception& e) {
    err = std::string("glTF: ") + e.what();
    return false;
  }
  s.sourceFormat = "gltf";
  return true;
}

}  // namespace rx
