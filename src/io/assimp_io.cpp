// Assimp bridge: imports ~40 formats (FBX, DAE, 3DS, BLEND (legacy), X, LWO, IFC, ...) and exports
// FBX, Collada, 3DS, X, X3D, 3MF, STEP, PLY, STL, OBJ, assbin ... The Scene <-> aiScene mapping
// keeps the node hierarchy, PBR factors and every texture slot.
#include "io/io.h"
#include "core/par.h"
#include "core/util.h"
#include "tex/image_ops.h"
#if defined(RX_HAVE_ASSIMP)
#include <assimp/Exporter.hpp>
#include <assimp/GltfMaterial.h>
#include <assimp/Importer.hpp>
#include <assimp/importerdesc.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#endif
#include <cstring>
#include <map>
#include <functional>

namespace rx {

#if defined(RX_HAVE_ASSIMP)

bool assimpAvailable() { return true; }

std::vector<std::pair<std::string, std::string>> assimpImportFormats() {
  Assimp::Importer imp;
  std::vector<std::pair<std::string, std::string>> r;
  for (size_t i = 0; i < imp.GetImporterCount(); ++i) {
    const aiImporterDesc* d = imp.GetImporterInfo(i);
    std::string exts = d->mFileExtensions;
    size_t p = 0;
    while (p < exts.size()) {
      size_t e = exts.find(' ', p);
      if (e == std::string::npos) e = exts.size();
      if (e > p) r.push_back({toLower(exts.substr(p, e - p)), d->mName});
      p = e + 1;
    }
  }
  return r;
}

std::vector<std::pair<std::string, std::string>> assimpExportFormats() {
  Assimp::Exporter ex;
  std::vector<std::pair<std::string, std::string>> r;
  for (size_t i = 0; i < ex.GetExportFormatCount(); ++i) {
    const aiExportFormatDesc* d = ex.GetExportFormatDescription(i);
    r.push_back({d->id, std::string(d->description) + " (." + d->fileExtension + ")"});
  }
  return r;
}

bool assimpCanRead(const std::string& ext) {
  Assimp::Importer imp;
  return imp.IsExtensionSupported("." + ext);
}

std::string assimpExportId(const std::string& ext) {
  static const std::map<std::string, std::string> preferred = {
      {"fbx", "fbx"}, {"dae", "collada"}, {"3ds", "3ds"}, {"x", "x"}, {"x3d", "x3d"}, {"3mf", "3mf"},
      {"stp", "stp"}, {"step", "stp"}, {"assbin", "assbin"}, {"assxml", "assxml"}, {"json", "assjson"}, {"pbrt", "pbrt"}};
  auto it = preferred.find(ext);
  Assimp::Exporter ex;
  for (size_t i = 0; i < ex.GetExportFormatCount(); ++i) {
    const aiExportFormatDesc* d = ex.GetExportFormatDescription(i);
    if (it != preferred.end() ? it->second == d->id : ext == d->fileExtension) return d->id;
  }
  return "";
}

namespace {

Mat4 toMat(const aiMatrix4x4& a) {
  Mat4 m;
  const float* r = &a.a1;  // row major
  for (int row = 0; row < 4; ++row) for (int col = 0; col < 4; ++col) m.m[col * 4 + row] = r[row * 4 + col];
  return m;
}
aiMatrix4x4 fromMat(const Mat4& m) {
  aiMatrix4x4 a;
  float* r = &a.a1;
  for (int row = 0; row < 4; ++row) for (int col = 0; col < 4; ++col) r[row * 4 + col] = m.m[col * 4 + row];
  return a;
}

int textureFrom(const aiScene* sc, const aiMaterial* am, aiTextureType type, Scene& s, const std::string& dir,
                std::map<std::string, int>& cache, int& uv) {
  if (am->GetTextureCount(type) == 0) return -1;
  aiString p;
  unsigned int uvIndex = 0;
  if (am->GetTexture(type, 0, &p, nullptr, &uvIndex) != AI_SUCCESS) return -1;
  uv = int(uvIndex);
  std::string key = p.C_Str();
  if (key.empty()) return -1;
  auto it = cache.find(key);
  if (it != cache.end()) return it->second;
  int idx = -1;
  if (const aiTexture* t = sc->GetEmbeddedTexture(p.C_Str())) {
    Image im;
    im.name = t->mFilename.length ? stemOf(t->mFilename.C_Str()) : "embedded" + std::to_string(s.images.size());
    if (t->mHeight == 0) {
      std::vector<uint8_t> v(reinterpret_cast<const uint8_t*>(t->pcData), reinterpret_cast<const uint8_t*>(t->pcData) + t->mWidth);
      im.encoded = Blob::fromVector(std::move(v));
      im.mime = sniffMime(im.encoded->data, im.encoded->size);
      if (im.mime.empty()) { im.pixels = decodeImage(im.encoded->data, im.encoded->size); im.encoded.reset(); }
    } else {
      auto px = makePixels(int(t->mWidth), int(t->mHeight));
      for (size_t i = 0; i < size_t(t->mWidth) * t->mHeight; ++i) {
        const aiTexel& x = t->pcData[i];
        px->rgba[i * 4] = x.r; px->rgba[i * 4 + 1] = x.g; px->rgba[i * 4 + 2] = x.b; px->rgba[i * 4 + 3] = x.a;
        px->hasAlpha |= x.a != 255;
      }
      im.pixels = px;
    }
    if (im.encoded || im.pixels) idx = s.addImage(std::move(im));
  } else {
    std::string f = resolveTexturePath(dir, key);
    if (!f.empty()) idx = loadTextureFile(s, f);
    else logWarn("texture '%s' not found", key.c_str());
  }
  cache[key] = idx;
  return idx;
}

}  // namespace

bool readAssimp(const std::string& path, Scene& s, std::string& err) {
  Assimp::Importer imp;
  imp.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);
  imp.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);
  imp.SetPropertyBool(AI_CONFIG_IMPORT_FBX_READ_ANIMATIONS, false);
  unsigned flags = aiProcess_Triangulate | aiProcess_SortByPType | aiProcess_ValidateDataStructure;
  const aiScene* sc = imp.ReadFile(path, flags);
  if (!sc || !sc->mRootNode) { err = imp.GetErrorString(); return false; }
  std::string dir = pathToUtf8(pathFromUtf8(path).parent_path());
  // materials
  std::map<std::string, int> texCache;
  for (unsigned i = 0; i < sc->mNumMaterials; ++i) {
    const aiMaterial* am = sc->mMaterials[i];
    Material m;
    aiString name;
    if (am->Get(AI_MATKEY_NAME, name) == AI_SUCCESS) m.name = name.C_Str();
    aiColor4D c;
    if (am->Get(AI_MATKEY_BASE_COLOR, c) == AI_SUCCESS) m.baseColor = {c.r, c.g, c.b, c.a};
    else if (am->Get(AI_MATKEY_COLOR_DIFFUSE, c) == AI_SUCCESS) m.baseColor = {c.r, c.g, c.b, 1.0f};
    float f;
    if (am->Get(AI_MATKEY_OPACITY, f) == AI_SUCCESS && f < 0.999f && f > 0.0f) { m.baseColor.w = f; m.alphaMode = AlphaMode::Blend; }
    bool hasPbr = false;
    if (am->Get(AI_MATKEY_METALLIC_FACTOR, f) == AI_SUCCESS) { m.metallic = f; hasPbr = true; }
    if (am->Get(AI_MATKEY_ROUGHNESS_FACTOR, f) == AI_SUCCESS) { m.roughness = f; hasPbr = true; }
    if (!hasPbr && am->Get(AI_MATKEY_SHININESS, f) == AI_SUCCESS && f > 0) m.roughness = std::clamp(std::sqrt(2.0f / (f + 2.0f)), 0.0f, 1.0f);
    aiColor3D e;
    if (am->Get(AI_MATKEY_COLOR_EMISSIVE, e) == AI_SUCCESS) m.emissive = {e.r, e.g, e.b};
    if (am->Get(AI_MATKEY_EMISSIVE_INTENSITY, f) == AI_SUCCESS && f > 0) m.emissiveStrength = f;
    int two = 0;
    if (am->Get(AI_MATKEY_TWOSIDED, two) == AI_SUCCESS) m.doubleSided = two != 0;
    auto slot = [&](std::initializer_list<aiTextureType> types, TextureSlot& t) {
      for (auto ty : types) {
        int uv = 0;
        int idx = textureFrom(sc, am, ty, s, dir, texCache, uv);
        if (idx >= 0) { t = {idx, uv}; return true; }
      }
      return false;
    };
    slot({aiTextureType_BASE_COLOR, aiTextureType_DIFFUSE}, m.baseColorTex);
    slot({aiTextureType_NORMALS, aiTextureType_NORMAL_CAMERA}, m.normalTex);
    if (!m.normalTex.valid()) slot({aiTextureType_HEIGHT}, m.normalTex);  // OBJ/3DS "bump" is usually a normal map
    slot({aiTextureType_EMISSIVE, aiTextureType_EMISSION_COLOR}, m.emissiveTex);
    slot({aiTextureType_AMBIENT_OCCLUSION, aiTextureType_LIGHTMAP}, m.occlusionTex);
    slot({aiTextureType_METALNESS}, m.metallicTex);
    slot({aiTextureType_DIFFUSE_ROUGHNESS}, m.roughnessTex);
    if (m.emissiveTex.valid() && m.emissive.x + m.emissive.y + m.emissive.z == 0) m.emissive = {1, 1, 1};
    if (m.baseColorTex.valid() && s.images[m.baseColorTex.image].pixels && s.images[m.baseColorTex.image].pixels->hasAlpha && m.alphaMode == AlphaMode::Opaque)
      m.alphaMode = AlphaMode::Mask;
    s.materials.push_back(m);
  }
  // meshes
  s.meshes.resize(sc->mNumMeshes);
  parallelFor(sc->mNumMeshes, [&](size_t i) {
    const aiMesh* am = sc->mMeshes[i];
    Mesh& m = s.meshes[i];
    m.name = am->mName.C_Str();
    m.material = int(am->mMaterialIndex);
    if (!(am->mPrimitiveTypes & aiPrimitiveType_TRIANGLE)) return;
    const size_t vc = am->mNumVertices;
    m.positions.resize(vc);
    memcpy(m.positions.data(), am->mVertices, vc * 12);
    if (am->mNormals) { m.normals.resize(vc); memcpy(m.normals.data(), am->mNormals, vc * 12); }
    if (am->mTangents && am->mBitangents && am->mNormals) {
      m.tangents.resize(vc);
      for (size_t v = 0; v < vc; ++v) {
        Vec3 t{am->mTangents[v].x, am->mTangents[v].y, am->mTangents[v].z}, b{am->mBitangents[v].x, am->mBitangents[v].y, am->mBitangents[v].z};
        float w = dot(cross(m.normals[v], t), b) < 0 ? -1.0f : 1.0f;
        t = normalize(t);
        m.tangents[v] = {t.x, t.y, t.z, w};
      }
    }
    for (int set = 0; set < 2; ++set) {
      if (!am->HasTextureCoords(set)) continue;
      auto& uv = set == 0 ? m.uv0 : m.uv1;
      uv.resize(vc);
      for (size_t v = 0; v < vc; ++v) uv[v] = {am->mTextureCoords[set][v].x, 1.0f - am->mTextureCoords[set][v].y};
    }
    if (am->HasVertexColors(0)) {
      m.colors.resize(vc);
      for (size_t v = 0; v < vc; ++v) {
        auto q = [](float x) { return uint8_t(std::clamp(x, 0.0f, 1.0f) * 255 + 0.5f); };
        const aiColor4D& c = am->mColors[0][v];
        m.colors[v] = {q(c.r), q(c.g), q(c.b), q(c.a)};
      }
    }
    m.indices.reserve(size_t(am->mNumFaces) * 3);
    for (unsigned f = 0; f < am->mNumFaces; ++f)
      if (am->mFaces[f].mNumIndices == 3)
        for (int k = 0; k < 3; ++k) m.indices.push_back(am->mFaces[f].mIndices[k]);
  }, 1);
  // nodes
  std::function<int(const aiNode*)> walk = [&](const aiNode* an) {
    int id = int(s.nodes.size());
    s.nodes.emplace_back();
    s.nodes[id].name = an->mName.C_Str();
    s.nodes[id].local = toMat(an->mTransformation);
    for (unsigned k = 0; k < an->mNumMeshes; ++k) s.nodes[id].meshes.push_back(int(an->mMeshes[k]));
    for (unsigned k = 0; k < an->mNumChildren; ++k) { int c = walk(an->mChildren[k]); s.nodes[id].children.push_back(c); }
    return id;
  };
  s.roots.push_back(walk(sc->mRootNode));
  // units / up axis metadata (FBX)
  if (sc->mMetaData) {
    double unit = 0;
    int up = 1;
    if (sc->mMetaData->Get("UnitScaleFactor", unit) && unit > 0) s.metersPerUnit = unit * 0.01;
    if (sc->mMetaData->Get("UpAxis", up) && up == 2) s.up = UpAxis::Z;
  }
  if (sc->mNumAnimations) logWarn("%u animation(s) are not carried over", sc->mNumAnimations);
  s.sourceFormat = extOf(path);
  return true;
}

bool writeAssimp(const Scene& s, const std::string& path, const std::string& formatId, const WriteSettings& w, std::string& err) {
  const bool embed = w.embedTextures && (formatId == "fbx" || formatId == "fbxa" || formatId == "assbin");
  std::vector<std::string> rel;
  if (!embed) rel = writeTextureFiles(s, textureDirFor(path), pathToUtf8(pathFromUtf8(path).parent_path()));
  aiScene* sc = new aiScene();
  // embedded textures
  std::vector<std::string> texRef(s.images.size());
  if (embed) {
    std::vector<aiTexture*> texs;
    for (size_t i = 0; i < s.images.size(); ++i) {
      const Image& im = s.images[i];
      if (!im.finalData) continue;
      aiTexture* t = new aiTexture();
      t->mWidth = unsigned(im.finalData->size);
      t->mHeight = 0;
      t->pcData = reinterpret_cast<aiTexel*>(new uint8_t[im.finalData->size]);
      memcpy(t->pcData, im.finalData->data, im.finalData->size);
      std::string ext = im.finalExt.size() > 1 ? im.finalExt.substr(1) : "png";
      if (ext == "jpeg") ext = "jpg";
      strncpy(t->achFormatHint, ext.c_str(), HINTMAXTEXTURELEN - 1);
      std::string fname = sanitizeFileName(im.name.empty() ? "tex" + std::to_string(i) : stemOf(im.name)) + im.finalExt;
      t->mFilename = aiString(fname);
      texRef[i] = "*" + std::to_string(texs.size());
      texs.push_back(t);
    }
    sc->mNumTextures = unsigned(texs.size());
    if (!texs.empty()) { sc->mTextures = new aiTexture*[texs.size()]; std::copy(texs.begin(), texs.end(), sc->mTextures); }
  } else texRef = rel;
  // materials (at least one)
  size_t nm = std::max<size_t>(1, s.materials.size());
  sc->mNumMaterials = unsigned(nm);
  sc->mMaterials = new aiMaterial*[nm];
  for (size_t i = 0; i < nm; ++i) {
    aiMaterial* am = new aiMaterial();
    Material m = i < s.materials.size() ? s.materials[i] : Material{};
    aiString name(m.name.empty() ? "material" + std::to_string(i) : m.name);
    am->AddProperty(&name, AI_MATKEY_NAME);
    aiColor4D base(m.baseColor.x, m.baseColor.y, m.baseColor.z, m.baseColor.w);
    aiColor3D diff(m.baseColor.x, m.baseColor.y, m.baseColor.z), em(m.emissive.x * m.emissiveStrength, m.emissive.y * m.emissiveStrength, m.emissive.z * m.emissiveStrength);
    aiColor3D spec(0.04f, 0.04f, 0.04f), amb(0, 0, 0);
    am->AddProperty(&base, 1, AI_MATKEY_BASE_COLOR);
    am->AddProperty(&diff, 1, AI_MATKEY_COLOR_DIFFUSE);
    am->AddProperty(&spec, 1, AI_MATKEY_COLOR_SPECULAR);
    am->AddProperty(&amb, 1, AI_MATKEY_COLOR_AMBIENT);
    am->AddProperty(&em, 1, AI_MATKEY_COLOR_EMISSIVE);
    float opacity = m.baseColor.w, metal = m.metallic, rough = m.roughness;
    float shin = std::min(1000.0f, 2.0f / std::max(rough * rough, 1e-3f) - 2.0f);
    am->AddProperty(&opacity, 1, AI_MATKEY_OPACITY);
    am->AddProperty(&metal, 1, AI_MATKEY_METALLIC_FACTOR);
    am->AddProperty(&rough, 1, AI_MATKEY_ROUGHNESS_FACTOR);
    am->AddProperty(&shin, 1, AI_MATKEY_SHININESS);
    int two = m.doubleSided ? 1 : 0;
    am->AddProperty(&two, 1, AI_MATKEY_TWOSIDED);
    auto tex = [&](const TextureSlot& t, aiTextureType ty) {
      if (!t.valid() || size_t(t.image) >= texRef.size() || texRef[t.image].empty()) return;
      aiString p(texRef[t.image]);
      am->AddProperty(&p, AI_MATKEY_TEXTURE(ty, 0));
      int uv = t.uvSet;
      am->AddProperty(&uv, 1, AI_MATKEY_UVWSRC(ty, 0));
    };
    tex(m.baseColorTex, aiTextureType_DIFFUSE);
    tex(m.baseColorTex, aiTextureType_BASE_COLOR);
    tex(m.normalTex, aiTextureType_NORMALS);
    tex(m.emissiveTex, aiTextureType_EMISSIVE);
    tex(m.occlusionTex, aiTextureType_AMBIENT_OCCLUSION);
    tex(m.metallicTex, aiTextureType_METALNESS);
    tex(m.roughnessTex, aiTextureType_DIFFUSE_ROUGHNESS);
    if (m.alphaMode != AlphaMode::Opaque) tex(m.baseColorTex, aiTextureType_OPACITY);
    sc->mMaterials[i] = am;
  }
  // meshes (skip empty ones but keep index mapping)
  std::vector<int> meshMap(s.meshes.size(), -1);
  std::vector<aiMesh*> meshes;
  for (size_t i = 0; i < s.meshes.size(); ++i) {
    const Mesh& m = s.meshes[i];
    if (m.indices.empty()) continue;
    const size_t vc = m.vertexCount();
    aiMesh* am = new aiMesh();
    am->mName = aiString(m.name.empty() ? "mesh" + std::to_string(i) : m.name);
    am->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
    am->mMaterialIndex = unsigned(m.material >= 0 && size_t(m.material) < nm ? m.material : 0);
    am->mNumVertices = unsigned(vc);
    am->mVertices = new aiVector3D[vc];
    memcpy(am->mVertices, m.positions.data(), vc * 12);
    if (m.hasNormals()) { am->mNormals = new aiVector3D[vc]; memcpy(am->mNormals, m.normals.data(), vc * 12); }
    if (m.hasTangents() && m.hasNormals()) {
      am->mTangents = new aiVector3D[vc]; am->mBitangents = new aiVector3D[vc];
      for (size_t v = 0; v < vc; ++v) {
        Vec3 t{m.tangents[v].x, m.tangents[v].y, m.tangents[v].z};
        Vec3 b = cross(m.normals[v], t) * m.tangents[v].w;
        am->mTangents[v] = {t.x, t.y, t.z}; am->mBitangents[v] = {b.x, b.y, b.z};
      }
    }
    for (int set = 0; set < 2; ++set) {
      const auto& uv = set == 0 ? m.uv0 : m.uv1;
      if (uv.size() != vc) continue;
      am->mTextureCoords[set] = new aiVector3D[vc];
      am->mNumUVComponents[set] = 2;
      for (size_t v = 0; v < vc; ++v) am->mTextureCoords[set][v] = {uv[v].x, 1.0f - uv[v].y, 0};
    }
    if (m.colors.size() == vc) {
      am->mColors[0] = new aiColor4D[vc];
      for (size_t v = 0; v < vc; ++v) am->mColors[0][v] = {m.colors[v].r / 255.f, m.colors[v].g / 255.f, m.colors[v].b / 255.f, m.colors[v].a / 255.f};
    }
    const size_t tc = m.triangleCount();
    am->mNumFaces = unsigned(tc);
    am->mFaces = new aiFace[tc];
    for (size_t t = 0; t < tc; ++t) {
      am->mFaces[t].mNumIndices = 3;
      am->mFaces[t].mIndices = new unsigned[3]{m.indices[t * 3], m.indices[t * 3 + 1], m.indices[t * 3 + 2]};
    }
    meshMap[i] = int(meshes.size());
    meshes.push_back(am);
  }
  sc->mNumMeshes = unsigned(meshes.size());
  sc->mMeshes = new aiMesh*[std::max<size_t>(1, meshes.size())];
  std::copy(meshes.begin(), meshes.end(), sc->mMeshes);
  // nodes
  std::vector<char> visiting(s.nodes.size(), 0);
  std::function<aiNode*(int, aiNode*)> build = [&](int ni, aiNode* parent) -> aiNode* {
    const Node& n = s.nodes[ni];
    aiNode* an = new aiNode(n.name.empty() ? "node" + std::to_string(ni) : n.name);
    an->mParent = parent;
    an->mTransformation = fromMat(n.local);
    std::vector<unsigned> ms;
    for (int mi : n.meshes) if (mi >= 0 && size_t(mi) < meshMap.size() && meshMap[mi] >= 0) ms.push_back(unsigned(meshMap[mi]));
    if (!ms.empty()) { an->mNumMeshes = unsigned(ms.size()); an->mMeshes = new unsigned[ms.size()]; std::copy(ms.begin(), ms.end(), an->mMeshes); }
    std::vector<aiNode*> kids;
    visiting[ni] = 1;
    for (int c : n.children) if (c >= 0 && size_t(c) < s.nodes.size() && !visiting[c]) kids.push_back(build(c, an));
    if (!kids.empty()) { an->mNumChildren = unsigned(kids.size()); an->mChildren = new aiNode*[kids.size()]; std::copy(kids.begin(), kids.end(), an->mChildren); }
    return an;
  };
  sc->mRootNode = new aiNode("RxTwister");
  std::vector<aiNode*> roots;
  for (int r : s.roots) if (r >= 0 && size_t(r) < s.nodes.size()) roots.push_back(build(r, sc->mRootNode));
  if (!roots.empty()) { sc->mRootNode->mNumChildren = unsigned(roots.size()); sc->mRootNode->mChildren = new aiNode*[roots.size()]; std::copy(roots.begin(), roots.end(), sc->mRootNode->mChildren); }

  Assimp::Exporter ex;
  aiReturn rc = ex.Export(sc, formatId, path, 0);
  if (rc != aiReturn_SUCCESS) err = ex.GetErrorString();
  delete sc;
  return rc == aiReturn_SUCCESS;
}

#else  // !RX_HAVE_ASSIMP
bool assimpAvailable() { return false; }
std::vector<std::pair<std::string, std::string>> assimpImportFormats() { return {}; }
std::vector<std::pair<std::string, std::string>> assimpExportFormats() { return {}; }
bool assimpCanRead(const std::string&) { return false; }
std::string assimpExportId(const std::string&) { return ""; }
bool readAssimp(const std::string&, Scene&, std::string& err) { err = "built without Assimp"; return false; }
bool writeAssimp(const Scene&, const std::string&, const std::string&, const WriteSettings&, std::string& err) { err = "built without Assimp"; return false; }
#endif

}  // namespace rx
