#include "rx/rx.h"
#include "core/par.h"
#include "core/util.h"
#include "geom/mesh_ops.h"
#include "geom/topology.h"
#include "io/io.h"
#include "tex/textures.h"
#include <atomic>
#include <cstring>
#include <map>
#include <sstream>

namespace rx {

const char* version() { return "1.0.0"; }

namespace {

enum class Backend { Native, Assimp, Blender, None };

bool nativeWrite(const std::string& e) {
  return e == "glb" || e == "gltf" || e == "obj" || e == "stl" || e == "ply" || e == "usda" || e == "usdz" || e == "usd";
}
// Formats where we always prefer our writer / reader over a third-party one.
bool formatEmbeds(const std::string& e) { return e == "glb" || e == "gltf" || e == "usdz" || e == "fbx" || e == "blend"; }
bool formatTakesWebP(const std::string& e) { return e == "glb" || e == "gltf"; }
bool formatPacksMR(const std::string& e) { return e == "glb" || e == "gltf" || e == "usda" || e == "usdz" || e == "usd" || e == "blend" || e == "abc" || e == "usdc"; }

void progress(const Options& o, const char* stage, double f) {
  if (o.onProgress) o.onProgress(stage, f);
}

void mergeByMaterial(Scene& s) {
  std::map<int, std::vector<int>> groups;
  for (size_t i = 0; i < s.meshes.size(); ++i) groups[s.meshes[i].material].push_back(int(i));
  std::vector<Mesh> out;
  for (auto& [mat, list] : groups) {
    if (list.size() == 1) { out.push_back(std::move(s.meshes[list[0]])); continue; }
    bool n = true, t = true, t1 = true, c = false, tg = true;
    for (int i : list) { auto& m = s.meshes[i]; n &= m.hasNormals(); t &= m.hasUV(); t1 &= m.uv1.size() == m.positions.size(); c |= !m.colors.empty(); tg &= m.hasTangents(); }
    Mesh r;
    r.name = s.meshes[list[0]].name;
    r.material = mat;
    for (int i : list) {
      Mesh& m = s.meshes[i];
      if (r.positions.size() + m.positions.size() > 0xFFFFFFF0ull) { out.push_back(std::move(r)); r = Mesh{}; r.material = mat; r.name = m.name; }
      uint32_t base = uint32_t(r.positions.size());
      r.positions.insert(r.positions.end(), m.positions.begin(), m.positions.end());
      if (n) r.normals.insert(r.normals.end(), m.normals.begin(), m.normals.end());
      if (tg) r.tangents.insert(r.tangents.end(), m.tangents.begin(), m.tangents.end());
      if (t) r.uv0.insert(r.uv0.end(), m.uv0.begin(), m.uv0.end());
      if (t1) r.uv1.insert(r.uv1.end(), m.uv1.begin(), m.uv1.end());
      if (c) { if (m.colors.size() != m.positions.size()) m.colors.assign(m.positions.size(), {255, 255, 255, 255}); r.colors.insert(r.colors.end(), m.colors.begin(), m.colors.end()); }
      for (uint32_t k : m.indices) r.indices.push_back(base + k);
      m = Mesh{};
    }
    out.push_back(std::move(r));
  }
  s.meshes.swap(out);
  s.nodes.clear(); s.roots.clear();
  for (size_t i = 0; i < s.meshes.size(); ++i) {
    Node nd; nd.name = s.meshes[i].name; nd.meshes = {int(i)};
    s.nodes.push_back(nd); s.roots.push_back(int(i));
  }
}

}  // namespace

bool parsePreset(const std::string& n0, Preset& p) {
  std::string n = toLower(n0);
  if (n == "none") p = Preset::None;
  else if (n == "web" || n == "three" || n == "babylon") p = Preset::Web;
  else if (n == "blender") p = Preset::Blender;
  else if (n == "unity") p = Preset::Unity;
  else if (n == "unreal" || n == "ue" || n == "ue5") p = Preset::Unreal;
  else if (n == "godot") p = Preset::Godot;
  else return false;
  return true;
}

void applyPreset(Options& o) {
  auto setFmt = [&](const char* f) { if (o.outFormat.empty() && extOf(o.output).empty()) o.outFormat = f; };
  switch (o.preset) {
    case Preset::Web:
      setFmt("glb");
      if (o.texFormat == TexFormat::Auto) o.texFormat = TexFormat::WebP;
      if (!o.maxTextureSize) o.maxTextureSize = 2048;
      break;
    case Preset::Blender: setFmt("glb"); break;
    case Preset::Godot: setFmt("glb"); break;
    case Preset::Unity:
      setFmt("fbx");
      if (o.texFormat == TexFormat::Auto) o.texFormat = TexFormat::PNG;
      o.generateTangents = true;
      break;
    case Preset::Unreal:
      setFmt("fbx");
      if (o.texFormat == TexFormat::Auto) o.texFormat = TexFormat::PNG;
      o.generateTangents = true;
      if (!o.maxTextureSize) o.maxTextureSize = 8192;
      break;
    case Preset::None: break;
  }
}

std::vector<FormatInfo> supportedFormats(const std::string& blenderHint) {
  std::map<std::string, FormatInfo> m;
  auto add = [&](const std::string& ext, const std::string& desc, const std::string& be, bool r, bool w) {
    FormatInfo& f = m[ext];
    if (f.ext.empty()) { f.ext = ext; f.description = desc; f.backend = be; }
    else if (f.backend.find(be) == std::string::npos) f.backend += "+" + be;
    f.canRead |= r; f.canWrite |= w;
    f.embedsTextures = formatEmbeds(ext);
  };
  add("glb", "glTF 2.0 binary (WebP textures, default output)", "native", true, true);
  add("gltf", "glTF 2.0 JSON + .bin", "native", true, true);
  add("obj", "Wavefront OBJ + MTL", "native", true, true);
  add("stl", "Stereolithography", "native", true, true);
  add("ply", "Stanford PLY", "native", true, true);
  add("usda", "Universal Scene Description (text)", "native", false, true);
  add("usd", "Universal Scene Description", "native", false, true);
  add("usdz", "USD zip package (Apple AR, Unity, Unreal)", "native", false, true);
  for (auto& [ext, desc] : assimpImportFormats()) add(ext, desc, "assimp", true, false);
  {
    for (auto& [id, desc] : assimpExportFormats()) {
      auto p = desc.rfind("(.");
      std::string ext = p == std::string::npos ? id : desc.substr(p + 2, desc.size() - p - 3);
      if (ext == "gltf" || ext == "glb" || ext == "obj" || ext == "stl" || ext == "ply") continue;  // native is better
      add(ext, desc.substr(0, p == std::string::npos ? desc.size() : p), "assimp", false, true);
    }
  }
  if (!findBlender(blenderHint).empty()) {
    for (const char* e : {"blend", "abc", "usd", "usda", "usdc", "usdz"}) add(e, "", "blender", blenderCanRead(e), blenderCanWrite(e));
    m["blend"].description = "Blender scene";
    m["abc"].description = "Alembic";
    m["usdc"].description = "USD crate (binary)";
  }
  std::vector<FormatInfo> r;
  for (auto& [k, v] : m) r.push_back(v);
  return r;
}

bool loadScene(const std::string& path, Scene& s, const Options& o, std::string& err) {
  std::string ext = extOf(path);
  std::error_code ec;
  if (!fs::exists(pathFromUtf8(path), ec)) { err = "input not found: " + path; return false; }
  bool ok = false;
  std::string blender = blenderCanRead(ext) ? findBlender(o.blenderPath) : "";
  if (ext == "glb" || ext == "gltf" || ext == "vrm") ok = readGltf(path, s, err);
  else if (ext == "obj") ok = readObj(path, s, err);
  else if (ext == "stl") ok = readStl(path, s, err);
  else if (ext == "ply") ok = readPly(path, s, err);
  else if (blenderCanRead(ext) && !blender.empty()) ok = readBlender(blender, path, s, err);
  else if (assimpCanRead(ext)) ok = readAssimp(path, s, err);
  else if (blenderCanRead(ext)) { err = "." + ext + " needs Blender (install it or pass --blender <path>)"; return false; }
  else { err = "unsupported input format: ." + ext; return false; }
  if (!ok && err.empty()) err = "failed to read " + path;
  if (!ok && assimpCanRead(ext) && ext != "obj" && ext != "gltf" && ext != "glb") return false;
  if (!ok && (ext == "obj" || ext == "stl" || ext == "ply") && assimpCanRead(ext)) {  // last resort
    logWarn("native reader failed (%s); retrying with Assimp", err.c_str());
    s = Scene{};
    err.clear();
    ok = readAssimp(path, s, err);
  }
  if (ok) s.sourcePath = path;
  return ok;
}

bool processScene(Scene& s, const Options& o, const std::string& outExt, std::string& err) {
  Timer tm;
  // 1. loose textures / UDIM
  progress(o, "textures:bind", 0);
  bindLooseTextures(s, o);

  // 2. coordinate system / units
  if (outExt == "fbx" && !o.setUnits) convertAxisAndUnits(s, o.setUp ? o.targetUp : UpAxis::Y, 0.01);  // FBX convention: cm, Y up
  else if (outExt == "3ds" && !o.setUp) convertAxisAndUnits(s, UpAxis::Z, o.setUnits ? o.targetMetersPerUnit : s.metersPerUnit);  // 3DS is Z up
  else if (o.setUp || o.setUnits) convertAxisAndUnits(s, o.setUp ? o.targetUp : s.up, o.setUnits ? o.targetMetersPerUnit : s.metersPerUnit);

  const bool reduces = o.unsubdivide > 0 || o.simplifyRatio < 1.0f || o.remesh || o.maxTriangles > 0;
  NormalGen ng = o.normalGen;
  if (ng == NormalGen::Auto) ng = reduces ? NormalGen::Bake : NormalGen::Height;
  if (ng == NormalGen::Bake && !reduces) {
    logWarn("normal bake needs a reduced mesh (unsubdivide / simplify / remesh); using --normal-map height instead");
    ng = NormalGen::Height;
  }
  const bool flatten = o.bakeTransforms || ng == NormalGen::Bake || o.mergeMeshes || outExt == "stl" || outExt == "ply";
  if (flatten && s.hasTransforms()) flattenScene(s);

  // 3. mesh cleanup
  progress(o, "geometry:weld", 0.1);
  if (o.flipV) for (auto& m : s.meshes) { for (auto& t : m.uv0) t.y = 1 - t.y; for (auto& t : m.uv1) t.y = 1 - t.y; }
  if (o.optimize) parallelFor(s.meshes.size(), [&](size_t i) { weldMesh(s.meshes[i]); removeDegenerateTriangles(s.meshes[i]); }, 1);
  logVerbose("weld: %.2fs", tm.seconds());

  Scene reference;
  if (ng == NormalGen::Bake) {
    for (auto& m : s.meshes) {
      Mesh r;
      r.positions = m.positions; r.normals = m.normals; r.indices = m.indices;
      reference.meshes.push_back(std::move(r));
    }
  }

  // 4. topology
  size_t tris = s.totalTriangles();
  if (o.subdivide > 0) {
    double est = double(tris) * std::pow(4.0, o.subdivide);
    if (est > 250e6 && !o.allowHuge) {
      err = "subdivision would create ~" + std::to_string(size_t(est / 1e6)) + "M triangles; pass --allow-huge to force";
      return false;
    }
  }
  progress(o, "geometry:topology", 0.2);
  std::atomic<size_t> done{0};
  const size_t nMeshes = s.meshes.size();
  parallelFor(nMeshes, [&](size_t i) {
    Mesh& m = s.meshes[i];
    if (m.indices.empty()) return;
    if (o.unsubdivide > 0) simplifyMesh(m, float(std::pow(0.25, o.unsubdivide)), o.lockSeams);
    if (o.simplifyRatio < 1.0f) simplifyMesh(m, o.simplifyRatio, o.lockSeams);
    if (o.remesh) {
      RemeshParams rp;
      rp.iterations = o.remeshIterations;
      rp.targetEdge = o.remeshEdgeLength;
      if (o.remeshFaces && tris) rp.targetFaces = std::max<size_t>(4, size_t(double(o.remeshFaces) * double(m.triangleCount()) / double(tris)));
      remeshMesh(m, rp, o.smoothAngle);
    }
    if (o.subdivide > 0) subdivideMesh(m, o.subdivide, o.subdivScheme);
    progress(o, "geometry:topology", 0.2 + 0.3 * double(++done) / double(std::max<size_t>(1, nMeshes)));
  }, 1);
  if (o.maxTriangles > 0 && s.totalTriangles() > o.maxTriangles) {
    float r = float(double(o.maxTriangles) / double(s.totalTriangles()));
    parallelFor(s.meshes.size(), [&](size_t i) { simplifyMesh(s.meshes[i], r, o.lockSeams); }, 1);
  }
  if (o.unsubdivide || o.simplifyRatio < 1 || o.remesh || o.subdivide || o.maxTriangles)
    logInfo("geometry: %zu -> %zu triangles", tris, s.totalTriangles());

  // 5. normals
  progress(o, "geometry:normals", 0.5);
  logVerbose("topology done: %.2fs", tm.seconds());
  parallelFor(s.meshes.size(), [&](size_t i) {
    Mesh& m = s.meshes[i];
    if (m.indices.empty()) return;
    if (o.recomputeNormals || !m.hasNormals()) computeNormals(m, o.smoothAngle);
    else for (auto& n : m.normals) n = normalize(n);
  }, 1);

  // 6. materials: atlas / merge
  if (o.atlas) atlasMaterials(s, o);
  if (o.mergeMeshes) mergeByMaterial(s);

  // 7. normal maps
  progress(o, "textures:normals", 0.6);
  if (ng != NormalGen::Off) {
    int def = -1;
    for (auto& m : s.meshes)
      if (m.material < 0 && m.hasUV()) {
        if (def < 0) { Material dm; dm.name = "default"; def = s.addMaterial(std::move(dm)); }
        m.material = def;
      }
  }
  if (ng == NormalGen::Bake) bakeNormalMaps(s, reference, o);
  else if (ng == NormalGen::Height) generateHeightNormals(s, o);
  reference = Scene{};

  // 8. tangents
  bool needTangents = o.generateTangents;
  if (needTangents)
    parallelFor(s.meshes.size(), [&](size_t i) { Mesh& m = s.meshes[i]; if (m.hasUV() && !m.hasTangents()) computeTangents(m); }, 1);

  // 9. ORM layout for the target
  Timer tp;
  if (formatPacksMR(outExt) && o.packOrm) packMetalRough(s);
  else if (!formatPacksMR(outExt)) unpackMetalRough(s);
  logVerbose("ORM layout: %.2fs", tp.seconds());

  // 10. vertex cache / fetch optimisation
  progress(o, "geometry:optimize", 0.7);
  Timer to;
  if (o.optimize) parallelFor(s.meshes.size(), [&](size_t i) { optimizeMesh(s.meshes[i]); }, 1);
  logVerbose("vertex cache optimisation: %.2fs", to.seconds());
  if (o.dropEmpty) {
    for (auto& n : s.nodes) {
      std::vector<int> keep;
      for (int mi : n.meshes) if (mi >= 0 && size_t(mi) < s.meshes.size() && !s.meshes[mi].indices.empty()) keep.push_back(mi);
      n.meshes.swap(keep);
    }
  }
  dropUnusedResources(s);
  markImageUsage(s);
  logVerbose("processing took %.2fs", tm.seconds());
  return true;
}

bool saveScene(Scene& s, const std::string& path, const Options& o, std::string& err) {
  std::string ext = o.outFormat.empty() ? extOf(path) : toLower(o.outFormat);
  // texture encoding
  TextureEncodeSettings t;
  t.format = o.texFormat;
  if (t.format == TexFormat::Auto) t.format = formatTakesWebP(ext) ? TexFormat::WebP : TexFormat::PNG;
  if (t.format == TexFormat::WebP && !formatTakesWebP(ext) && o.texFormat == TexFormat::WebP)
    logWarn("WebP textures in .%s are only readable by some tools (e.g. Blender)", ext.c_str());
  std::string blender;
  const bool viaBlender = blenderCanWrite(ext) && !nativeWrite(ext) && !(blender = findBlender(o.blenderPath)).empty();
  if (viaBlender && t.format == TexFormat::WebP) t.format = TexFormat::PNG;
  if ((ext == "usdz" || ext == "usda" || ext == "usd") && t.format == TexFormat::WebP) t.format = TexFormat::PNG;
  t.quality = o.texQuality;
  t.effort = o.texEffort;
  t.losslessNormals = o.losslessNormals;
  t.maxSize = o.maxTextureSize;
  t.powerOfTwo = o.powerOfTwo;
  t.memoryBudget = o.memoryBudgetMB << 20;
  progress(o, "textures:encode", 0.75);
  Timer tt;
  finalizeImages(s, t);
  logVerbose("texture encoding took %.2fs", tt.seconds());
  Timer tw;

  {
    std::error_code ec;
    fs::path parent = pathFromUtf8(path).parent_path();
    if (!parent.empty()) fs::create_directories(parent, ec);
  }
  WriteSettings w;
  w.opts = &o;
  w.embedTextures = o.embedTextures;
  if (o.embedTextures && !formatEmbeds(ext) && !s.images.empty())
    logInfo("textures: .%s cannot embed textures - writing them to %s/", ext.c_str(), pathToUtf8(pathFromUtf8(textureDirFor(path)).filename()).c_str());
  progress(o, "write", 0.9);
  bool ok;
  if (ext == "glb") ok = writeGltf(s, path, w, true, err);
  else if (ext == "gltf") ok = writeGltf(s, path, w, false, err);
  else if (ext == "obj") ok = writeObj(s, path, w, err);
  else if (ext == "stl") ok = writeStl(s, path, w, err);
  else if (ext == "ply") ok = writePly(s, path, w, err);
  else if (ext == "usda" || ext == "usd") ok = writeUsd(s, path, w, false, err);
  else if (ext == "usdz") ok = writeUsd(s, path, w, true, err);
  else if (viaBlender) ok = writeBlender(blender, s, path, w, err);
  else if (blenderCanWrite(ext)) { err = "." + ext + " output needs Blender (install it or pass --blender <path>)"; ok = false; }
  else {
    std::string id = assimpExportId(ext);
    if (id.empty()) { err = "unsupported output format: ." + ext; return false; }
    ok = writeAssimp(s, path, id, w, err);
  }
  progress(o, "done", 1.0);
  logVerbose("writing took %.2fs", tw.seconds());
  return ok;
}

std::string describeScene(const Scene& s) {
  std::ostringstream o;
  Aabb b = s.bounds();
  o << "format     : " << s.sourceFormat << "\n";
  o << "meshes     : " << s.meshes.size() << "  (nodes " << s.nodes.size() << ")\n";
  o << "triangles  : " << s.totalTriangles() << "\n";
  o << "vertices   : " << s.totalVertices() << "\n";
  if (b.valid()) {
    Vec3 sz = b.size();
    char buf[200];
    snprintf(buf, sizeof buf, "size       : %.4g x %.4g x %.4g (units, %g m/unit, %s-up)\n", sz.x, sz.y, sz.z, s.metersPerUnit, s.up == UpAxis::Z ? "Z" : "Y");
    o << buf;
  }
  size_t uv = 0, nrm = 0, col = 0;
  for (auto& m : s.meshes) { uv += m.hasUV(); nrm += m.hasNormals(); col += !m.colors.empty(); }
  o << "attributes : " << nrm << " with normals, " << uv << " with UVs, " << col << " with vertex colours\n";
  o << "materials  : " << s.materials.size() << "\n";
  for (size_t i = 0; i < s.materials.size() && i < 32; ++i) {
    const Material& m = s.materials[i];
    o << "  - " << (m.name.empty() ? "(unnamed)" : m.name) << ":";
    if (m.baseColorTex.valid()) o << " base";
    if (m.normalTex.valid()) o << " normal";
    if (m.metalRoughTex.valid()) o << " metalRough";
    if (m.roughnessTex.valid()) o << " rough";
    if (m.metallicTex.valid()) o << " metal";
    if (m.occlusionTex.valid()) o << " ao";
    if (m.emissiveTex.valid()) o << " emissive";
    o << "\n";
  }
  o << "images     : " << s.images.size() << "\n";
  for (size_t i = 0; i < s.images.size() && i < 32; ++i) {
    const Image& im = s.images[i];
    int w = 0, h = 0;
    imageSize(im, w, h);
    o << "  - " << im.name << "  " << w << "x" << h << "  " << (im.mime.empty() ? "decoded" : im.mime);
    if (im.encoded) o << "  " << humanBytes(double(im.encoded->size));
    if (!im.sourcePath.empty()) o << "  (" << im.sourcePath << ")";
    o << "\n";
  }
  return o.str();
}

Result convert(const Options& in) {
  Options o = in;
  Result r;
  Timer tm;
  setLogLevel(o.quiet ? LogLevel::Quiet : o.verbose ? LogLevel::Verbose : LogLevel::Info);
  setThreadLimit(o.threads);
  applyPreset(o);
  if (o.output.empty()) {
    fs::path p = pathFromUtf8(o.input);
    o.output = pathToUtf8(p.parent_path() / pathFromUtf8(pathToUtf8(p.stem()) + "_rx." + (o.outFormat.empty() ? "glb" : o.outFormat)));
  } else if (extOf(o.output).empty()) {
    o.output += "." + (o.outFormat.empty() ? std::string("glb") : o.outFormat);
  }
  std::string ext = o.outFormat.empty() ? extOf(o.output) : toLower(o.outFormat);
  std::error_code ec;
  r.bytesIn = fs::file_size(pathFromUtf8(o.input), ec);
  Scene s;
  progress(o, "load", 0);
  Timer tl;
  if (!loadScene(o.input, s, o, r.error)) return r;
  r.trianglesIn = s.totalTriangles();
  r.verticesIn = s.totalVertices();
  logInfo("loaded %s: %zu meshes, %zu triangles, %zu materials, %zu images (%.2fs)", pathToUtf8(pathFromUtf8(o.input).filename()).c_str(),
          s.meshes.size(), r.trianglesIn, s.materials.size(), s.images.size(), tl.seconds());
  if (!processScene(s, o, ext, r.error)) return r;
  r.trianglesOut = s.totalTriangles();
  r.verticesOut = s.totalVertices();
  r.meshes = s.meshes.size();
  r.materials = s.materials.size();
  r.images = s.images.size();
  if (o.dryRun) { r.ok = true; r.output = o.output; r.seconds = tm.seconds(); return r; }
  if (!saveScene(s, o.output, o, r.error)) return r;
  r.output = o.output;
  if (ext == "glb" && !fs::exists(pathFromUtf8(o.output), ec)) r.output = pathToUtf8(pathFromUtf8(o.output).replace_extension(".gltf"));
  r.bytesOut = fs::file_size(pathFromUtf8(r.output), ec);
  r.ok = true;
  r.seconds = tm.seconds();
  return r;
}

}  // namespace rx
