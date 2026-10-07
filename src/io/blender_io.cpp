// Blender bridge: runs Blender headless (blender -b) to read / write formats that only Blender
// handles well: .blend, Alembic (.abc), USD (.usd/.usdc/.usda/.usdz import) and Blender's own FBX.
// Data goes through a temporary GLB so all of Rx Twister's processing still applies.
#include "io/io.h"
#include "core/util.h"
#include <cstdlib>
#include <cstdio>
#include <random>

namespace rx {

namespace {

const char* kImportScript = R"PY(
import bpy, sys
def need(mod, op, what):
    if op not in dir(mod):
        print("RXERR: " + what + " is not available in this Blender build")
        raise SystemExit(3)
argv = sys.argv[sys.argv.index("--") + 1:]
src, dst = argv[0], argv[1]
ext = src.lower().rsplit('.', 1)[-1]
if ext != 'blend':
    bpy.ops.wm.read_factory_settings(use_empty=True)
    if ext == 'abc': need(bpy.ops.wm, 'alembic_import', 'Alembic import'); bpy.ops.wm.alembic_import(filepath=src)
    elif ext in ('usd', 'usda', 'usdc', 'usdz'): need(bpy.ops.wm, 'usd_import', 'USD import'); bpy.ops.wm.usd_import(filepath=src)
    elif ext == 'fbx': bpy.ops.import_scene.fbx(filepath=src)
    elif ext == 'dae': bpy.ops.wm.collada_import(filepath=src)
    elif ext in ('x3d', 'wrl'): bpy.ops.import_scene.x3d(filepath=src)
    elif ext == 'obj': bpy.ops.wm.obj_import(filepath=src)
    elif ext == 'ply': bpy.ops.wm.ply_import(filepath=src)
    elif ext == 'stl': bpy.ops.wm.stl_import(filepath=src)
    elif ext == 'svg': bpy.ops.import_curve.svg(filepath=src)
    elif ext in ('gltf', 'glb'): bpy.ops.import_scene.gltf(filepath=src)
    else: raise SystemExit("unsupported: " + ext)
# make every object exportable as mesh
for o in bpy.context.scene.objects:
    o.hide_set(False); o.hide_viewport = False
bpy.ops.export_scene.gltf(filepath=dst, export_format='GLB', export_apply=True, export_yup=True,
                          export_animations=False, export_image_format='AUTO')
)PY";

const char* kExportScript = R"PY(
import bpy, sys
def need(mod, op, what):
    if op not in dir(mod):
        print("RXERR: " + what + " is not available in this Blender build")
        raise SystemExit(3)
argv = sys.argv[sys.argv.index("--") + 1:]
src, dst = argv[0], argv[1]
embed = len(argv) > 2 and argv[2] == '1'
ext = dst.lower().rsplit('.', 1)[-1]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=src)
if ext == 'blend':
    if embed:
        bpy.ops.file.pack_all()
    bpy.ops.wm.save_as_mainfile(filepath=dst, compress=True)
elif ext == 'abc':
    need(bpy.ops.wm, 'alembic_export', 'Alembic export')
    bpy.ops.wm.alembic_export(filepath=dst, selected=False, uvs=True, normals=True, face_sets=True)
elif ext in ('usd', 'usdc', 'usda', 'usdz'):
    need(bpy.ops.wm, 'usd_export', 'USD export')
    bpy.ops.wm.usd_export(filepath=dst, export_textures=True, generate_preview_surface=True)
elif ext == 'fbx':
    bpy.ops.export_scene.fbx(filepath=dst, path_mode='COPY', embed_textures=embed, apply_unit_scale=True,
                             axis_forward='-Z', axis_up='Y', mesh_smooth_type='FACE', add_leaf_bones=False)
elif ext == 'dae':
    bpy.ops.wm.collada_export(filepath=dst)
elif ext == 'x3d':
    bpy.ops.export_scene.x3d(filepath=dst)
else:
    raise SystemExit("unsupported: " + ext)
)PY";

std::string quote(const std::string& s) {
#if defined(_WIN32)
  return "\"" + s + "\"";
#else
  std::string r = "'";
  for (char c : s) { if (c == '\'') r += "'\\''"; else r += c; }
  return r + "'";
#endif
}

std::string tempPath(const std::string& ext) {
  std::random_device rd;
  auto dir = fs::temp_directory_path();
  return pathToUtf8(dir / ("rxtwister_" + std::to_string(rd()) + std::to_string(rd()) + ext));
}

bool runBlender(const std::string& blender, const std::string& script, const std::string& blendFile,
                const std::vector<std::string>& args, std::string& err) {
  std::string py = tempPath(".py");
  writeWholeFile(py, reinterpret_cast<const uint8_t*>(script.data()), script.size());
  std::string cmd = quote(blender) + " -b ";
  if (!blendFile.empty()) cmd += quote(blendFile) + " ";
  else cmd += "--factory-startup ";
  cmd += "--python-exit-code 3 --python " + quote(py) + " --";
  for (auto& a : args) cmd += " " + quote(a);
  std::string logFile = tempPath(".log");
#if defined(_WIN32)
  cmd = "\"" + cmd + " > " + quote(logFile) + " 2>&1\"";
#else
  cmd += " > " + quote(logFile) + " 2>&1";
#endif
  logVerbose("running: %s", cmd.c_str());
  int rc = std::system(cmd.c_str());
  std::error_code ec;
  fs::remove(pathFromUtf8(py), ec);
  std::vector<uint8_t> log;
  readWholeFile(logFile, log);
  fs::remove(pathFromUtf8(logFile), ec);
  std::string text(log.begin(), log.end());
  if (logLevel() == LogLevel::Verbose) fprintf(stdout, "%s", text.c_str());
  if (rc != 0) {
    std::string why;
    size_t p = text.find("RXERR: ");
    if (p != std::string::npos) why = text.substr(p + 7, text.find('\n', p) - p - 7);
    else if ((p = text.rfind("Error")) != std::string::npos) why = text.substr(p, text.find('\n', p) - p);
    err = "Blender failed: " + (why.empty() ? "exit " + std::to_string(rc) : why) + " (rerun with --verbose for its log)";
    return false;
  }
  return true;
}

}  // namespace

std::string findBlender(const std::string& hint) {
  std::error_code ec;
  if (!hint.empty() && fs::exists(pathFromUtf8(hint), ec)) return hint;
  if (const char* e = std::getenv("RX_BLENDER")) if (fs::exists(pathFromUtf8(e), ec)) return e;
  if (const char* e = std::getenv("BLENDER_PATH")) if (fs::exists(pathFromUtf8(e), ec)) return e;
#if defined(_WIN32)
  const char* exe = "blender.exe";
  const char sep = ';';
  for (const char* base : {"C:\\Program Files\\Blender Foundation"}) {
    if (!fs::is_directory(base, ec)) continue;
    std::string best;
    for (auto& d : fs::directory_iterator(base, ec)) {
      auto p = d.path() / exe;
      if (fs::exists(p, ec) && pathToUtf8(p) > best) best = pathToUtf8(p);
    }
    if (!best.empty()) return best;
  }
#else
  const char* exe = "blender";
  const char sep = ':';
  for (const char* p : {"/Applications/Blender.app/Contents/MacOS/Blender", "/snap/bin/blender", "/usr/bin/blender", "/usr/local/bin/blender"})
    if (fs::exists(p, ec)) return p;
#endif
  if (const char* path = std::getenv("PATH")) {
    std::string all = path;
    size_t p = 0;
    while (p <= all.size()) {
      size_t e = all.find(sep, p);
      if (e == std::string::npos) e = all.size();
      fs::path c = pathFromUtf8(all.substr(p, e - p)) / exe;
      if (e > p && fs::exists(c, ec)) return pathToUtf8(c);
      p = e + 1;
    }
  }
  return "";
}

bool blenderCanRead(const std::string& e) {
  return e == "blend" || e == "abc" || e == "usd" || e == "usda" || e == "usdc" || e == "usdz" || e == "svg";
}
bool blenderCanWrite(const std::string& e) {
  return e == "blend" || e == "abc" || e == "usd" || e == "usdc";
}

bool readBlender(const std::string& blender, const std::string& path, Scene& s, std::string& err) {
  std::string tmp = tempPath(".glb");
  bool isBlend = extOf(path) == "blend";
  if (!runBlender(blender, kImportScript, isBlend ? path : "", {path, tmp}, err)) return false;
  bool ok = readGltf(tmp, s, err);
  if (ok) {
    // pull everything into memory before the temp file goes away
    for (auto& im : s.images)
      if (im.encoded) im.encoded = Blob::fromVector(std::vector<uint8_t>(im.encoded->data, im.encoded->data + im.encoded->size));
  }
  std::error_code ec;
  fs::remove(pathFromUtf8(tmp), ec);
  s.sourceFormat = extOf(path);
  s.sourcePath = path;
  return ok;
}

bool writeBlender(const std::string& blender, const Scene& s, const std::string& path, const WriteSettings& w, std::string& err) {
  std::string tmp = tempPath(".glb");
  WriteSettings tw = w;
  tw.embedTextures = true;
  if (!writeGltf(s, tmp, tw, true, err)) return false;
  std::string abs = pathToUtf8(fs::absolute(pathFromUtf8(path)));
  bool ok = runBlender(blender, kExportScript, "", {tmp, abs, w.embedTextures ? "1" : "0"}, err);
  std::error_code ec;
  fs::remove(pathFromUtf8(tmp), ec);
  return ok;
}

}  // namespace rx
