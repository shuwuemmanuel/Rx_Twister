#include "cli_args.h"
#include "rx/rx.h"
#include "core/util.h"
#include <cstdlib>

namespace rx {

const char* usageText() {
  return R"(Rx Twister - 3D model converter & optimiser

USAGE
  rxtwister <input> [output] [options]        convert (default output: <input>_rx.glb, WebP textures)
  rxtwister info <file>                       describe a model
  rxtwister formats                           list every supported format

OUTPUT
  -o, --output <file>         output path; the extension picks the format
  -f, --format <ext>          force output format (glb, gltf, fbx, obj, usdz, blend, dae, ...)
  --preset <name>             web | blender | unity | unreal | godot

TEXTURES
  --embed / --no-embed        embed textures when the format allows it (default on);
                              otherwise they go to <name>_textures/
  --texture-format <f>        auto | webp | png | jpeg | keep  (auto = WebP for glTF, PNG otherwise)
  --quality <0-100>           lossy WebP/JPEG quality (default 85)
  --texture-effort <0-6>      WebP encoder effort (default 2; 4-6 = slower, ~1-3% smaller)
  --max-texture <px>          downscale textures larger than this (e.g. 2048)
  --pot                       force power-of-two texture sizes
  --lossless-normals          lossless WebP for normal maps
  --textures <dir|file>       loose textures to bind to materials (repeatable); names like
                              rock_albedo.png / rock_normal.png / rock_roughness.1001.png
  --no-auto-textures          do not search next to the model for loose textures
  --no-udim                   do not merge UDIM tiles into an atlas
  --atlas                     merge many materials' textures into one atlas (UVs remapped)
  --atlas-size <px>           maximum atlas size (default 8192)

GEOMETRY
  --subdivide <n>             subdivide n times (Loop smooth; UVs preserved)
  --subdiv-scheme <s>         loop | linear
  --unsubdivide <n>           remove ~75% of triangles per level (UV / seam aware)
  --simplify <ratio>          keep this fraction of triangles (0..1)
  --max-triangles <n>         cap the total triangle count
  --remesh                    isotropic remesh that keeps UV seams and UV mapping
  --remesh-faces <n>          target face count for --remesh
  --remesh-edge <len>         target edge length for --remesh
  --remesh-iterations <n>     default 5
  --allow-huge                allow subdivision beyond 250M triangles
  --recompute-normals         rebuild vertex normals
  --smooth-angle <deg>        hard-edge angle for generated normals (default 60)
  --tangents                  write tangents

NORMAL MAPS
  --normal-map [mode]         generate normal maps for materials that lack one:
                              auto (default when the flag is given) | bake | height | off
                              bake   = from the original high-poly surface after reduction
                              height = from base-colour luminance
  --force-normal-map          regenerate even if a normal map exists
  --normal-strength <f>       height mode strength (default 2)
  --normal-size <px>          baked map size (default 2048)

SCENE / OPTIMISATION
  --no-optimize               skip welding and vertex cache optimisation
  --compress meshopt          EXT_meshopt_compression for glTF (smaller, needs a compatible viewer)
  --bake-transforms           apply node transforms to vertices
  --merge-meshes              merge meshes sharing a material (fewer draw calls)
  --up <y|z>                  convert the up axis
  --units <m|cm|mm|in|ft>     convert to these units
  --flip-v                    flip the V texture coordinate

ENGINE
  -j, --threads <n>           worker threads (default: all cores)
  --memory <MB>               memory budget for texture processing
  --blender <path>            Blender executable (for .blend / .abc / .usdc)
  --dry-run                   process but do not write
  -v, --verbose / -q, --quiet
)";
}

std::vector<std::string> splitArgs(const std::string& s) {
  std::vector<std::string> r;
  std::string cur;
  bool q = false, any = false;
  char qc = 0;
  for (char c : s) {
    if (q) { if (c == qc) q = false; else cur += c; continue; }
    if (c == '"' || c == '\'') { q = true; qc = c; any = true; continue; }
    if (c == ' ' || c == '\t' || c == '\n') { if (!cur.empty() || any) r.push_back(cur); cur.clear(); any = false; continue; }
    cur += c;
  }
  if (!cur.empty() || any) r.push_back(cur);
  return r;
}

bool parseArgs(const std::vector<std::string>& a, Options& o, std::vector<std::string>& pos, std::string& err) {
  for (size_t i = 0; i < a.size(); ++i) {
    const std::string& k = a[i];
    auto val = [&]() -> std::string {
      if (i + 1 >= a.size()) { err = "missing value for " + k; return ""; }
      return a[++i];
    };
    auto num = [&](double lo, double hi) -> double {
      std::string v = val();
      if (!err.empty()) return 0;
      char* e = nullptr;
      double d = std::strtod(v.c_str(), &e);
      if (e == v.c_str() || *e || d < lo || d > hi) { err = "bad value '" + v + "' for " + k; return 0; }
      return d;
    };
    if (k == "-o" || k == "--output") o.output = val();
    else if (k == "-f" || k == "--format") o.outFormat = toLower(val());
    else if (k == "--preset") { std::string v = val(); if (!parsePreset(v, o.preset)) err = "unknown preset " + v; }
    else if (k == "--embed") { o.embedTextures = true; o.embedExplicit = true; }
    else if (k == "--no-embed") { o.embedTextures = false; o.embedExplicit = true; }
    else if (k == "--texture-format") {
      std::string v = toLower(val());
      if (v == "auto") o.texFormat = TexFormat::Auto;
      else if (v == "webp") o.texFormat = TexFormat::WebP;
      else if (v == "png") o.texFormat = TexFormat::PNG;
      else if (v == "jpg" || v == "jpeg") o.texFormat = TexFormat::JPEG;
      else if (v == "keep") o.texFormat = TexFormat::Keep;
      else err = "unknown texture format " + v;
    }
    else if (k == "--quality") o.texQuality = int(num(0, 100));
    else if (k == "--texture-effort") o.texEffort = int(num(0, 6));
    else if (k == "--max-texture") o.maxTextureSize = int(num(1, 65536));
    else if (k == "--pot") o.powerOfTwo = true;
    else if (k == "--lossless-normals") o.losslessNormals = true;
    else if (k == "--textures") o.textureSources.push_back(val());
    else if (k == "--no-auto-textures") o.bindTextures = false;
    else if (k == "--no-udim") o.udimAtlas = false;
    else if (k == "--no-pack-orm") o.packOrm = false;
    else if (k == "--atlas") o.atlas = true;
    else if (k == "--atlas-size") o.atlasMaxSize = int(num(64, 32768));
    else if (k == "--subdivide") o.subdivide = int(num(0, 8));
    else if (k == "--subdiv-scheme") {
      std::string v = toLower(val());
      if (v == "loop" || v == "smooth") o.subdivScheme = Subdivision::Loop;
      else if (v == "linear" || v == "simple") o.subdivScheme = Subdivision::Linear;
      else err = "unknown subdivision scheme " + v;
    }
    else if (k == "--unsubdivide") o.unsubdivide = int(num(0, 8));
    else if (k == "--simplify") o.simplifyRatio = float(num(0.0001, 1));
    else if (k == "--max-triangles") o.maxTriangles = size_t(num(4, 4e9));
    else if (k == "--remesh") o.remesh = true;
    else if (k == "--remesh-faces") { o.remesh = true; o.remeshFaces = size_t(num(4, 4e9)); }
    else if (k == "--remesh-edge") { o.remesh = true; o.remeshEdgeLength = float(num(1e-9, 1e9)); }
    else if (k == "--remesh-iterations") o.remeshIterations = int(num(1, 100));
    else if (k == "--no-lock-seams") o.lockSeams = false;
    else if (k == "--allow-huge") o.allowHuge = true;
    else if (k == "--recompute-normals") o.recomputeNormals = true;
    else if (k == "--smooth-angle") o.smoothAngle = float(num(0, 180));
    else if (k == "--tangents") o.generateTangents = true;
    else if (k == "--normal-map") {
      o.normalGen = NormalGen::Auto;
      if (i + 1 < a.size() && a[i + 1].size() && a[i + 1][0] != '-') {
        std::string v = toLower(a[i + 1]);
        if (v == "auto" || v == "bake" || v == "height" || v == "off" || v == "on") {
          ++i;
          o.normalGen = v == "bake" ? NormalGen::Bake : v == "height" ? NormalGen::Height : v == "off" ? NormalGen::Off : NormalGen::Auto;
        }
      }
    }
    else if (k == "--no-normal-map") o.normalGen = NormalGen::Off;
    else if (k == "--force-normal-map") { o.forceNormalMap = true; if (o.normalGen == NormalGen::Off) o.normalGen = NormalGen::Auto; }
    else if (k == "--normal-strength") o.normalStrength = float(num(0, 100));
    else if (k == "--normal-size") o.normalMapSize = int(num(16, 16384));
    else if (k == "--no-optimize") o.optimize = false;
    else if (k == "--compress") {
      std::string v = toLower(val());
      if (v == "meshopt") o.compression = Compression::Meshopt;
      else if (v == "none") o.compression = Compression::None;
      else err = "unknown compression " + v + " (meshopt | none)";
    }
    else if (k == "--bake-transforms") o.bakeTransforms = true;
    else if (k == "--merge-meshes") o.mergeMeshes = true;
    else if (k == "--up") {
      std::string v = toLower(val());
      o.setUp = true;
      if (v == "y") o.targetUp = UpAxis::Y; else if (v == "z") o.targetUp = UpAxis::Z; else err = "--up takes y or z";
    }
    else if (k == "--units") {
      std::string v = toLower(val());
      o.setUnits = true;
      if (v == "m") o.targetMetersPerUnit = 1; else if (v == "cm") o.targetMetersPerUnit = 0.01;
      else if (v == "mm") o.targetMetersPerUnit = 0.001; else if (v == "in") o.targetMetersPerUnit = 0.0254;
      else if (v == "ft") o.targetMetersPerUnit = 0.3048; else err = "unknown unit " + v;
    }
    else if (k == "--flip-v") o.flipV = true;
    else if (k == "-j" || k == "--threads") o.threads = int(num(0, 1024));
    else if (k == "--memory") o.memoryBudgetMB = size_t(num(64, 1e7));
    else if (k == "--blender") o.blenderPath = val();
    else if (k == "--dry-run") o.dryRun = true;
    else if (k == "-v" || k == "--verbose") o.verbose = true;
    else if (k == "-q" || k == "--quiet") o.quiet = true;
    else if (k.size() > 1 && k[0] == '-' && !(k.size() > 1 && std::isdigit((unsigned char)k[1]))) err = "unknown option " + k;
    else pos.push_back(k);
    if (!err.empty()) return false;
  }
  return true;
}

}  // namespace rx
