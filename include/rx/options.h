// Rx Twister - user facing options for a conversion job.
#pragma once
#include <functional>
#include <string>
#include <vector>
#include "rx/scene.h"

namespace rx {

enum class Subdivision { Loop, Linear };
enum class NormalGen { Off, Auto, Bake, Height };
enum class Compression { None, Meshopt };
enum class Preset { None, Web, Blender, Unity, Unreal, Godot };

struct Options {
  std::string input, output;
  std::string outFormat;            // extension without dot; empty = deduce from `output`
  Preset preset = Preset::None;

  // ---- textures -----------------------------------------------------------
  bool embedTextures = true;        // embed when the target supports it, else folder
  bool embedExplicit = false;       // user passed --embed/--no-embed
  TexFormat texFormat = TexFormat::Auto;   // Auto: WebP for glTF, PNG for everything else
  int texQuality = 85;              // lossy WebP / JPEG quality
  int texEffort = 2;                // WebP encoder effort 0..6 (>=3 is ~10x slower for ~1-3% gain)
  bool losslessNormals = false;     // lossless WebP for normal maps (bigger, exact)
  int maxTextureSize = 0;           // 0 = keep
  bool powerOfTwo = false;
  std::vector<std::string> textureSources;  // files / folders holding loose textures to bind
  bool bindTextures = true;         // auto bind loose textures found next to the model or given
  bool udimAtlas = true;            // merge UDIM tiles into one atlas and remap UVs
  bool packOrm = true;              // pack roughness / metallic into glTF metallicRoughness
  bool atlas = false;               // merge the textures of many materials into one atlas (+UV remap)
  int atlasMaxSize = 8192;
  bool mergeMeshes = false;         // concatenate meshes sharing a material (fewer draw calls)

  // ---- topology -----------------------------------------------------------
  int subdivide = 0;
  Subdivision subdivScheme = Subdivision::Loop;
  int unsubdivide = 0;              // each level removes ~75% of triangles (UV aware)
  float simplifyRatio = 1.0f;       // additional keep-ratio, 1 = off
  size_t maxTriangles = 0;          // hard cap after all operations (0 = none)
  bool remesh = false;
  size_t remeshFaces = 0;           // target face count (0 = keep current density)
  float remeshEdgeLength = 0;       // explicit target edge length (overrides remeshFaces)
  int remeshIterations = 5;
  bool lockSeams = true;
  bool allowHuge = false;           // allow subdivision beyond the safety limit

  // ---- normals ------------------------------------------------------------
  NormalGen normalGen = NormalGen::Off;    // Auto: bake when geometry was reduced, else from albedo
  bool forceNormalMap = false;      // regenerate even when a normal map exists
  float normalStrength = 2.0f;      // for height based generation
  int normalMapSize = 2048;         // baked map resolution
  bool recomputeNormals = false;
  float smoothAngle = 60.0f;        // degrees, for generated vertex normals
  bool generateTangents = false;

  // ---- optimisation / scene ----------------------------------------------
  bool optimize = true;             // weld + vertex cache/fetch ordering + dedupe
  Compression compression = Compression::None;
  bool bakeTransforms = false;
  bool dropEmpty = true;
  UpAxis targetUp = UpAxis::Y;
  bool setUp = false;
  double targetMetersPerUnit = 1.0;
  bool setUnits = false;
  bool flipV = false;

  // ---- engine ------------------------------------------------------------
  int threads = 0;                  // 0 = all cores
  size_t memoryBudgetMB = 0;        // 0 = auto (60% of RAM) for texture processing
  std::string blenderPath;          // optional override
  bool verbose = false;
  bool quiet = false;
  bool dryRun = false;

  std::function<void(const std::string& stage, double fraction)> onProgress;
};

}  // namespace rx
