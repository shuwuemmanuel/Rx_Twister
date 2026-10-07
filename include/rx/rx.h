// Rx Twister - 3D model converter & optimiser. Public C++ API.
#pragma once
#include <string>
#include <vector>
#include "rx/options.h"
#include "rx/scene.h"

namespace rx {

const char* version();

struct FormatInfo {
  std::string ext;          // without dot
  std::string description;
  std::string backend;      // "native", "assimp", "blender"
  bool canRead = false, canWrite = false;
  bool embedsTextures = false;
};
std::vector<FormatInfo> supportedFormats(const std::string& blenderHint = "");

struct Result {
  bool ok = false;
  std::string error;
  std::string output;
  size_t trianglesIn = 0, trianglesOut = 0, verticesIn = 0, verticesOut = 0;
  size_t meshes = 0, materials = 0, images = 0;
  uint64_t bytesIn = 0, bytesOut = 0;
  double seconds = 0;
};

// One call does everything: load -> bind textures -> topology -> normals -> textures -> save.
Result convert(const Options& opts);

// Building blocks for embedding (GUI, plugins, batch tools).
bool loadScene(const std::string& path, Scene& scene, const Options& opts, std::string& err);
bool processScene(Scene& scene, const Options& opts, const std::string& outFormat, std::string& err);
bool saveScene(Scene& scene, const std::string& path, const Options& opts, std::string& err);
std::string describeScene(const Scene& scene);

// Applies a preset (unity, unreal, ...) to fields the user did not set explicitly.
void applyPreset(Options& o);
bool parsePreset(const std::string& name, Preset& p);

}  // namespace rx
