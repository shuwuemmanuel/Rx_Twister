// Importers / exporters.
#pragma once
#include <string>
#include <vector>
#include "rx/options.h"
#include "rx/scene.h"

namespace rx {

struct WriteSettings {
  const Options* opts = nullptr;
  bool embedTextures = true;
};

// native
bool readGltf(const std::string& path, Scene& s, std::string& err);
bool writeGltf(const Scene& s, const std::string& path, const WriteSettings& w, bool binary, std::string& err);
bool readObj(const std::string& path, Scene& s, std::string& err);
bool writeObj(const Scene& s, const std::string& path, const WriteSettings& w, std::string& err);
bool readStl(const std::string& path, Scene& s, std::string& err);
bool writeStl(const Scene& s, const std::string& path, const WriteSettings& w, std::string& err);
bool readPly(const std::string& path, Scene& s, std::string& err);
bool writePly(const Scene& s, const std::string& path, const WriteSettings& w, std::string& err);
bool writeUsd(const Scene& s, const std::string& path, const WriteSettings& w, bool zip, std::string& err);

// Assimp (40+ import formats, ~15 export formats)
bool assimpAvailable();
bool assimpCanRead(const std::string& ext);
std::string assimpExportId(const std::string& ext);   // "" when unsupported
bool readAssimp(const std::string& path, Scene& s, std::string& err);
bool writeAssimp(const Scene& s, const std::string& path, const std::string& formatId, const WriteSettings& w, std::string& err);
std::vector<std::pair<std::string, std::string>> assimpImportFormats();   // ext, description
std::vector<std::pair<std::string, std::string>> assimpExportFormats();

// Blender (headless, optional at runtime)
std::string findBlender(const std::string& hint);
bool blenderCanRead(const std::string& ext);
bool blenderCanWrite(const std::string& ext);
bool readBlender(const std::string& blender, const std::string& path, Scene& s, std::string& err);
bool writeBlender(const std::string& blender, const Scene& s, const std::string& path, const WriteSettings& w, std::string& err);

// shared helpers
std::string resolveTexturePath(const std::string& baseDir, std::string raw);
int loadTextureFile(Scene& s, const std::string& path);   // dedupes by path; -1 on failure
// Writes Image::finalData files into `dir` and returns the path of each image relative to `relBase`.
std::vector<std::string> writeTextureFiles(const Scene& s, const std::string& dir, const std::string& relBase);
std::string textureDirFor(const std::string& outPath);   // "<dir>/<stem>_textures"

}  // namespace rx
