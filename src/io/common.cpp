#include "io/io.h"
#include "core/util.h"
#include "tex/image_ops.h"
#include <set>

namespace rx {

std::string resolveTexturePath(const std::string& baseDir, std::string raw) {
  if (raw.empty()) return "";
  raw = percentDecode(raw);
  for (auto& c : raw) if (c == '\\') c = '/';
  if (startsWith(raw, "file://")) raw = raw.substr(7);
  std::error_code ec;
  fs::path base = pathFromUtf8(baseDir);
  fs::path p = pathFromUtf8(raw);
  if (p.is_absolute() && fs::exists(p, ec)) return pathToUtf8(p);
  if (fs::exists(base / p, ec)) return pathToUtf8(base / p);
  // try the file name alone, in the model folder and in common texture folders, case-insensitive
  std::string name = toLower(pathToUtf8(p.filename()));
  for (const char* sub : {"", "textures", "Textures", "texture", "tex", "maps", "images"}) {
    fs::path d = *sub ? base / sub : base;
    if (!fs::is_directory(d, ec)) continue;
    for (auto& e : fs::directory_iterator(d, ec))
      if (e.is_regular_file(ec) && toLower(pathToUtf8(e.path().filename())) == name) return pathToUtf8(e.path());
  }
  // same stem with a different extension (e.g. .tif referenced but .png shipped)
  std::string stem = toLower(pathToUtf8(p.stem()));
  for (const char* sub : {"", "textures", "Textures"}) {
    fs::path d = *sub ? base / sub : base;
    if (!fs::is_directory(d, ec)) continue;
    for (auto& e : fs::directory_iterator(d, ec)) {
      if (!e.is_regular_file(ec)) continue;
      std::string ext = extOf(pathToUtf8(e.path()));
      if (toLower(pathToUtf8(e.path().stem())) == stem && (ext == "png" || ext == "jpg" || ext == "jpeg" || ext == "webp" || ext == "tga"))
        return pathToUtf8(e.path());
    }
  }
  return "";
}

int loadTextureFile(Scene& s, const std::string& path) {
  for (size_t i = 0; i < s.images.size(); ++i) if (s.images[i].sourcePath == path) return int(i);
  std::string err;
  auto b = mapFile(path, &err);
  if (!b) { logWarn("texture: %s", err.c_str()); return -1; }
  Image im;
  im.name = stemOf(path);
  im.sourcePath = path;
  im.encoded = b;
  im.mime = sniffMime(b->data, b->size);
  if (im.mime.empty()) {  // formats stb can still decode (tga, psd ...): decode now so they can be re-encoded
    std::string e2;
    im.pixels = decodeImage(b->data, b->size, &e2);
    if (!im.pixels) { logWarn("texture %s: unsupported image (%s)", path.c_str(), e2.c_str()); return -1; }
    im.encoded.reset();
  }
  return s.addImage(std::move(im));
}

std::string textureDirFor(const std::string& outPath) {
  fs::path p = pathFromUtf8(outPath);
  return pathToUtf8(p.parent_path() / pathFromUtf8(pathToUtf8(p.stem()) + "_textures"));
}

std::vector<std::string> writeTextureFiles(const Scene& s, const std::string& dir, const std::string& relBase) {
  std::vector<std::string> rel(s.images.size());
  if (s.images.empty()) return rel;
  std::error_code ec;
  fs::create_directories(pathFromUtf8(dir), ec);
  std::set<std::string> used;
  fs::path rb = pathFromUtf8(relBase);
  for (size_t i = 0; i < s.images.size(); ++i) {
    const Image& im = s.images[i];
    if (!im.finalData) continue;
    std::string base = sanitizeFileName(im.name.empty() ? "texture" + std::to_string(i) : stemOf(im.name));
    std::string name = base + im.finalExt;
    for (int k = 1; used.count(toLower(name)); ++k) name = base + "_" + std::to_string(k) + im.finalExt;
    used.insert(toLower(name));
    fs::path full = pathFromUtf8(dir) / pathFromUtf8(name);
    if (!writeWholeFile(pathToUtf8(full), im.finalData->data, im.finalData->size)) logWarn("cannot write %s", pathToUtf8(full).c_str());
    rel[i] = pathToUtf8(fs::relative(full, rb, ec));
    if (ec || rel[i].empty()) rel[i] = pathToUtf8(pathFromUtf8(dir).filename() / pathFromUtf8(name));
    for (auto& c : rel[i]) if (c == '\\') c = '/';
  }
  return rel;
}

}  // namespace rx
