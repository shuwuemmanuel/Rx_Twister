// Texture stage: usage tagging, ORM packing, generation, binding, atlasing and final encoding.
#pragma once
#include "rx/options.h"
#include "rx/scene.h"

namespace rx {

void markImageUsage(Scene& s);
std::shared_ptr<Pixels> imagePixels(Image& im, std::string* err = nullptr);   // decodes on demand
bool imageSize(const Image& im, int& w, int& h);

// Separate roughness / metallic (/ occlusion) -> glTF metallicRoughness (+ occlusion in R).
void packMetalRough(Scene& s);
// Packed metallicRoughness -> separate grey roughness / metallic maps (OBJ / FBX ...).
void unpackMetalRough(Scene& s);

// Loose texture files (folders / files / next to the model) -> material slots, UDIM -> atlas.
void bindLooseTextures(Scene& s, const Options& o);
// Merge textures of several materials into one atlas per channel, remapping UVs.
void atlasMaterials(Scene& s, const Options& o);

// Normal map from base colour luminance for materials without one.
void generateHeightNormals(Scene& s, const Options& o);
// Bake a tangent space normal map from `reference` (high detail, world space) onto `s` (flattened).
void bakeNormalMaps(Scene& s, const Scene& reference, const Options& o);
std::shared_ptr<Pixels> heightToNormal(const Pixels& height, float strength, int maxSize);

struct TextureEncodeSettings {
  TexFormat format = TexFormat::WebP;
  int quality = 85;
  int effort = 2;
  bool losslessNormals = false;
  int maxSize = 0;
  bool powerOfTwo = false;
  size_t memoryBudget = 0;
};
// Resizes / transcodes every image into Image::finalData (pass-through when nothing changes).
bool finalizeImages(Scene& s, const TextureEncodeSettings& t);

}  // namespace rx
