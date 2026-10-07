// Rx Twister - in-memory scene model shared by every importer, processor and exporter.
// Conventions: UVs use the glTF convention (origin top-left, v down). Matrices are column-major.
#pragma once
#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include "rx/math.h"

namespace rx {

// Read-only byte range with an owner that keeps the memory alive (mmap, vector, ...).
struct Blob {
  const uint8_t* data = nullptr;
  size_t size = 0;
  std::shared_ptr<void> owner;
  static std::shared_ptr<Blob> fromVector(std::vector<uint8_t>&& v);
};

struct Pixels {  // 8-bit RGBA, row-major, top row first
  int width = 0, height = 0;
  std::vector<uint8_t> rgba;
  bool hasAlpha = false;
  size_t bytes() const { return rgba.size(); }
};

enum ImageUsage : uint32_t {
  kUseColor = 1,      // sRGB colour data (base colour, emissive)
  kUseNormal = 2,     // tangent space normal map
  kUseData = 4,       // linear data (roughness, metal, occlusion)
};

enum class TexFormat { Auto, Keep, WebP, PNG, JPEG };

struct Image {
  std::string name;
  std::string mime;                    // mime of `encoded`
  std::shared_ptr<Blob> encoded;       // original file bytes (may be null if only `pixels` exists)
  std::shared_ptr<Pixels> pixels;      // decoded pixels (generated or loaded on demand)
  std::string sourcePath;              // file it came from, if any
  uint32_t usage = 0;

  // Result of the texture stage - what exporters actually write.
  std::shared_ptr<Blob> finalData;
  std::string finalMime;
  std::string finalExt;                // ".webp" / ".png" / ".jpg"
};

struct TextureSlot {
  int image = -1;
  int uvSet = 0;
  bool valid() const { return image >= 0; }
};

enum class AlphaMode { Opaque, Mask, Blend };

struct Material {
  std::string name;
  Vec4 baseColor{1, 1, 1, 1};
  float metallic = 0.0f, roughness = 1.0f;
  Vec3 emissive{0, 0, 0};
  float emissiveStrength = 1.0f;
  float normalScale = 1.0f, occlusionStrength = 1.0f;
  AlphaMode alphaMode = AlphaMode::Opaque;
  float alphaCutoff = 0.5f;
  bool doubleSided = false;
  bool unlit = false;
  TextureSlot baseColorTex, normalTex, occlusionTex, emissiveTex;
  TextureSlot metalRoughTex;   // glTF packing: G=roughness, B=metallic
  TextureSlot roughnessTex, metallicTex;  // separate (non-glTF sources); packed on export when needed
};

struct Color8 { uint8_t r, g, b, a; };

struct Mesh {
  std::string name;
  std::vector<Vec3> positions, normals;
  std::vector<Vec4> tangents;
  std::vector<Vec2> uv0, uv1;
  std::vector<Color8> colors;
  std::vector<uint32_t> indices;  // triangle list
  int material = -1;

  size_t vertexCount() const { return positions.size(); }
  size_t triangleCount() const { return indices.size() / 3; }
  bool hasNormals() const { return normals.size() == positions.size() && !positions.empty(); }
  bool hasUV() const { return uv0.size() == positions.size() && !positions.empty(); }
  bool hasTangents() const { return tangents.size() == positions.size() && !positions.empty(); }
  size_t memoryBytes() const;
};

struct Node {
  std::string name;
  Mat4 local;
  std::vector<int> meshes;
  std::vector<int> children;
};

enum class UpAxis { Y, Z };

struct Scene {
  std::vector<Node> nodes;
  std::vector<int> roots;
  std::vector<Mesh> meshes;
  std::vector<Material> materials;
  std::vector<Image> images;
  std::string sourceFormat;
  std::string sourcePath;
  UpAxis up = UpAxis::Y;
  double metersPerUnit = 1.0;

  size_t totalTriangles() const;
  size_t totalVertices() const;
  Aabb bounds() const;               // world space
  bool hasTransforms() const;        // any non-identity node matrix or instanced mesh
  int addImage(Image&& im);
  int addMaterial(Material&& m);
};

}  // namespace rx
