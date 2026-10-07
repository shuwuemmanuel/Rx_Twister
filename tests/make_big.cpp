// Generates a large stress-test model: <out_dir>/big.obj (+ .mtl) with N*N*2 triangles and four 8K PNG textures.
#include "rx/rx.h"
#include "core/par.h"
#include "core/util.h"
#include "tex/image_ops.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
using namespace rx;
int main(int argc, char** argv) {
  if (argc < 3) { printf("usage: make_big <out_dir> <grid N> [texture size]\n"); return 1; }
  std::string dir = argv[1];
  int N = atoi(argv[2]), T = argc > 3 ? atoi(argv[3]) : 8192;
  fs::create_directories(dir);
  Scene s;
  Mesh m; m.name = "terrain"; m.material = 0;
  m.positions.resize(size_t(N) * N); m.uv0.resize(m.positions.size()); m.normals.resize(m.positions.size());
  parallelFor(size_t(N), [&](size_t y) {
    for (int x = 0; x < N; ++x) {
      float u = float(x) / (N - 1), v = float(y) / (N - 1);
      float h = 0.05f * std::sin(u * 40) * std::cos(v * 37) + 0.01f * std::sin(u * 400 + v * 300);
      size_t i = y * N + x;
      m.positions[i] = {u * 10, h * 10, v * 10};
      m.uv0[i] = {u, v};
      m.normals[i] = {0, 1, 0};
    }
  }, 16);
  m.indices.resize(size_t(N - 1) * (N - 1) * 6);
  parallelFor(size_t(N - 1), [&](size_t y) {
    for (int x = 0; x < N - 1; ++x) {
      uint32_t a = uint32_t(y * N + x), b = a + 1, c = a + N, d = c + 1;
      uint32_t* o = &m.indices[(y * (N - 1) + x) * 6];
      o[0] = a; o[1] = c; o[2] = b; o[3] = b; o[4] = c; o[5] = d;
    }
  }, 16);
  s.meshes.push_back(std::move(m));
  Node nd; nd.meshes = {0}; s.nodes.push_back(nd); s.roots = {0};
  Material mat; mat.name = "ground";
  const char* names[] = {"ground_basecolor", "ground_normal", "ground_roughness", "ground_ao"};
  for (int k = 0; k < 4; ++k) {
    auto px = makePixels(T, T);
    parallelFor(size_t(T), [&](size_t y) {
      uint32_t seed = uint32_t(y * 2654435761u + k);
      for (int x = 0; x < T; ++x) {
        seed = seed * 1664525u + 1013904223u;
        uint8_t n = uint8_t(seed >> 27);
        uint8_t* p = &px->rgba[(y * T + x) * 4];
        float f = 0.5f + 0.5f * std::sin(x * 0.01f + k) * std::cos(y * 0.013f);
        p[0] = uint8_t(f * 200 + n); p[1] = uint8_t(f * 150 + n); p[2] = k == 1 ? 255 : uint8_t(f * 100 + n); p[3] = 255;
      }
    }, 16);
    Image im; im.name = names[k]; im.pixels = px; s.images.push_back(im);
  }
  mat.baseColorTex = {0, 0}; mat.normalTex = {1, 0}; mat.roughnessTex = {2, 0}; mat.occlusionTex = {3, 0};
  s.materials.push_back(mat);
  Options o; o.texFormat = TexFormat::PNG; o.optimize = false; o.packOrm = false;
  std::string err;
  Timer t;
  if (!saveScene(s, dir + "/big.obj", o, err)) { printf("error: %s\n", err.c_str()); return 1; }
  printf("wrote %s/big.obj in %.1fs\n", dir.c_str(), t.seconds());
}
