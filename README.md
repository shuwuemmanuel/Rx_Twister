# Rx Twister

A fast, CPU-only 3D model **converter and optimiser** written in C++20.

Load a model (with or without embedded textures), process it, and write it in another
format. **GLB with WebP textures is the default output**: it's small, self-contained,
and readable by Blender, three.js, Babylon.js, Godot and Unity (glTFast).

```
rxtwister model.fbx                          # -> model_rx.glb (WebP textures embedded)
rxtwister model.obj out/model.usdz           # Apple AR / USD
rxtwister model.glb model.fbx --preset unity # FBX + PNG textures for Unity
rxtwister scan.ply scan.glb --unsubdivide 2 --normal-map bake --max-texture 2048
```

## Features

| Need | How |
| --- | --- |
| Convert between formats | 78 import / 19 export extensions (see [Formats](#formats)) |
| Embedded textures | Embedded when the target supports it (GLB, glTF `.bin`, USDZ, FBX, .blend). Other formats get a `<name>_textures/` folder. Toggle with `--embed` / `--no-embed`. |
| Light output | WebP for glTF (`EXT_texture_webp`), PNG/JPEG elsewhere, `--quality`, `--texture-format`. Optional `--compress meshopt`. |
| Texture resizing | `--max-texture 2048`, `--pot` (Lanczos3, alpha-correct, normal maps re-normalised) |
| Subdivide | `--subdivide N`: Loop (smooth) or `--subdiv-scheme linear`. UVs are interpolated per face, so UV seams stay exact and seam positions never crack. |
| Unsubdivide | `--unsubdivide N`: each level removes about 75 % of triangles. Quadric simplification weighted by UVs and normals; seams preserved. |
| Remesh with UVs | `--remesh [--remesh-faces N \| --remesh-edge L]`: isotropic remeshing that keeps UV seams fixed and re-derives interior UVs from the original surface. |
| Many textures → one model | Loose textures are matched to materials by name (`rock_albedo.png`, `T_Rock_N.tga`, `rock_roughness.1001.png`, ...). DirectX normals are flipped to OpenGL, gloss is inverted to roughness, roughness/metal/AO are packed into ORM, and opacity is merged into base-colour alpha. |
| UDIM | Tiles `name.1001.png`, `name.1002.png`, ... are merged into one atlas and UVs are remapped. |
| Material atlas | `--atlas --merge-meshes`: many materials become one atlas and one material (fewer draw calls). |
| Normal map generator (switchable) | `--normal-map [auto\|bake\|height\|off]`, off by default. **bake** captures the original high-poly detail after unsubdivide / simplify / remesh. **height** derives normals from base-colour luminance. Applies only to materials without a normal map unless `--force-normal-map`. |
| Engines | `--preset web\|blender\|unity\|unreal\|godot`. FBX is always written Y-up in centimetres, which Unity and Unreal import at the correct scale. |
| Blender | `.blend` / Alembic / USD import and export through a headless Blender (`blender -b`), found automatically or via `--blender <path>` / `RX_BLENDER`. Textures are packed into the `.blend`. |
| Big files | Memory-mapped parallel readers, a streamed GLB writer, a memory-budgeted texture pipeline, parallel welding, and automatic `.gltf` + `.bin` output above the 4 GB GLB limit. |

## Performance

Measured on a 4-core / 16 GB Linux VM. The `-j 2` column limits the tool to two worker
threads, which is roughly what a 2-core 5th-gen laptop i5/i7 provides.

| Input | Output | Time (4 threads) | Time (`-j 2`) | Peak RAM |
| --- | --- | --- | --- | --- |
| OBJ 4.4 GB, 40.5M triangles, plus four 8K PNG textures (4.5 GB total) | GLB 1.1 GB with WebP | **33 s** | 46 s | **3.9 GB** |
| OBJ 0.8 GB, 8M triangles, plus four 8K PNG textures | GLB 267 MB with WebP | 14 s | – | 1.9 GB |
| GLB 1.1 GB, 40.5M triangles | GLB with textures capped at 2048 px | 19 s | – | 3.6 GB |
| OBJ, 1M triangles | `--remesh-faces 200000` | 12 s | – | – |
| OBJ, 1M triangles | `--subdivide 2` (→ 16M triangles) | 5.7 s | – | – |

Peak memory stays below the input size, so a 5 GB model fits on an 8 GB laptop.
Reproduce with `rx_make_big <dir> 4500 8192` followed by `rxtwister <dir>/big.obj out.glb -v`.
Verbose mode prints per-stage timings and memory.

## Formats

`rxtwister formats` prints the live list for your build.

* **Native readers** (fast, memory mapped, parallel): `glb`, `gltf` (incl. `KHR_mesh_quantization`,
  `EXT_meshopt_compression`, `KHR_draco_mesh_compression`, `EXT_texture_webp`, sparse accessors), `obj`/`mtl`,
  `stl` (binary/ASCII), `ply` (ASCII / binary LE / BE, per-corner texcoords).
* **Native writers**: `glb`, `gltf`, `obj`/`mtl` (with PBR `Pr`/`Pm` extensions), `stl`, `ply`, `usda`/`usd`,
  `usdz` (64-byte aligned package, UsdPreviewSurface).
* **Via Assimp**: reads 3ds, 3mf, ac, amf, ase, b3d, blend (legacy), bvh, cob, dae, dxf, fbx, ifc, iqm,
  irr, lwo, lws, lxo, md2/md3/md5, mdl, ms3d, nff, off, ogex, pmx, q3o, raw, sib, smd, step, ter, x, x3d,
  xgl and more. Writes **fbx** (textures embedded), dae, 3ds, x, x3d, 3mf, stp, assbin.
* **Via Blender** (when installed): reads `.blend`, `.abc`, `.usd/.usda/.usdc/.usdz`; writes `.blend`
  (textures packed), `.abc`, `.usdc`.

Validation done in this repository: the glTF outputs pass the Khronos glTF-Validator with
0 errors; the USD/USDZ outputs pass all 28 of Pixar's `UsdValidation` validators with 0 issues; and
GLB (WebP), FBX (embedded textures), OBJ and `.blend` outputs re-import correctly in Blender 4.0.

## Usage

```
rxtwister <input> [output] [options]   convert (default: <input>_rx.glb)
rxtwister info <file>                  describe a model
rxtwister formats                      list formats
rxtwister --help                       all options
```

Common recipes:

```bash
# web: GLB + WebP, textures capped at 2K
rxtwister castle.fbx --preset web

# keep textures as files next to the glTF instead of embedding
rxtwister castle.fbx castle.gltf --no-embed --texture-format png

# reduce a 20M-triangle scan to ~1.25M triangles and bake the lost detail into a normal map
rxtwister scan.obj scan.glb --unsubdivide 2 --normal-map bake --normal-size 4096

# clean, even topology at 50k faces while keeping the existing texture mapping
rxtwister sculpt.glb clean.glb --remesh-faces 50000

# a bare mesh plus a folder of textures -> one textured GLB
rxtwister statue.obj --textures ./statue_textures

# many materials -> 1 atlas + 1 draw call
rxtwister props.fbx props.glb --atlas --merge-meshes --atlas-size 4096

# Unreal / Unity
rxtwister car.glb --preset unreal          # car_rx.fbx, cm, Y-up (Unreal converts), tangents
rxtwister car.glb car.fbx --preset unity

# Blender
rxtwister scene.blend scene.glb
rxtwister model.glb model.blend
```

## Building

Requirements: CMake ≥ 3.20 and a C++20 compiler (GCC 11+, Clang 14+, MSVC 2022).

Dependencies: meshoptimizer, libwebp, stb, nlohmann_json, Assimp (optional but recommended), oneTBB
(optional), Draco (optional), zlib (optional). CMake uses installed packages first and **downloads
anything missing** with FetchContent (`-DRX_FETCH_DEPS=OFF` turns that off).

**Linux (Debian / Ubuntu)**
```bash
sudo apt install cmake ninja-build libassimp-dev libwebp-dev libtbb-dev libmeshoptimizer-dev \
                 nlohmann-json3-dev libstb-dev libdraco-dev zlib1g-dev
cmake -S . -B build -G Ninja && cmake --build build
./build/rx_tests build/test_out     # end-to-end tests
```

**Windows (vcpkg)**
```powershell
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
```

**macOS (Homebrew)**
```bash
brew install cmake ninja assimp webp nlohmann-json tbb   # meshoptimizer + stb are downloaded by CMake
cmake -S . -B build -G Ninja && cmake --build build
```

Add `-DRX_NATIVE_ARCH=ON` to optimise for the build machine's CPU.

Outputs: `rxtwister` (CLI), `librxtwister` (shared library with the C API) and `librxcore` (static C++ core).

## Embedding the core

C++ (`include/rx/rx.h`):
```cpp
rx::Options o;
o.input = "in.fbx";
o.output = "out.glb";
o.maxTextureSize = 2048;
o.unsubdivide = 1;
o.onProgress = [](const std::string& stage, double f) { /* update UI */ };
rx::Result r = rx::convert(o);
```

C (`include/rx/c_api.h`), for C#, Python, Unity and Unreal plugins or a GUI:
```c
char err[512];
int rc = rx_convert("in.fbx", "out.glb", "--max-texture 2048 --unsubdivide 1", NULL, NULL, err, sizeof err);
```

## Architecture

```
include/rx/      public API: scene model, options, pipeline, C API
src/core/        mmap, parallel helpers (TBB or std::thread), memory gate, logging
src/geom/        weld, normals, tangents, Loop/linear subdivision, UV-aware simplification,
                 UV-preserving isotropic remesher, BVH (closest point)
src/tex/         decode/encode (stb, libwebp), Lanczos resampling, ORM packing, height->normal,
                 normal baking, loose texture binder, UDIM merge, material atlas
src/io/          glTF/GLB, OBJ, STL, PLY, USD/USDZ (native); Assimp bridge; Blender bridge
src/pipeline.cpp load -> bind textures -> axis/units -> weld -> topology -> normals -> atlas ->
                 normal maps -> tangents -> ORM layout -> cache optimisation -> encode textures -> write
apps/cli/        rxtwister
tests/           end-to-end tests (111 checks) + large model generator
```

How the remesher keeps UVs: the mesh is held as welded positions plus per-corner UVs, so a UV seam is
an edge whose two faces disagree on UVs. Seam and boundary vertices are locked; they can gain
vertices through edge splits but never move or disappear, so every UV island keeps its outline.
Interior vertices are relaxed tangentially and projected back onto the source surface (BVH). Each
one takes its UV from its own one-ring, which always lies inside a single chart. On the test sphere,
100 % of remeshed vertices land within 0.01 UV of the original mapping, and the mean minimum
triangle angle rises from 32° to 47°.

## Limitations

* Static geometry only: skeletons, skinning, animation and morph targets are **not** carried over
  (a warning is printed).
* glTF `KHR_texture_transform`, cameras and lights are dropped.
* `--compress meshopt` output needs a viewer with `EXT_meshopt_compression` support (three.js,
  Babylon.js, glTFast). Blender 4.0's importer does not support it, so it is off by default.
* WebP inside `.glb` works in Blender, browsers, Godot and Unity glTFast. Unreal's glTF importer does
  not read WebP, which is why the Unreal and Unity presets write FBX with PNG textures.
* Alembic and USD through Blender need a Blender build that includes them (official builds do; some
  Linux distribution packages don't, and Rx Twister reports this clearly).
* Remeshing locks UV seams. Models whose UVs are cut into many tiny islands keep their seam density.
* Writing Draco-compressed glTF and KTX2/Basis textures is not implemented yet (they can be read or
  passed through).
