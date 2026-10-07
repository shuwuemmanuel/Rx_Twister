// Loose texture binding, UDIM tile merging and multi-material texture atlasing.
#include "tex/textures.h"
#include "tex/image_ops.h"
#include "core/par.h"
#include "core/util.h"
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#define STB_RECT_PACK_IMPLEMENTATION
#include <stb/stb_rect_pack.h>

namespace rx {

namespace {

enum class Kind { None, Base, Normal, NormalDX, Rough, Gloss, Metal, AO, Emissive, Opacity, Height, MetalRough, ORM };

const std::map<std::string, Kind>& keywords() {
  static const std::map<std::string, Kind> k = {
      {"basecolor", Kind::Base}, {"base", Kind::Base}, {"color", Kind::Base}, {"colour", Kind::Base}, {"albedo", Kind::Base},
      {"diffuse", Kind::Base}, {"diff", Kind::Base}, {"col", Kind::Base}, {"bc", Kind::Base}, {"alb", Kind::Base},
      {"basecolour", Kind::Base}, {"albedotransparency", Kind::Base}, {"basemap", Kind::Base}, {"maintex", Kind::Base},
      {"normal", Kind::Normal}, {"normals", Kind::Normal}, {"nrm", Kind::Normal}, {"nor", Kind::Normal}, {"norm", Kind::Normal},
      {"normalmap", Kind::Normal}, {"nml", Kind::Normal}, {"normalgl", Kind::Normal}, {"normaldx", Kind::NormalDX},
      {"roughness", Kind::Rough}, {"rough", Kind::Rough}, {"rgh", Kind::Rough},
      {"gloss", Kind::Gloss}, {"glossiness", Kind::Gloss}, {"smoothness", Kind::Gloss},
      {"metallic", Kind::Metal}, {"metalness", Kind::Metal}, {"metal", Kind::Metal}, {"met", Kind::Metal}, {"mtl", Kind::Metal},
      {"ao", Kind::AO}, {"occlusion", Kind::AO}, {"ambientocclusion", Kind::AO}, {"occ", Kind::AO}, {"ambient", Kind::AO},
      {"emissive", Kind::Emissive}, {"emission", Kind::Emissive}, {"emit", Kind::Emissive}, {"glow", Kind::Emissive},
      {"illum", Kind::Emissive}, {"selfillum", Kind::Emissive}, {"emissivecolor", Kind::Emissive},
      {"opacity", Kind::Opacity}, {"alpha", Kind::Opacity}, {"transparency", Kind::Opacity}, {"mask", Kind::Opacity},
      {"height", Kind::Height}, {"disp", Kind::Height}, {"displacement", Kind::Height}, {"bump", Kind::Height},
      {"metallicroughness", Kind::MetalRough}, {"metalrough", Kind::MetalRough}, {"mr", Kind::MetalRough},
      {"orm", Kind::ORM}, {"arm", Kind::ORM}, {"occlusionroughnessmetallic", Kind::ORM}};
  return k;
}
// single letters accepted only as the last token: T_Rock_D, rock_n ...
Kind letterKind(const std::string& t) {
  if (t == "d" || t == "a" || t == "c") return Kind::Base;
  if (t == "n") return Kind::Normal;
  if (t == "r") return Kind::Rough;
  if (t == "m") return Kind::Metal;
  if (t == "e") return Kind::Emissive;
  if (t == "h") return Kind::Height;
  return Kind::None;
}

bool isImageExt(const std::string& e) {
  return e == "png" || e == "jpg" || e == "jpeg" || e == "webp" || e == "tga" || e == "bmp" || e == "psd" || e == "gif";
}

std::string normKey(const std::string& s) {
  std::string r;
  for (unsigned char c : s) if (std::isalnum(c)) r += char(std::tolower(c));
  return r;
}

struct Classified { std::string key; Kind kind = Kind::None; int tile = 1001; bool dx = false; };

Classified classify(const std::string& stem) {
  std::vector<std::string> tok;
  std::string cur;
  for (char ch : stem) {
    if (ch == '_' || ch == '-' || ch == '.' || ch == ' ') { if (!cur.empty()) tok.push_back(toLower(cur)); cur.clear(); }
    else cur += ch;
  }
  if (!cur.empty()) tok.push_back(toLower(cur));
  Classified c;
  std::vector<char> used(tok.size(), 0);
  auto isTile = [](const std::string& t) {
    if (t.size() != 4 || !std::all_of(t.begin(), t.end(), ::isdigit)) return false;
    int v = std::stoi(t); return v >= 1001 && v <= 1999;
  };
  // trailing keywords / tile / modifiers
  int i = int(tok.size()) - 1;
  for (; i >= 0; --i) {
    const std::string& t = tok[i];
    if (isTile(t)) { c.tile = std::stoi(t); used[i] = 1; continue; }
    if (t == "dx" || t == "directx") { c.dx = true; used[i] = 1; continue; }
    if (t == "gl" || t == "ogl" || t == "opengl" || t == "map" || t == "tex" || t == "texture" || t == "srgb" || t == "linear" ||
        t == "1k" || t == "2k" || t == "4k" || t == "8k" || t == "16k") { used[i] = 1; continue; }
    auto it = keywords().find(t);
    Kind k = it != keywords().end() ? it->second : (i == int(tok.size()) - 1 && tok.size() > 1 ? letterKind(t) : Kind::None);
    if (k == Kind::None) break;
    if (c.kind == Kind::None || c.kind == k || (c.kind == Kind::AO && k == Kind::AO)) { c.kind = k; used[i] = 1; continue; }
    break;
  }
  if (c.kind == Kind::None) {  // leading keyword: albedo_rock
    for (size_t j = 0; j < tok.size(); ++j) {
      auto it = keywords().find(tok[j]);
      if (it == keywords().end()) break;
      c.kind = it->second; used[j] = 1;
    }
  }
  if (c.kind == Kind::Normal && c.dx) c.kind = Kind::NormalDX;
  for (size_t j = 0; j < tok.size(); ++j) if (!used[j]) c.key += tok[j];
  return c;
}

struct Group {
  std::string key;
  std::map<Kind, std::map<int, std::string>> files;  // kind -> tile -> path
  bool dedicated = false;  // came from an explicit / texture-only folder
  bool udim() const {
    for (auto& [k, tiles] : files) if (tiles.size() > 1 || tiles.begin()->first != 1001) return true;
    return false;
  }
};

void scanPath(const fs::path& p, bool dedicated, std::map<std::string, Group>& groups, int depth = 0) {
  std::error_code ec;
  if (fs::is_directory(p, ec)) {
    for (auto& e : fs::directory_iterator(p, ec)) {
      if (e.is_directory(ec)) { if (depth < 2 && dedicated) scanPath(e.path(), true, groups, depth + 1); continue; }
      scanPath(e.path(), dedicated, groups, depth + 1);
    }
    return;
  }
  std::string path = pathToUtf8(p);
  if (!isImageExt(extOf(path))) return;
  Classified c = classify(pathToUtf8(p.stem()));
  if (c.kind == Kind::None) c.kind = Kind::Base;  // un-suffixed image: treat as colour
  Group& g = groups[c.key];
  g.key = c.key;
  g.dedicated |= dedicated;
  g.files[c.kind][c.tile] = path;
}

int loadImageFile(Scene& s, const std::string& path, uint32_t usage) {
  for (size_t i = 0; i < s.images.size(); ++i) if (s.images[i].sourcePath == path) return int(i);
  std::string err;
  auto blob = mapFile(path, &err);
  if (!blob) { logWarn("%s", err.c_str()); return -1; }
  Image im;
  im.name = stemOf(path);
  im.sourcePath = path;
  im.encoded = blob;
  im.mime = sniffMime(blob->data, blob->size);
  im.usage = usage;
  return s.addImage(std::move(im));
}

int pixelsImage(Scene& s, const std::string& name, std::shared_ptr<Pixels> px, uint32_t usage) {
  Image im;
  im.name = name; im.pixels = std::move(px); im.usage = usage;
  return s.addImage(std::move(im));
}

// Builds a UDIM atlas for one kind; tile (u,v) lands at column u, row (gh-1-v).
std::shared_ptr<Pixels> udimAtlas(const std::map<int, std::string>& tiles, int gw, int gh, int ts, const uint8_t fill[4]) {
  auto out = makePixels(gw * ts, gh * ts, fill[0], fill[1], fill[2], fill[3]);
  for (auto& [tile, path] : tiles) {
    int u = (tile - 1001) % 10, v = (tile - 1001) / 10;
    if (u >= gw || v >= gh) continue;
    std::string err;
    auto p = decodeImageFile(path, &err);
    if (!p) { logWarn("UDIM tile %s: %s", path.c_str(), err.c_str()); continue; }
    if (p->width != ts || p->height != ts) p = resizePixels(*p, ts, ts, false);
    out->hasAlpha |= p->hasAlpha;
    int ox = u * ts, oy = (gh - 1 - v) * ts;
    for (int y = 0; y < ts; ++y) memcpy(&out->rgba[(size_t(oy + y) * out->width + ox) * 4], &p->rgba[size_t(y) * ts * 4], size_t(ts) * 4);
  }
  return out;
}

void applyGroup(Scene& s, Material& m, int mi, const Group& g, const Options& o) {
  int gw = 1, gh = 1, ts = 0;
  const bool udim = g.udim() && o.udimAtlas;
  if (udim) {
    for (auto& [k, tiles] : g.files)
      for (auto& [tile, path] : tiles) {
        gw = std::max(gw, (tile - 1001) % 10 + 1);
        gh = std::max(gh, (tile - 1001) / 10 + 1);
        auto b = mapFile(path);
        int w, h; bool a;
        if (b && imageInfo(b->data, b->size, w, h, a)) ts = std::max({ts, w, h});
      }
    int limit = (o.maxTextureSize > 0 ? o.maxTextureSize : 16384) / std::max(gw, gh);
    ts = std::max(16, std::min(ts, limit));
  }
  auto get = [&](Kind k, uint32_t usage, const uint8_t fill[4]) -> int {
    auto it = g.files.find(k);
    if (it == g.files.end()) return -1;
    if (udim) return pixelsImage(s, g.key + "_udim_" + std::to_string(int(k)), udimAtlas(it->second, gw, gh, ts, fill), usage);
    return loadImageFile(s, it->second.begin()->second, usage);
  };
  static const uint8_t white[4] = {255, 255, 255, 255}, flat[4] = {128, 128, 255, 255}, black[4] = {0, 0, 0, 255};
  bool changed = false;
  if (!m.baseColorTex.valid()) { int i = get(Kind::Base, kUseColor, white); if (i >= 0) { m.baseColorTex = {i, 0}; changed = true; } }
  if (!m.normalTex.valid()) {
    int i = get(Kind::Normal, kUseNormal, flat);
    if (i < 0 && (i = get(Kind::NormalDX, kUseNormal, flat)) >= 0) {  // DirectX -> OpenGL: flip green
      auto p = imagePixels(s.images[i]);
      if (p) {
        auto q = std::make_shared<Pixels>(*p);
        for (size_t k = 1; k < q->rgba.size(); k += 4) q->rgba[k] = uint8_t(255 - q->rgba[k]);
        s.images[i].pixels = q; s.images[i].encoded.reset();
      }
    }
    if (i < 0 && o.normalGen != NormalGen::Off) {
      int hi = get(Kind::Height, kUseData, black);
      if (hi >= 0) if (auto hp = imagePixels(s.images[hi]))
        i = pixelsImage(s, g.key + "_normal_from_height", heightToNormal(*hp, o.normalStrength, o.normalMapSize), kUseNormal);
    }
    if (i >= 0) { m.normalTex = {i, 0}; changed = true; }
  }
  if (!m.metalRoughTex.valid()) {
    int i = get(Kind::ORM, kUseData, white);
    if (i >= 0) { m.metalRoughTex = {i, 0}; m.occlusionTex = {i, 0}; m.metallic = m.roughness = 1; changed = true; }
    else if ((i = get(Kind::MetalRough, kUseData, white)) >= 0) { m.metalRoughTex = {i, 0}; m.metallic = m.roughness = 1; changed = true; }
  }
  if (!m.metalRoughTex.valid()) {
    int r = get(Kind::Rough, kUseData, white);
    if (r < 0 && (r = get(Kind::Gloss, kUseData, black)) >= 0) {
      if (auto p = imagePixels(s.images[r])) {
        auto q = std::make_shared<Pixels>(*p);
        for (size_t k = 0; k < q->rgba.size(); k += 4) for (int c = 0; c < 3; ++c) q->rgba[k + c] = uint8_t(255 - q->rgba[k + c]);
        s.images[r].pixels = q; s.images[r].encoded.reset();
      }
    }
    if (r >= 0) { m.roughnessTex = {r, 0}; changed = true; }
    int mt = get(Kind::Metal, kUseData, black);
    if (mt >= 0) { m.metallicTex = {mt, 0}; changed = true; }
  }
  if (!m.occlusionTex.valid()) { int i = get(Kind::AO, kUseData, white); if (i >= 0) { m.occlusionTex = {i, 0}; changed = true; } }
  if (!m.emissiveTex.valid()) {
    int i = get(Kind::Emissive, kUseColor, black);
    if (i >= 0) { m.emissiveTex = {i, 0}; if (m.emissive.x + m.emissive.y + m.emissive.z == 0) m.emissive = {1, 1, 1}; changed = true; }
  }
  {
    int op = get(Kind::Opacity, kUseData, white);
    if (op >= 0) {
      auto ap = imagePixels(s.images[op]);
      std::shared_ptr<Pixels> base = m.baseColorTex.valid() ? imagePixels(s.images[m.baseColorTex.image]) : nullptr;
      if (ap) {
        int w = base ? base->width : ap->width, h = base ? base->height : ap->height;
        auto out = base ? std::make_shared<Pixels>(*base) : makePixels(w, h, 255, 255, 255, 255);
        auto a = (ap->width == w && ap->height == h) ? ap : resizePixels(*ap, w, h, false);
        bool binary = true;
        for (size_t k = 0; k < size_t(w) * h; ++k) {
          uint8_t v = a->rgba[k * 4];
          out->rgba[k * 4 + 3] = v;
          binary &= (v < 8 || v > 247);
        }
        out->hasAlpha = true;
        m.baseColorTex = {pixelsImage(s, g.key + "_basecolor_alpha", out, kUseColor), 0};
        m.alphaMode = binary ? AlphaMode::Mask : AlphaMode::Blend;
        changed = true;
      }
    }
  }
  if (!changed) return;
  logInfo("textures: bound '%s' -> material '%s'%s", g.key.c_str(), m.name.c_str(), udim ? " (UDIM atlas)" : "");
  if (udim) {
    for (auto& mesh : s.meshes) {
      if (mesh.material != mi) continue;
      for (auto& uv : mesh.uv0) { uv.x = uv.x / float(gw); uv.y = 1.0f - (1.0f - uv.y) / float(gh); }
    }
  }
}

}  // namespace

void bindLooseTextures(Scene& s, const Options& o) {
  std::map<std::string, Group> groups;
  bool explicitSources = !o.textureSources.empty();
  if (explicitSources) {
    for (auto& src : o.textureSources) scanPath(pathFromUtf8(src), true, groups);
  } else {
    if (!o.bindTextures || s.sourcePath.empty()) return;
    // self-contained formats define their materials completely; only bind when asked (--textures)
    const std::string fmt = s.sourceFormat;
    if (fmt == "gltf" || fmt == "glb" || fmt == "blend" || fmt == "usd" || fmt == "usda" || fmt == "usdc" || fmt == "usdz" || fmt == "3mf") return;
    bool needs = s.materials.empty();
    for (auto& m : s.materials) needs |= !m.baseColorTex.valid();
    if (!needs) return;
    fs::path dir = pathFromUtf8(s.sourcePath).parent_path();
    std::string stem = stemOf(s.sourcePath);
    std::error_code ec;
    for (const char* sub : {"textures", "Textures", "texture", "tex", "maps", "Maps", "images", "materials"})
      if (fs::is_directory(dir / sub, ec)) scanPath(dir / sub, true, groups);
    for (auto sub : {stem + "_textures", stem + ".fbm", stem + "_tex"})
      if (fs::is_directory(dir / pathFromUtf8(sub), ec)) scanPath(dir / pathFromUtf8(sub), true, groups);
    if (fs::is_directory(dir, ec))
      for (auto& e : fs::directory_iterator(dir, ec)) if (e.is_regular_file(ec)) scanPath(e.path(), false, groups);
  }
  if (groups.empty()) return;

  if (s.materials.empty()) {
    Material m; m.name = "material";
    s.materials.push_back(m);
  }
  for (auto& mesh : s.meshes) if (mesh.material < 0) mesh.material = 0;

  // match by name: material name, then names of meshes using it, then model stem
  std::set<std::string> usedGroups;
  std::vector<int> unmatched;
  for (size_t mi = 0; mi < s.materials.size(); ++mi) {
    Material& m = s.materials[mi];
    std::vector<std::string> names = {normKey(m.name)};
    for (auto& mesh : s.meshes) if (mesh.material == int(mi)) names.push_back(normKey(mesh.name));
    const Group* best = nullptr;
    size_t bestLen = 0;
    for (auto& [key, g] : groups) {
      if (key.empty()) continue;
      for (auto& n : names) {
        if (n.empty()) continue;
        size_t l = (n == key) ? key.size() * 4 : (n.find(key) != std::string::npos || key.find(n) != std::string::npos) ? std::min(n.size(), key.size()) : 0;
        if (l > bestLen) { bestLen = l; best = &g; }
      }
    }
    if (best && bestLen >= 3) { applyGroup(s, m, int(mi), *best, o); usedGroups.insert(best->key); }
    else unmatched.push_back(int(mi));
  }
  // fallback: one remaining texture set -> every material still lacking textures
  std::vector<const Group*> rest;
  for (auto& [k, g] : groups) if (!usedGroups.count(k) && (g.dedicated || explicitSources || groups.size() == 1)) rest.push_back(&g);
  if (!unmatched.empty() && rest.size() == 1) {
    for (int mi : unmatched) applyGroup(s, s.materials[mi], mi, *rest[0], o);
  } else if (!unmatched.empty() && !rest.empty()) {
    // pair the remaining sets with the remaining materials in order
    for (size_t i = 0; i < unmatched.size() && i < rest.size(); ++i) applyGroup(s, s.materials[unmatched[i]], unmatched[i], *rest[i], o);
    if (rest.size() != unmatched.size())
      logWarn("textures: %zu texture sets for %zu unmatched materials - paired in order", rest.size(), unmatched.size());
  }
}

// ------------------------------------------------------------------ material atlas
void atlasMaterials(Scene& s, const Options& o) {
  std::vector<std::vector<int>> users(s.materials.size());
  for (size_t i = 0; i < s.meshes.size(); ++i) if (s.meshes[i].material >= 0) users[s.meshes[i].material].push_back(int(i));
  // eligible: UVs inside [0,1], uv set 0
  std::map<std::tuple<int, bool, bool>, std::vector<int>> groups;
  for (size_t mi = 0; mi < s.materials.size(); ++mi) {
    const Material& m = s.materials[mi];
    if (users[mi].empty()) continue;
    bool ok = true;
    for (const TextureSlot* t : {&m.baseColorTex, &m.normalTex, &m.metalRoughTex, &m.occlusionTex, &m.emissiveTex})
      ok &= !t->valid() || t->uvSet == 0;
    for (int u : users[mi]) {
      const Mesh& me = s.meshes[u];
      if (!me.hasUV()) { ok = false; break; }
      for (auto& uv : me.uv0) if (uv.x < -0.001f || uv.x > 1.001f || uv.y < -0.001f || uv.y > 1.001f) { ok = false; break; }
    }
    if (!ok) { logVerbose("atlas: material '%s' skipped (tiling / no UVs)", m.name.c_str()); continue; }
    groups[{int(m.alphaMode), m.doubleSided, m.unlit}].push_back(int(mi));
  }
  int maxSize = o.atlasMaxSize;
  if (o.maxTextureSize > 0) maxSize = std::min(maxSize, o.maxTextureSize);
  for (auto& [gk, mats] : groups) {
    if (mats.size() < 2) continue;
    // rectangle per material
    std::vector<std::pair<int, int>> dims;
    double area = 0;
    bool anyNormal = false, anyMR = false, anyEm = false, anyAO = false;
    float r0 = s.materials[mats[0]].roughness, m0 = s.materials[mats[0]].metallic;
    for (int mi : mats) {
      const Material& m = s.materials[mi];
      int w = 16, h = 16;
      for (const TextureSlot* t : {&m.baseColorTex, &m.normalTex, &m.metalRoughTex, &m.occlusionTex, &m.emissiveTex})
        if (t->valid()) { int iw, ih; if (imageSize(s.images[t->image], iw, ih)) { w = std::max(w, iw); h = std::max(h, ih); } }
      dims.push_back({w, h});
      area += double(w + 8) * (h + 8);
      anyNormal |= m.normalTex.valid();
      anyMR |= m.metalRoughTex.valid() || m.roughness != r0 || m.metallic != m0;
      anyAO |= m.occlusionTex.valid();
      anyEm |= m.emissiveTex.valid() || (m.emissive.x + m.emissive.y + m.emissive.z) > 0;
    }
    // pick an atlas size, shrinking rects until they pack
    int A = std::min(maxSize, std::max(64, nearestPow2(int(std::ceil(std::sqrt(area * 1.15))))));
    float scale = 1.0f;
    std::vector<stbrp_rect> rects;
    const int pad = 4;
    for (int attempt = 0; attempt < 24; ++attempt) {
      rects.assign(mats.size(), {});
      for (size_t i = 0; i < mats.size(); ++i) {
        rects[i].id = int(i);
        rects[i].w = std::max(4, int(dims[i].first * scale)) + pad * 2;
        rects[i].h = std::max(4, int(dims[i].second * scale)) + pad * 2;
      }
      std::vector<stbrp_node> nodes(A);
      stbrp_context ctx;
      stbrp_init_target(&ctx, A, A, nodes.data(), A);
      if (stbrp_pack_rects(&ctx, rects.data(), int(rects.size()))) break;
      if (A < maxSize) A *= 2; else scale *= 0.85f;
    }
    auto atlasOf = [&](auto fillFn, uint32_t usage, bool normal) {
      auto out = makePixels(A, A, 0, 0, 0, 255);
      for (size_t i = 0; i < mats.size(); ++i) {
        const stbrp_rect& r = rects[i];
        int iw = r.w - pad * 2, ih = r.h - pad * 2;
        std::shared_ptr<Pixels> src = fillFn(s.materials[mats[i]], iw, ih);
        out->hasAlpha |= src->hasAlpha;
        // copy with edge padding (clamp)
        for (int y = -pad; y < ih + pad; ++y)
          for (int x = -pad; x < iw + pad; ++x) {
            int sx = std::clamp(x, 0, iw - 1), sy = std::clamp(y, 0, ih - 1);
            memcpy(&out->rgba[(size_t(r.y + pad + y) * A + (r.x + pad + x)) * 4], &src->rgba[(size_t(sy) * iw + sx) * 4], 4);
          }
      }
      (void)normal;
      return pixelsImage(s, "atlas_" + std::to_string(s.images.size()), out, usage);
    };
    auto texOr = [&](const TextureSlot& t, int w, int h, Vec4 factor, uint8_t defA) {
      std::shared_ptr<Pixels> p;
      if (t.valid()) if (auto src = imagePixels(s.images[t.image])) p = (src->width == w && src->height == h) ? std::make_shared<Pixels>(*src) : resizePixels(*src, w, h, false);
      if (!p) p = makePixels(w, h, 255, 255, 255, defA);
      for (size_t k = 0; k < size_t(w) * h; ++k) {
        uint8_t* q = &p->rgba[k * 4];
        q[0] = uint8_t(q[0] * factor.x + 0.5f); q[1] = uint8_t(q[1] * factor.y + 0.5f);
        q[2] = uint8_t(q[2] * factor.z + 0.5f); q[3] = uint8_t(q[3] * factor.w + 0.5f);
      }
      if (factor.w < 1.0f) p->hasAlpha = true;
      return p;
    };
    Material merged = s.materials[mats[0]];
    merged.name = "atlas_material_" + std::to_string(gk == groups.begin()->first ? 0 : s.materials.size());
    merged.baseColor = {1, 1, 1, 1};
    merged.baseColorTex = {atlasOf([&](const Material& m, int w, int h) {
      auto c = m.baseColor;
      return texOr(m.baseColorTex, w, h, {std::clamp(c.x, 0.f, 1.f), std::clamp(c.y, 0.f, 1.f), std::clamp(c.z, 0.f, 1.f), std::clamp(c.w, 0.f, 1.f)}, 255);
    }, kUseColor, false), 0};
    merged.normalTex = {};
    if (anyNormal)
      merged.normalTex = {atlasOf([&](const Material& m, int w, int h) {
        if (m.normalTex.valid()) return texOr(m.normalTex, w, h, {1, 1, 1, 1}, 255);
        return makePixels(w, h, 128, 128, 255, 255);
      }, kUseNormal, true), 0};
    merged.metalRoughTex = merged.occlusionTex = {};
    if (anyMR || anyAO) {
      int idx = atlasOf([&](const Material& m, int w, int h) {
        auto p = texOr(m.metalRoughTex, w, h, {1, std::clamp(m.roughness, 0.f, 1.f), std::clamp(m.metallic, 0.f, 1.f), 1}, 255);
        if (m.occlusionTex.valid() && m.occlusionTex.image != m.metalRoughTex.image) {
          auto ao = texOr(m.occlusionTex, w, h, {1, 1, 1, 1}, 255);
          for (size_t k = 0; k < size_t(w) * h; ++k) p->rgba[k * 4] = ao->rgba[k * 4];
        } else if (!m.occlusionTex.valid()) for (size_t k = 0; k < size_t(w) * h; ++k) p->rgba[k * 4] = 255;
        return p;
      }, kUseData, false);
      merged.metalRoughTex = {idx, 0};
      merged.roughness = merged.metallic = 1;
      if (anyAO) merged.occlusionTex = {idx, 0};
    }
    merged.emissiveTex = {};
    if (anyEm) {
      merged.emissiveTex = {atlasOf([&](const Material& m, int w, int h) {
        auto e = m.emissive * m.emissiveStrength;
        return texOr(m.emissiveTex, w, h, {std::min(e.x, 1.f), std::min(e.y, 1.f), std::min(e.z, 1.f), 1}, 255);
      }, kUseColor, false), 0};
      merged.emissive = {1, 1, 1}; merged.emissiveStrength = 1;
    }
    int mergedIdx = s.addMaterial(std::move(merged));
    for (size_t i = 0; i < mats.size(); ++i) {
      const stbrp_rect& r = rects[i];
      float ox = float(r.x + pad) / A, oy = float(r.y + pad) / A;
      float sx = float(r.w - pad * 2) / A, sy = float(r.h - pad * 2) / A;
      for (int u : users[mats[i]]) {
        Mesh& me = s.meshes[u];
        for (auto& uv : me.uv0) uv = {ox + std::clamp(uv.x, 0.f, 1.f) * sx, oy + std::clamp(uv.y, 0.f, 1.f) * sy};
        me.material = mergedIdx;
      }
    }
    logInfo("atlas: merged %zu materials into one %dx%d atlas", mats.size(), A, A);
  }
}

}  // namespace rx
