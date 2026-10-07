#include "tex/textures.h"
#include "tex/image_ops.h"
#include "core/par.h"
#include "core/util.h"
#include <atomic>
#include <cmath>
#include <map>
#include <mutex>

namespace rx {

void markImageUsage(Scene& s) {
  for (auto& im : s.images) im.usage = 0;
  auto mark = [&](const TextureSlot& t, uint32_t u) {
    if (t.image >= 0 && size_t(t.image) < s.images.size()) s.images[t.image].usage |= u;
  };
  for (auto& m : s.materials) {
    mark(m.baseColorTex, kUseColor); mark(m.emissiveTex, kUseColor);
    mark(m.normalTex, kUseNormal);
    mark(m.occlusionTex, kUseData); mark(m.metalRoughTex, kUseData);
    mark(m.roughnessTex, kUseData); mark(m.metallicTex, kUseData);
  }
}

static std::mutex g_decodeMutex;  // guards lazy decode of a shared image

std::shared_ptr<Pixels> imagePixels(Image& im, std::string* err) {
  {
    std::lock_guard<std::mutex> lk(g_decodeMutex);
    if (im.pixels) return im.pixels;
    if (!im.encoded) { if (err) *err = "image has no data"; return nullptr; }
  }
  auto p = decodeImage(im.encoded->data, im.encoded->size, err);  // decode outside the lock
  std::lock_guard<std::mutex> lk(g_decodeMutex);
  if (!im.pixels) im.pixels = p;
  return im.pixels;
}

bool imageSize(const Image& im, int& w, int& h) {
  if (im.pixels) { w = im.pixels->width; h = im.pixels->height; return true; }
  bool a;
  return im.encoded && imageInfo(im.encoded->data, im.encoded->size, w, h, a);
}

// ------------------------------------------------------------------ ORM packing
static std::shared_ptr<Pixels> resizedTo(Image& im, int w, int h) {
  auto p = imagePixels(im);
  if (!p) return nullptr;
  if (p->width == w && p->height == h) return p;
  return resizePixels(*p, w, h, false);
}

void packMetalRough(Scene& s) {
  {  // decode every source in parallel up front
    std::vector<int> need;
    for (auto& m : s.materials)
      if (!m.metalRoughTex.valid() && (m.roughnessTex.valid() || m.metallicTex.valid()))
        for (const TextureSlot* t : {&m.roughnessTex, &m.metallicTex, &m.occlusionTex})
          if (t->valid() && std::find(need.begin(), need.end(), t->image) == need.end()) need.push_back(t->image);
    parallelFor(need.size(), [&](size_t i) { imagePixels(s.images[need[i]]); }, 1);
  }
  std::map<std::tuple<int, int, int>, int> cache;
  for (auto& m : s.materials) {
    if (m.metalRoughTex.valid() || (!m.roughnessTex.valid() && !m.metallicTex.valid())) {
      m.roughnessTex = m.metallicTex = {};
      continue;
    }
    int r = m.roughnessTex.image, mt = m.metallicTex.image;
    int ao = (m.occlusionTex.valid()) ? m.occlusionTex.image : -1;
    auto key = std::make_tuple(r, mt, ao);
    auto it = cache.find(key);
    int idx;
    if (it != cache.end()) idx = it->second;
    else {
      int w = 0, h = 0;
      for (int i : {r, mt, ao}) {
        int iw, ih;
        if (i >= 0 && imageSize(s.images[i], iw, ih)) { w = std::max(w, iw); h = std::max(h, ih); }
      }
      if (!w) continue;
      auto rp = r >= 0 ? resizedTo(s.images[r], w, h) : nullptr;
      auto mp = mt >= 0 ? resizedTo(s.images[mt], w, h) : nullptr;
      auto ap = ao >= 0 ? resizedTo(s.images[ao], w, h) : nullptr;
      auto out = makePixels(w, h, 255, 255, 255, 255);
      const uint8_t rc = uint8_t(std::clamp(m.roughness, 0.0f, 1.0f) * 255 + 0.5f);
      const uint8_t mc = uint8_t(std::clamp(m.metallic, 0.0f, 1.0f) * 255 + 0.5f);
      parallelFor(size_t(w) * h, [&](size_t i) {
        uint8_t* o = &out->rgba[i * 4];
        o[0] = ap ? ap->rgba[i * 4] : 255;
        o[1] = rp ? rp->rgba[i * 4 + 1] : rc;   // grey maps: any channel works; G keeps packed sources right
        o[2] = mp ? mp->rgba[i * 4 + 2] : mc;
      }, 1 << 15);
      Image im;
      im.name = (m.name.empty() ? std::string("material") : m.name) + "_metalRough";
      im.pixels = out;
      im.usage = kUseData;
      idx = s.addImage(std::move(im));
      cache[key] = idx;
    }
    m.metalRoughTex = {idx, m.roughnessTex.valid() ? m.roughnessTex.uvSet : m.metallicTex.uvSet};
    m.roughness = 1.0f;
    m.metallic = 1.0f;
    if (ao >= 0) m.occlusionTex = {idx, m.occlusionTex.uvSet};
    m.roughnessTex = m.metallicTex = {};
  }
}

void unpackMetalRough(Scene& s) {
  std::map<int, std::pair<int, int>> cache;
  for (auto& m : s.materials) {
    if (!m.metalRoughTex.valid() || m.roughnessTex.valid()) continue;
    int src = m.metalRoughTex.image;
    auto it = cache.find(src);
    if (it == cache.end()) {
      auto p = imagePixels(s.images[src]);
      if (!p) continue;
      auto r = makePixels(p->width, p->height), mt = makePixels(p->width, p->height);
      for (size_t i = 0; i < size_t(p->width) * p->height; ++i) {
        uint8_t g = p->rgba[i * 4 + 1], b = p->rgba[i * 4 + 2];
        r->rgba[i * 4] = r->rgba[i * 4 + 1] = r->rgba[i * 4 + 2] = g;
        mt->rgba[i * 4] = mt->rgba[i * 4 + 1] = mt->rgba[i * 4 + 2] = b;
      }
      Image ri, mi;
      std::string base = s.images[src].name.empty() ? "texture" : s.images[src].name;
      ri.name = base + "_roughness"; ri.pixels = r; ri.usage = kUseData;
      mi.name = base + "_metallic"; mi.pixels = mt; mi.usage = kUseData;
      int a = s.addImage(std::move(ri)), b = s.addImage(std::move(mi));
      it = cache.emplace(src, std::make_pair(a, b)).first;
    }
    m.roughnessTex = {it->second.first, m.metalRoughTex.uvSet};
    m.metallicTex = {it->second.second, m.metalRoughTex.uvSet};
  }
}

// ------------------------------------------------------------------ height normals
std::shared_ptr<Pixels> heightToNormal(const Pixels& src, float strength, int maxSize) {
  const Pixels* in = &src;
  std::shared_ptr<Pixels> small;
  if (maxSize > 0 && std::max(src.width, src.height) > maxSize) {
    float k = float(maxSize) / float(std::max(src.width, src.height));
    small = resizePixels(src, std::max(1, int(src.width * k)), std::max(1, int(src.height * k)), false);
    in = small.get();
  }
  const int w = in->width, h = in->height;
  std::vector<float> hgt(size_t(w) * h);
  parallelFor(hgt.size(), [&](size_t i) {
    const uint8_t* p = &in->rgba[i * 4];
    hgt[i] = (0.2126f * p[0] + 0.7152f * p[1] + 0.0722f * p[2]) / 255.0f;
  }, 1 << 15);
  auto out = makePixels(w, h);
  const float scale = strength * float(std::max(w, h)) / 512.0f;  // resolution independent strength
  parallelFor(size_t(h), [&](size_t y) {
    auto H = [&](int x, int yy) { x = (x % w + w) % w; yy = (yy % h + h) % h; return hgt[size_t(yy) * w + x]; };
    for (int x = 0; x < w; ++x) {
      int yi = int(y);
      float dx = (H(x + 1, yi - 1) + 2 * H(x + 1, yi) + H(x + 1, yi + 1)) - (H(x - 1, yi - 1) + 2 * H(x - 1, yi) + H(x - 1, yi + 1));
      float dy = (H(x - 1, yi + 1) + 2 * H(x, yi + 1) + H(x + 1, yi + 1)) - (H(x - 1, yi - 1) + 2 * H(x, yi - 1) + H(x + 1, yi - 1));
      Vec3 n = normalize(Vec3{-dx * scale * 0.125f, dy * scale * 0.125f, 1.0f});  // OpenGL (+Y up)
      uint8_t* o = &out->rgba[(y * w + x) * 4];
      o[0] = uint8_t(n.x * 127.5f + 127.5f); o[1] = uint8_t(n.y * 127.5f + 127.5f); o[2] = uint8_t(n.z * 127.5f + 127.5f); o[3] = 255;
    }
  }, 8);
  return out;
}

void generateHeightNormals(Scene& s, const Options& o) {
  std::map<int, int> cache;
  for (auto& m : s.materials) {
    if (m.unlit || !m.baseColorTex.valid() || (m.normalTex.valid() && !o.forceNormalMap)) continue;
    int src = m.baseColorTex.image;
    auto it = cache.find(src);
    if (it == cache.end()) {
      auto p = imagePixels(s.images[src]);
      if (!p) continue;
      Image im;
      im.name = (s.images[src].name.empty() ? std::string("texture") : stemOf(s.images[src].name)) + "_normal";
      im.pixels = heightToNormal(*p, o.normalStrength, o.normalMapSize);
      im.usage = kUseNormal;
      it = cache.emplace(src, s.addImage(std::move(im))).first;
      logVerbose("generated normal map '%s' from base colour", s.images[it->second].name.c_str());
    }
    m.normalTex = {it->second, m.baseColorTex.uvSet};
    m.normalScale = 1.0f;
  }
}

// ------------------------------------------------------------------ final encoding
bool finalizeImages(Scene& s, const TextureEncodeSettings& t) {
  size_t budget = t.memoryBudget ? t.memoryBudget : physicalMemoryBytes() * 6 / 10;
  MemoryGate gate(budget);
  std::atomic<int> failures{0};
  std::atomic<size_t> before{0}, after{0};
  parallelFor(s.images.size(), [&](size_t i) {
    Image& im = s.images[i];
    int w = 0, h = 0;
    if (!imageSize(im, w, h)) {
      // unknown format (ktx2, dds ...): pass through untouched
      if (im.encoded) { im.finalData = im.encoded; im.finalMime = im.mime; im.finalExt = mimeToExt(im.mime); }
      else failures++;
      return;
    }
    int tw = w, th = h;
    if (t.maxSize > 0 && std::max(w, h) > t.maxSize) {
      float k = float(t.maxSize) / float(std::max(w, h));
      tw = std::max(1, int(std::lround(w * k))); th = std::max(1, int(std::lround(h * k)));
    }
    if (t.powerOfTwo) {
      tw = nearestPow2(tw); th = nearestPow2(th);
      if (t.maxSize > 0) { while (tw > t.maxSize) tw /= 2; while (th > t.maxSize) th /= 2; }
    }
    TexFormat fmt = t.format;
    std::string srcMime = im.encoded ? sniffMime(im.encoded->data, im.encoded->size) : "";
    if (fmt == TexFormat::WebP && std::max(tw, th) > 16383) {  // WebP hard limit
      float k = 16383.0f / float(std::max(tw, th));
      tw = std::max(1, int(tw * k)); th = std::max(1, int(th * k));
    }
    const bool resize = tw != w || th != h;
    std::string wantMime = fmt == TexFormat::WebP ? "image/webp" : fmt == TexFormat::PNG ? "image/png" : fmt == TexFormat::JPEG ? "image/jpeg" : "";
    if (fmt == TexFormat::Keep || fmt == TexFormat::Auto) {
      if (srcMime == "image/png" || srcMime == "image/jpeg" || srcMime == "image/webp") wantMime = srcMime;
      else wantMime = "image/png";
      fmt = wantMime == "image/webp" ? TexFormat::WebP : wantMime == "image/jpeg" ? TexFormat::JPEG : TexFormat::PNG;
    }
    if (!resize && im.encoded && srcMime == wantMime) {   // nothing to do: zero copy
      im.finalData = im.encoded; im.finalMime = srcMime; im.finalExt = mimeToExt(srcMime);
      before += im.encoded->size; after += im.encoded->size;
      im.pixels.reset();
      return;
    }
    size_t need = size_t(w) * h * 4 * 2 + size_t(tw) * th * 4 * 3;
    gate.acquire(need);
    std::string err;
    auto px = imagePixels(im, &err);
    if (!px) { logWarn("texture '%s': %s", im.name.c_str(), err.c_str()); failures++; gate.release(need); return; }
    std::shared_ptr<Pixels> out = resize ? resizePixels(*px, tw, th, (im.usage & kUseNormal) != 0) : px;
    std::vector<uint8_t> bytes;
    std::string mime;
    const bool normal = (im.usage & kUseNormal) != 0;
    int q = normal ? std::max(t.quality, 90) : t.quality;
    if (!encodeImage(*out, fmt, q, normal && t.losslessNormals, bytes, mime, t.effort)) {
      logWarn("texture '%s': encoding failed", im.name.c_str());
      failures++;
    } else {
      before += im.encoded ? im.encoded->size : size_t(w) * h * 4;
      after += bytes.size();
      im.finalData = Blob::fromVector(std::move(bytes));
      im.finalMime = mime;
      im.finalExt = mimeToExt(mime);
      logVerbose("texture '%s' %dx%d -> %dx%d %s (%s)", im.name.c_str(), w, h, tw, th, mime.c_str(),
                 humanBytes(double(im.finalData->size)).c_str());
    }
    im.pixels.reset();
    out.reset(); px.reset();
    gate.release(need);
  }, 1);
  if (!s.images.empty())
    logInfo("textures: %zu images, %s -> %s", s.images.size(), humanBytes(double(before)).c_str(), humanBytes(double(after)).c_str());
  return failures == 0;
}

}  // namespace rx
