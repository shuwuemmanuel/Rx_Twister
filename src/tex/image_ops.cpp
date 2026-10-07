#define STB_IMAGE_IMPLEMENTATION
#define STBI_FAILURE_USERMSG
#define STBI_MAX_DIMENSIONS (1 << 16)
#include <stb/stb_image.h>
#if defined(RX_HAVE_ZLIB)
#include <zlib.h>
static unsigned char* rxZlibCompress(unsigned char* data, int dataLen, int* outLen, int /*quality*/) {
  uLongf n = compressBound(uLong(dataLen));
  unsigned char* out = (unsigned char*)malloc(n);
  if (!out) return nullptr;
  if (compress2(out, &n, data, uLong(dataLen), 6) != Z_OK) { free(out); return nullptr; }
  *outLen = int(n);
  return out;
}
#define STBIW_ZLIB_COMPRESS rxZlibCompress
#endif
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>

#include "tex/image_ops.h"
#include "core/par.h"
#include "core/util.h"
#include <cmath>
#include <cstring>
#if defined(RX_HAVE_WEBP)
#include <webp/decode.h>
#include <webp/encode.h>
#endif

namespace rx {

std::string sniffMime(const uint8_t* d, size_t n) {
  if (n >= 8 && d[0] == 0x89 && d[1] == 'P' && d[2] == 'N' && d[3] == 'G') return "image/png";
  if (n >= 3 && d[0] == 0xFF && d[1] == 0xD8 && d[2] == 0xFF) return "image/jpeg";
  if (n >= 12 && !memcmp(d, "RIFF", 4) && !memcmp(d + 8, "WEBP", 4)) return "image/webp";
  if (n >= 12 && !memcmp(d + 1, "KTX 20", 6)) return "image/ktx2";
  if (n >= 4 && !memcmp(d, "DDS ", 4)) return "image/vnd-ms.dds";
  if (n >= 2 && d[0] == 'B' && d[1] == 'M') return "image/bmp";
  if (n >= 4 && !memcmp(d, "GIF8", 4)) return "image/gif";
  return "";
}
std::string mimeToExt(const std::string& m) {
  if (m == "image/png") return ".png";
  if (m == "image/jpeg") return ".jpg";
  if (m == "image/webp") return ".webp";
  if (m == "image/ktx2") return ".ktx2";
  if (m == "image/bmp") return ".bmp";
  return ".bin";
}

bool imageInfo(const uint8_t* d, size_t n, int& w, int& h, bool& alpha) {
#if defined(RX_HAVE_WEBP)
  if (sniffMime(d, n) == "image/webp") {
    WebPBitstreamFeatures f;
    if (WebPGetFeatures(d, n, &f) != VP8_STATUS_OK) return false;
    w = f.width; h = f.height; alpha = f.has_alpha;
    return true;
  }
#endif
  int c = 0;
  if (!stbi_info_from_memory(d, int(std::min<size_t>(n, 0x7fffffff)), &w, &h, &c)) return false;
  alpha = (c == 2 || c == 4);
  return true;
}

std::shared_ptr<Pixels> makePixels(int w, int h, uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
  auto p = std::make_shared<Pixels>();
  p->width = w; p->height = h;
  p->rgba.resize(size_t(w) * h * 4);
  for (size_t i = 0; i < size_t(w) * h; ++i) { p->rgba[i * 4] = r; p->rgba[i * 4 + 1] = g; p->rgba[i * 4 + 2] = b; p->rgba[i * 4 + 3] = a; }
  return p;
}

std::shared_ptr<Pixels> decodeImage(const uint8_t* d, size_t n, std::string* err) {
  auto px = std::make_shared<Pixels>();
  if (sniffMime(d, n) == "image/webp") {
#if defined(RX_HAVE_WEBP)
    WebPBitstreamFeatures f;
    if (WebPGetFeatures(d, n, &f) != VP8_STATUS_OK) { if (err) *err = "bad webp header"; return nullptr; }
    px->width = f.width; px->height = f.height; px->hasAlpha = f.has_alpha;
    px->rgba.resize(size_t(f.width) * f.height * 4);
    if (!WebPDecodeRGBAInto(d, n, px->rgba.data(), px->rgba.size(), f.width * 4)) { if (err) *err = "webp decode failed"; return nullptr; }
    return px;
#else
    if (err) *err = "built without libwebp";
    return nullptr;
#endif
  }
  int w, h, c;
  unsigned char* data = stbi_load_from_memory(d, int(std::min<size_t>(n, 0x7fffffff)), &w, &h, &c, 4);
  if (!data) { if (err) *err = stbi_failure_reason() ? stbi_failure_reason() : "decode failed"; return nullptr; }
  px->width = w; px->height = h;
  px->rgba.assign(data, data + size_t(w) * h * 4);
  stbi_image_free(data);
  px->hasAlpha = (c == 2 || c == 4);
  if (px->hasAlpha) {  // verify alpha is really used
    bool used = false;
    for (size_t i = 3; i < px->rgba.size(); i += 4) if (px->rgba[i] != 255) { used = true; break; }
    px->hasAlpha = used;
  }
  return px;
}

std::shared_ptr<Pixels> decodeImageFile(const std::string& path, std::string* err) {
  auto b = mapFile(path, err);
  if (!b) return nullptr;
  return decodeImage(b->data, b->size, err);
}

static void pngWrite(void* ctx, void* data, int size) {
  auto* v = static_cast<std::vector<uint8_t>*>(ctx);
  v->insert(v->end(), (uint8_t*)data, (uint8_t*)data + size);
}

bool encodeImage(const Pixels& px, TexFormat fmt, int quality, bool lossless, std::vector<uint8_t>& out,
                 std::string& mime, int effort) {
  out.clear();
#if defined(RX_HAVE_WEBP)
  if (fmt == TexFormat::WebP && px.width <= 16383 && px.height <= 16383) {
    WebPConfig cfg;
    if (!WebPConfigPreset(&cfg, WEBP_PRESET_DEFAULT, float(quality))) return false;
    cfg.lossless = lossless ? 1 : 0;
    cfg.method = std::clamp(effort, 0, 6);
    if (lossless) { cfg.quality = 75; cfg.exact = 0; }
    else { cfg.use_sharp_yuv = 0; cfg.alpha_quality = 100; }
    cfg.thread_level = 1;
    if (!WebPValidateConfig(&cfg)) return false;
    WebPPicture pic;
    if (!WebPPictureInit(&pic)) return false;
    pic.width = px.width; pic.height = px.height; pic.use_argb = lossless ? 1 : 0;
    bool ok = WebPPictureImportRGBA(&pic, px.rgba.data(), px.width * 4) != 0;
    WebPMemoryWriter mw; WebPMemoryWriterInit(&mw);
    pic.writer = WebPMemoryWrite; pic.custom_ptr = &mw;
    ok = ok && WebPEncode(&cfg, &pic);
    WebPPictureFree(&pic);
    if (ok) out.assign(mw.mem, mw.mem + mw.size);
    WebPMemoryWriterClear(&mw);
    if (ok) { mime = "image/webp"; return true; }
    return false;
  }
#endif
  bool useJpeg = fmt == TexFormat::JPEG && !px.hasAlpha;
  if (useJpeg) {
    std::vector<uint8_t> rgb(size_t(px.width) * px.height * 3);
    for (size_t i = 0; i < size_t(px.width) * px.height; ++i) memcpy(&rgb[i * 3], &px.rgba[i * 4], 3);
    out.reserve(rgb.size() / 6);
    if (!stbi_write_jpg_to_func(pngWrite, &out, px.width, px.height, 3, rgb.data(), std::clamp(quality, 1, 100))) return false;
    mime = "image/jpeg";
    return true;
  }
  // PNG (also the fallback for WebP when unavailable / too large, and for JPEG + alpha)
  const int comps = px.hasAlpha ? 4 : 3;
  std::vector<uint8_t> tmp;
  const uint8_t* src = px.rgba.data();
  if (comps == 3) {
    tmp.resize(size_t(px.width) * px.height * 3);
    for (size_t i = 0; i < size_t(px.width) * px.height; ++i) memcpy(&tmp[i * 3], &px.rgba[i * 4], 3);
    src = tmp.data();
  }
  if (!stbi_write_png_to_func(pngWrite, &out, px.width, px.height, comps, src, px.width * comps)) return false;
  mime = "image/png";
  return true;
}

int nearestPow2(int v) {
  if (v <= 1) return 1;
  int lo = 1;
  while (lo * 2 <= v) lo *= 2;
  int hi = lo * 2;
  return (v - lo) < (hi - v) ? lo : hi;
}

void renormalizeNormalMap(Pixels& px) {
  parallelRanges(size_t(px.width) * px.height, 1 << 16, [&](size_t b, size_t e) {
    for (size_t i = b; i < e; ++i) {
      uint8_t* p = &px.rgba[i * 4];
      float x = p[0] / 127.5f - 1, y = p[1] / 127.5f - 1, z = p[2] / 127.5f - 1;
      float l = std::sqrt(x * x + y * y + z * z);
      if (l < 1e-6f) { x = 0; y = 0; z = 1; l = 1; }
      p[0] = uint8_t(std::clamp((x / l * 0.5f + 0.5f) * 255.0f + 0.5f, 0.0f, 255.0f));
      p[1] = uint8_t(std::clamp((y / l * 0.5f + 0.5f) * 255.0f + 0.5f, 0.0f, 255.0f));
      p[2] = uint8_t(std::clamp((z / l * 0.5f + 0.5f) * 255.0f + 0.5f, 0.0f, 255.0f));
    }
  });
}

namespace {
struct Taps { std::vector<int> start, count; std::vector<float> w; int maxTaps = 0; };
inline float sinc(float x) { if (std::fabs(x) < 1e-6f) return 1.0f; float t = kPi * x; return std::sin(t) / t; }
inline float lanczos3(float x) { x = std::fabs(x); return x >= 3.0f ? 0.0f : sinc(x) * sinc(x / 3.0f); }

Taps buildTaps(int srcN, int dstN) {
  Taps t;
  const float scale = float(srcN) / float(dstN);
  const float fscale = std::max(1.0f, scale);
  const float radius = 3.0f * fscale;
  t.maxTaps = int(std::ceil(radius * 2)) + 2;
  t.start.resize(dstN); t.count.resize(dstN); t.w.assign(size_t(dstN) * t.maxTaps, 0.0f);
  for (int i = 0; i < dstN; ++i) {
    float center = (i + 0.5f) * scale;
    int lo = std::max(0, int(std::floor(center - radius)));
    int hi = std::min(srcN - 1, int(std::ceil(center + radius)));
    float sum = 0;
    float* w = &t.w[size_t(i) * t.maxTaps];
    int cnt = 0;
    for (int s = lo; s <= hi && cnt < t.maxTaps; ++s, ++cnt) {
      float v = lanczos3((s + 0.5f - center) / fscale);
      w[cnt] = v; sum += v;
    }
    if (sum != 0) for (int k = 0; k < cnt; ++k) w[k] /= sum;
    t.start[i] = lo; t.count[i] = cnt;
  }
  return t;
}
}  // namespace

std::shared_ptr<Pixels> resizePixels(const Pixels& src, int dw, int dh, bool isNormal) {
  auto dst = std::make_shared<Pixels>();
  dst->width = dw; dst->height = dh; dst->hasAlpha = src.hasAlpha;
  if (dw == src.width && dh == src.height) { dst->rgba = src.rgba; return dst; }
  dst->rgba.resize(size_t(dw) * dh * 4);
  const int sw = src.width, sh = src.height;
  const bool alpha = src.hasAlpha;
  Taps tx = buildTaps(sw, dw), ty = buildTaps(sh, dh);
  // Horizontal pass -> 16 bit intermediate (premultiplied when alpha is present).
  std::vector<uint16_t> mid(size_t(sh) * dw * 4);
  parallelFor(size_t(sh), [&](size_t y) {
    const uint8_t* row = &src.rgba[y * sw * 4];
    uint16_t* o = &mid[y * dw * 4];
    for (int x = 0; x < dw; ++x) {
      const float* w = &tx.w[size_t(x) * tx.maxTaps];
      float r = 0, g = 0, b = 0, a = 0;
      for (int k = 0; k < tx.count[x]; ++k) {
        const uint8_t* p = row + size_t(tx.start[x] + k) * 4;
        float pa = alpha ? p[3] / 255.0f : 1.0f;
        float wk = w[k];
        r += wk * p[0] * pa; g += wk * p[1] * pa; b += wk * p[2] * pa; a += wk * p[3];
      }
      auto q = [](float v, float hi) { return uint16_t(std::clamp(v, 0.0f, hi) * (65535.0f / hi) + 0.5f); };
      o[x * 4] = q(r, 255); o[x * 4 + 1] = q(g, 255); o[x * 4 + 2] = q(b, 255); o[x * 4 + 3] = q(a, 255);
    }
  }, 4);
  // Vertical pass
  parallelFor(size_t(dh), [&](size_t y) {
    const float* w = &ty.w[y * ty.maxTaps];
    uint8_t* o = &dst->rgba[y * dw * 4];
    std::vector<float> acc(size_t(dw) * 4, 0.0f);
    for (int k = 0; k < ty.count[y]; ++k) {
      const uint16_t* m = &mid[size_t(ty.start[y] + k) * dw * 4];
      float wk = w[k] * (255.0f / 65535.0f);
      for (int i = 0; i < dw * 4; ++i) acc[i] += wk * m[i];
    }
    for (int x = 0; x < dw; ++x) {
      float a = std::clamp(acc[x * 4 + 3], 0.0f, 255.0f);
      float inv = (alpha && a > 0.5f) ? 255.0f / a : 1.0f;
      for (int c = 0; c < 3; ++c) o[x * 4 + c] = uint8_t(std::clamp(acc[x * 4 + c] * inv, 0.0f, 255.0f) + 0.5f);
      o[x * 4 + 3] = alpha ? uint8_t(a + 0.5f) : 255;
    }
  }, 4);
  if (isNormal) renormalizeNormalMap(*dst);
  return dst;
}

}  // namespace rx
