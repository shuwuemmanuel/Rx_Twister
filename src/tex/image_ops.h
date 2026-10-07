// Image decode / encode / resize.
#pragma once
#include <memory>
#include <string>
#include <vector>
#include "rx/scene.h"

namespace rx {

std::string sniffMime(const uint8_t* d, size_t n);                 // "image/png", "image/jpeg", "image/webp", ""
std::string mimeToExt(const std::string& mime);                    // ".png" ...
bool imageInfo(const uint8_t* d, size_t n, int& w, int& h, bool& hasAlpha);
std::shared_ptr<Pixels> decodeImage(const uint8_t* d, size_t n, std::string* err = nullptr);
std::shared_ptr<Pixels> decodeImageFile(const std::string& path, std::string* err = nullptr);

// fmt must be WebP / PNG / JPEG. JPEG falls back to PNG when alpha is present.
// Returns the mime actually produced.
bool encodeImage(const Pixels& px, TexFormat fmt, int quality, bool lossless, std::vector<uint8_t>& out,
                 std::string& mimeOut, int effort = 2);

// Lanczos3 resampling in premultiplied space; renormalises vectors when isNormalMap.
std::shared_ptr<Pixels> resizePixels(const Pixels& src, int w, int h, bool isNormalMap);

std::shared_ptr<Pixels> makePixels(int w, int h, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255);
int nearestPow2(int v);
void renormalizeNormalMap(Pixels& px);

}  // namespace rx
