// Rx Twister - small math helpers (float32, column-major matrices).
#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>

namespace rx {

struct Vec2 { float x = 0, y = 0; };
struct Vec3 { float x = 0, y = 0, z = 0; };
struct Vec4 { float x = 0, y = 0, z = 0, w = 0; };

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, float s) { return {a.x * s, a.y * s}; }

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(float s, Vec3 a) { return a * s; }
inline Vec3& operator+=(Vec3& a, Vec3 b) { a.x += b.x; a.y += b.y; a.z += b.z; return a; }
inline Vec3& operator-=(Vec3& a, Vec3 b) { a.x -= b.x; a.y -= b.y; a.z -= b.z; return a; }
inline Vec3& operator*=(Vec3& a, float s) { a.x *= s; a.y *= s; a.z *= s; return a; }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalize(Vec3 a) {
  float l = length(a);
  return l > 1e-20f ? a * (1.0f / l) : Vec3{0, 0, 1};
}
inline Vec3 lerp(Vec3 a, Vec3 b, float t) { return a + (b - a) * t; }
inline Vec2 lerp(Vec2 a, Vec2 b, float t) { return a + (b - a) * t; }

// Column-major 4x4 matrix (glTF layout).
struct Mat4 {
  float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  static Mat4 identity() { return Mat4{}; }
  bool isIdentity() const {
    static const float id[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    for (int i = 0; i < 16; ++i) if (std::fabs(m[i] - id[i]) > 1e-7f) return false;
    return true;
  }
  Vec3 point(Vec3 p) const {
    return {m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12], m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13],
            m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14]};
  }
  Vec3 vector(Vec3 p) const {
    return {m[0] * p.x + m[4] * p.y + m[8] * p.z, m[1] * p.x + m[5] * p.y + m[9] * p.z,
            m[2] * p.x + m[6] * p.y + m[10] * p.z};
  }
  float det3() const {
    return m[0] * (m[5] * m[10] - m[9] * m[6]) - m[4] * (m[1] * m[10] - m[9] * m[2]) +
           m[8] * (m[1] * m[6] - m[5] * m[2]);
  }
  static Mat4 translation(Vec3 t) { Mat4 r; r.m[12] = t.x; r.m[13] = t.y; r.m[14] = t.z; return r; }
  static Mat4 scale(Vec3 s) { Mat4 r; r.m[0] = s.x; r.m[5] = s.y; r.m[10] = s.z; return r; }
  static Mat4 fromQuat(float x, float y, float z, float w) {
    Mat4 r;
    r.m[0] = 1 - 2 * (y * y + z * z); r.m[1] = 2 * (x * y + z * w);     r.m[2] = 2 * (x * z - y * w);
    r.m[4] = 2 * (x * y - z * w);     r.m[5] = 1 - 2 * (x * x + z * z); r.m[6] = 2 * (y * z + x * w);
    r.m[8] = 2 * (x * z + y * w);     r.m[9] = 2 * (y * z - x * w);     r.m[10] = 1 - 2 * (x * x + y * y);
    return r;
  }
  Mat4 operator*(const Mat4& b) const {
    Mat4 r;
    for (int c = 0; c < 4; ++c)
      for (int rI = 0; rI < 4; ++rI) {
        float s = 0;
        for (int k = 0; k < 4; ++k) s += m[k * 4 + rI] * b.m[c * 4 + k];
        r.m[c * 4 + rI] = s;
      }
    return r;
  }
};

struct Aabb {
  Vec3 lo{1e30f, 1e30f, 1e30f}, hi{-1e30f, -1e30f, -1e30f};
  void add(Vec3 p) {
    lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
    hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
  }
  void add(const Aabb& b) { add(b.lo); add(b.hi); }
  bool valid() const { return lo.x <= hi.x; }
  Vec3 size() const { return hi - lo; }
};

}  // namespace rx
