// Wavefront OBJ / MTL. The reader parses the memory mapped file in parallel chunks, then builds
// one mesh per (object, material) with a parallel per-mesh vertex dedupe.
#include "io/io.h"
#include "core/par.h"
#include "core/util.h"
#include "tex/textures.h"
#include "geom/mesh_ops.h"
#include <charconv>
#include <cstring>
#include <map>
#include <climits>

namespace rx {

namespace {

inline const char* skipWs(const char* p, const char* e) { while (p < e && (*p == ' ' || *p == '\t')) ++p; return p; }
inline const char* parseFloat(const char* p, const char* e, float& v) {
  p = skipWs(p, e);
  if (p < e && *p == '+') ++p;
  auto r = std::from_chars(p, e, v);
  if (r.ec != std::errc()) { v = 0; while (p < e && *p != ' ' && *p != '\t' && *p != '\n' && *p != '\r') ++p; return p; }
  return r.ptr;
}
inline const char* parseInt(const char* p, const char* e, int64_t& v) {
  auto r = std::from_chars(p, e, v);
  if (r.ec != std::errc()) { v = 0; return p; }
  return r.ptr;
}

struct Corner { int32_t v, t, n; };  // 12 bytes: OBJ files stay below 2^31 elements
struct StateEvent { size_t face; int kind; std::string name; };  // kind 0 = object, 1 = material
struct Chunk {
  std::vector<float> v, vt, vn, col;
  std::vector<Corner> corners;     // 3 per triangle
  std::vector<StateEvent> events;
  std::vector<std::string> mtllibs;
  bool hasColors = false;
};

void parseChunk(const char* b, const char* e, Chunk& c) {
  const char* p = b;
  const char* released = b;
  std::vector<Corner> poly;
  size_t lv = 0, lt = 0, ln = 0;  // local counts for negative indices
  while (p < e) {
    const char* le = static_cast<const char*>(memchr(p, '\n', size_t(e - p)));
    if (!le) le = e;
    const char* q = skipWs(p, le);
    if (q + 1 < le) {
      if (q[0] == 'v' && (q[1] == ' ' || q[1] == '\t')) {
        float x, y, z; q = parseFloat(q + 2, le, x); q = parseFloat(q, le, y); q = parseFloat(q, le, z);
        c.v.push_back(x); c.v.push_back(y); c.v.push_back(z);
        q = skipWs(q, le);
        if (q < le && *q != '\r' && *q != '#') {
          float r, g, bl; q = parseFloat(q, le, r); q = parseFloat(q, le, g); q = parseFloat(q, le, bl);
          c.col.resize(lv * 3, 1.0f);
          c.col.push_back(r); c.col.push_back(g); c.col.push_back(bl);
          c.hasColors = true;
        }
        ++lv;
      } else if (q[0] == 'v' && q[1] == 't') {
        float u = 0, v = 0; q = parseFloat(q + 2, le, u);
        const char* q2 = skipWs(q, le);
        if (q2 < le && *q2 != '\r') parseFloat(q2, le, v);
        c.vt.push_back(u); c.vt.push_back(v); ++lt;
      } else if (q[0] == 'v' && q[1] == 'n') {
        float x, y, z; q = parseFloat(q + 2, le, x); q = parseFloat(q, le, y); q = parseFloat(q, le, z);
        c.vn.push_back(x); c.vn.push_back(y); c.vn.push_back(z); ++ln;
      } else if (q[0] == 'f' && (q[1] == ' ' || q[1] == '\t')) {
        poly.clear();
        q += 2;
        while (true) {
          q = skipWs(q, le);
          if (q >= le || *q == '\r' || *q == '#') break;
          Corner k{0, 0, 0};
          int64_t x;
          const char* n = parseInt(q, le, x);
          if (n == q) break;
          k.v = int32_t(x < 0 ? -(int64_t(lv) + x + 1) : x);   // negative => chunk local (encoded as -(local+1))
          q = n;
          if (q < le && *q == '/') {
            ++q;
            if (q < le && *q != '/') { q = parseInt(q, le, x); k.t = int32_t(x < 0 ? -(int64_t(lt) + x + 1) : x); }
            if (q < le && *q == '/') { ++q; q = parseInt(q, le, x); k.n = int32_t(x < 0 ? -(int64_t(ln) + x + 1) : x); }
          }
          poly.push_back(k);
          while (q < le && *q != ' ' && *q != '\t' && *q != '\r') ++q;
        }
        for (size_t i = 2; i < poly.size(); ++i) { c.corners.push_back(poly[0]); c.corners.push_back(poly[i - 1]); c.corners.push_back(poly[i]); }
      } else if (!strncmp(q, "usemtl", 6) || ((q[0] == 'o' || q[0] == 'g') && (q[1] == ' ' || q[1] == '\t')) || !strncmp(q, "mtllib", 6)) {
        bool mtl = q[0] == 'u', lib = q[0] == 'm';
        const char* s = skipWs(q + (mtl || lib ? 6 : 1), le);
        const char* t = le;
        while (t > s && (t[-1] == '\r' || t[-1] == ' ' || t[-1] == '\t')) --t;
        std::string name(s, t);
        if (lib) c.mtllibs.push_back(name);
        else c.events.push_back({c.corners.size() / 3, mtl ? 1 : (q[0] == 'o' ? 0 : 2), name});
      }
    }
    p = le + 1;
    if (size_t(p - released) > (size_t(32) << 20)) { releaseMappedRange(released, size_t(p - released)); released = p; }
  }
  if (c.hasColors) c.col.resize(lv * 3, 1.0f);
}

// Parse "map_Kd -o 1 1 -bm 0.5 my file.png" -> options + path
std::string mapPath(const std::string& rest, float* bm = nullptr) {
  std::vector<std::string> tok;
  size_t i = 0;
  while (i < rest.size()) {
    while (i < rest.size() && (rest[i] == ' ' || rest[i] == '\t')) ++i;
    size_t j = i;
    while (j < rest.size() && rest[j] != ' ' && rest[j] != '\t') ++j;
    if (j > i) tok.push_back(rest.substr(i, j - i));
    i = j;
  }
  static const std::map<std::string, int> argc = {{"-bm", 1}, {"-blendu", 1}, {"-blendv", 1}, {"-boost", 1}, {"-cc", 1},
      {"-clamp", 1}, {"-imfchan", 1}, {"-mm", 2}, {"-o", 3}, {"-s", 3}, {"-t", 3}, {"-texres", 1}, {"-type", 1}};
  size_t k = 0;
  while (k < tok.size() && tok[k].size() > 1 && tok[k][0] == '-' && !std::isdigit((unsigned char)tok[k][1])) {
    auto it = argc.find(tok[k]);
    int n = it == argc.end() ? 1 : it->second;
    if (tok[k] == "-bm" && bm && k + 1 < tok.size()) *bm = std::strtof(tok[k + 1].c_str(), nullptr);
    ++k;
    for (int a = 0; a < n && k < tok.size(); ++a) {
      if (n == 3 && a > 0 && !(std::isdigit((unsigned char)tok[k][0]) || tok[k][0] == '-' || tok[k][0] == '.')) break;
      ++k;
    }
  }
  std::string r;
  for (; k < tok.size(); ++k) { if (!r.empty()) r += ' '; r += tok[k]; }
  return r;
}

void readMtl(const std::string& path, Scene& s, std::map<std::string, int>& byName) {
  auto b = mapFile(path);
  if (!b) { logWarn("OBJ: material library %s not found", path.c_str()); return; }
  std::string dir = pathToUtf8(pathFromUtf8(path).parent_path());
  std::string text(reinterpret_cast<const char*>(b->data), b->size);
  Material* m = nullptr;
  float ns = -1;
  bool pbrRough = false;
  auto finish = [&]() {
    if (m && !pbrRough && ns >= 0) m->roughness = std::clamp(std::sqrt(2.0f / (ns + 2.0f)), 0.0f, 1.0f);
  };
  size_t pos = 0;
  while (pos < text.size()) {
    size_t e = text.find('\n', pos);
    if (e == std::string::npos) e = text.size();
    std::string line = text.substr(pos, e - pos);
    pos = e + 1;
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\t')) line.pop_back();
    size_t a = line.find_first_not_of(" \t");
    if (a == std::string::npos || line[a] == '#') continue;
    size_t sp = line.find_first_of(" \t", a);
    std::string key = toLower(line.substr(a, sp == std::string::npos ? std::string::npos : sp - a));
    std::string rest = sp == std::string::npos ? "" : line.substr(line.find_first_not_of(" \t", sp) == std::string::npos ? line.size() : line.find_first_not_of(" \t", sp));
    auto f3 = [&](Vec3& v) { std::sscanf(rest.c_str(), "%f %f %f", &v.x, &v.y, &v.z); };
    if (key == "newmtl") {
      finish();
      Material nm; nm.name = rest; nm.roughness = 0.8f; ns = -1; pbrRough = false;
      byName[rest] = s.addMaterial(std::move(nm));
      m = &s.materials.back();
      continue;
    }
    if (!m) continue;
    auto tex = [&](TextureSlot& t, float* bm = nullptr) {
      std::string p = resolveTexturePath(dir, mapPath(rest, bm));
      if (p.empty()) { logWarn("OBJ: texture '%s' not found", mapPath(rest).c_str()); return; }
      int i = loadTextureFile(s, p);
      if (i >= 0) t = {i, 0};
    };
    if (key == "kd") { Vec3 c; f3(c); m->baseColor = {c.x, c.y, c.z, m->baseColor.w}; }
    else if (key == "d") m->baseColor.w = std::strtof(rest.c_str(), nullptr);
    else if (key == "tr") m->baseColor.w = 1.0f - std::strtof(rest.c_str(), nullptr);
    else if (key == "ns") ns = std::strtof(rest.c_str(), nullptr);
    else if (key == "ke") f3(m->emissive);
    else if (key == "pr") { m->roughness = std::strtof(rest.c_str(), nullptr); pbrRough = true; }
    else if (key == "pm") m->metallic = std::strtof(rest.c_str(), nullptr);
    else if (key == "map_kd") tex(m->baseColorTex);
    else if (key == "map_bump" || key == "bump" || key == "norm" || key == "map_normal") tex(m->normalTex, &m->normalScale);
    else if (key == "map_ke" || key == "map_emissive") { tex(m->emissiveTex); if (m->emissive.x + m->emissive.y + m->emissive.z == 0) m->emissive = {1, 1, 1}; }
    else if (key == "map_pr") tex(m->roughnessTex);
    else if (key == "map_pm") tex(m->metallicTex);
    else if (key == "map_ao" || key == "map_ka") { if (key == "map_ao") tex(m->occlusionTex); }
    else if (key == "map_d") { if (m->alphaMode == AlphaMode::Opaque) m->alphaMode = AlphaMode::Mask; }
    else if (key == "map_orm" || key == "map_rma") { tex(m->metalRoughTex); m->occlusionTex = m->metalRoughTex; m->metallic = m->roughness = 1; pbrRough = true; }
  }
  finish();
  for (auto& mm : s.materials) if (mm.baseColor.w < 0.999f && mm.alphaMode == AlphaMode::Opaque) mm.alphaMode = AlphaMode::Blend;
}

inline uint64_t cornerHash(const Corner& k) {
  uint64_t h = uint64_t(uint32_t(k.v)) * 0x9E3779B97F4A7C15ull ^ uint64_t(uint32_t(k.t)) * 0xC2B2AE3D27D4EB4Full ^
               uint64_t(uint32_t(k.n)) * 0x165667B19E3779F9ull;
  return h ^ (h >> 31);
}

// Open addressing corner -> vertex table (much faster and leaner than std::unordered_map).
class CornerMap {
 public:
  explicit CornerMap(size_t expected) { rehash(capFor(expected)); }
  // returns (value, inserted)
  std::pair<uint32_t, bool> insert(const Corner& k, uint64_t h, uint32_t v) {
    if ((size_ + 1) * 10 > keys_.size() * 7) grow();
    size_t i = size_t(h) & mask_;
    for (;;) {
      Corner& c = keys_[i];
      if (c.v == INT32_MIN) { c = k; vals_[i] = v; ++size_; return {v, true}; }
      if (c.v == k.v && c.t == k.t && c.n == k.n) return {vals_[i], false};
      i = (i + 1) & mask_;
    }
  }
 private:
  static size_t capFor(size_t n) { size_t c = 16; while (c * 7 < n * 10) c <<= 1; return c; }
  void rehash(size_t cap) {
    std::vector<Corner> ok; std::vector<uint32_t> ov;
    ok.swap(keys_); ov.swap(vals_);
    keys_.assign(cap, Corner{INT32_MIN, 0, 0});
    vals_.resize(cap);
    mask_ = cap - 1;
    size_ = 0;
    for (size_t i = 0; i < ok.size(); ++i) if (ok[i].v != INT32_MIN) insert(ok[i], cornerHash(ok[i]), ov[i]);
  }
  void grow() { rehash(keys_.size() * 2); }
  std::vector<Corner> keys_;
  std::vector<uint32_t> vals_;
  size_t mask_ = 0, size_ = 0;
};

}  // namespace

bool readObj(const std::string& path, Scene& s, std::string& err) {
  auto file = mapFile(path, &err);
  if (!file) return false;
  const char* d = reinterpret_cast<const char*>(file->data);
  const size_t n = file->size;
  // chunk boundaries at line starts
  size_t nChunks = std::max<size_t>(1, std::min<size_t>(size_t(threadLimit()) * 4, n / (4 << 20) + 1));
  std::vector<size_t> bounds{0};
  for (size_t i = 1; i < nChunks; ++i) {
    size_t p = n * i / nChunks;
    const char* nl = static_cast<const char*>(memchr(d + p, '\n', n - p));
    p = nl ? size_t(nl - d) + 1 : n;
    if (p > bounds.back()) bounds.push_back(p);
  }
  bounds.push_back(n);
  std::vector<Chunk> chunks(bounds.size() - 1);
  parallelFor(chunks.size(), [&](size_t i) { parseChunk(d + bounds[i], d + bounds[i + 1], chunks[i]); }, 1);
  file.reset();  // unmap: the text is no longer needed (keeps peak memory down on huge files)
  logVerbose("OBJ parsed: %zu chunks", chunks.size());

  // global arrays + prefix offsets
  size_t nv = 0, nt = 0, nn = 0;
  bool colors = false;
  std::vector<size_t> ov(chunks.size()), ot(chunks.size()), on(chunks.size());
  for (size_t i = 0; i < chunks.size(); ++i) {
    ov[i] = nv; ot[i] = nt; on[i] = nn;
    nv += chunks[i].v.size() / 3; nt += chunks[i].vt.size() / 2; nn += chunks[i].vn.size() / 3;
    colors |= chunks[i].hasColors;
  }
  std::vector<Vec3> V(nv), N(nn);
  std::vector<Vec2> T(nt);
  std::vector<Color8> C(colors ? nv : 0);
  parallelFor(chunks.size(), [&](size_t i) {
    auto& c = chunks[i];
    if (!c.v.empty()) memcpy(&V[ov[i]], c.v.data(), c.v.size() * 4);
    if (!c.vn.empty()) memcpy(&N[on[i]], c.vn.data(), c.vn.size() * 4);
    for (size_t k = 0; k < c.vt.size() / 2; ++k) T[ot[i] + k] = {c.vt[k * 2], 1.0f - c.vt[k * 2 + 1]};
    if (colors) {
      size_t cnt = c.v.size() / 3;
      for (size_t k = 0; k < cnt; ++k) {
        auto q = [](float x) { return uint8_t(std::clamp(x, 0.0f, 1.0f) * 255 + 0.5f); };
        C[ov[i] + k] = c.hasColors ? Color8{q(c.col[k * 3]), q(c.col[k * 3 + 1]), q(c.col[k * 3 + 2]), 255} : Color8{255, 255, 255, 255};
      }
      c.col.clear(); c.col.shrink_to_fit();
    }
    c.v.clear(); c.v.shrink_to_fit(); c.vn.clear(); c.vn.shrink_to_fit(); c.vt.clear(); c.vt.shrink_to_fit();
  }, 1);

  // materials
  std::map<std::string, int> matByName;
  fs::path dir = pathFromUtf8(path).parent_path();
  for (auto& c : chunks)
    for (auto& lib : c.mtllibs) {
      std::string p = resolveTexturePath(pathToUtf8(dir), lib);
      if (p.empty()) p = pathToUtf8(dir / pathFromUtf8(lib));
      readMtl(p, s, matByName);
    }
  if (chunks.size() && chunks[0].mtllibs.empty()) {
    bool any = false;
    for (auto& c : chunks) any |= !c.mtllibs.empty();
    fs::path guess = pathFromUtf8(path).replace_extension(".mtl");
    std::error_code ec;
    if (!any && fs::exists(guess, ec)) readMtl(pathToUtf8(guess), s, matByName);
  }

  // resolve indices to global 0-based and assign triangles to (object, material) groups
  struct Group { std::string object; int material; std::vector<std::pair<size_t, size_t>> ranges; size_t tris = 0; };
  std::vector<Group> groups;
  std::map<std::pair<std::string, int>, size_t> groupIdx;
  std::string obj = stemOf(path), grp;
  int mat = -1;
  bool sawObject = false;
  for (size_t ci = 0; ci < chunks.size(); ++ci) {
    auto& c = chunks[ci];
    auto fix = [](int32_t x, size_t off) -> int32_t { return int32_t(x > 0 ? int64_t(x) - 1 : x < 0 ? int64_t(off) + (-int64_t(x) - 1) : -1); };
    parallelRanges(c.corners.size(), 1 << 16, [&](size_t b, size_t e) {
      for (size_t i = b; i < e; ++i) {
        Corner& k = c.corners[i];
        k.v = fix(k.v, ov[ci]); k.t = fix(k.t, ot[ci]); k.n = fix(k.n, on[ci]);
      }
    });
    size_t ev = 0, tris = c.corners.size() / 3, start = 0;
    auto flush = [&](size_t end) {
      if (end <= start) return;
      std::string name = sawObject ? obj : (grp.empty() ? obj : grp);
      auto key = std::make_pair(name, mat);
      auto it = groupIdx.find(key);
      if (it == groupIdx.end()) { it = groupIdx.emplace(key, groups.size()).first; groups.push_back({name, mat, {}, 0}); }
      groups[it->second].ranges.push_back({ci, start * 3});
      groups[it->second].ranges.push_back({ci, end * 3});
      groups[it->second].tris += end - start;
      start = end;
    };
    while (ev <= c.events.size()) {
      size_t at = ev < c.events.size() ? c.events[ev].face : tris;
      flush(at);
      if (ev == c.events.size()) break;
      const auto& e = c.events[ev++];
      if (e.kind == 1) { auto it = matByName.find(e.name); mat = it == matByName.end() ? -1 : it->second; if (it == matByName.end() && !e.name.empty()) { Material m; m.name = e.name; mat = matByName[e.name] = s.addMaterial(std::move(m)); } }
      else if (e.kind == 0) { obj = e.name; sawObject = true; }
      else grp = e.name;
    }
  }

  // build meshes (dedupe corners). Big groups are deduped by all threads at once: every thread owns
  // the keys of one hash partition, so no locking is needed.
  std::vector<Mesh> meshes(groups.size());
  const bool hasT = nt > 0, hasN = nn > 0;
  auto buildGroup = [&](size_t gi, size_t parts) {
    const Group& g = groups[gi];
    Mesh& m = meshes[gi];
    m.name = g.object;
    m.material = g.material;
    const size_t nc = g.tris * 3;
    m.indices.resize(nc);
    // corner offsets of each range inside the group
    std::vector<size_t> rangeStart;
    for (size_t r = 0, acc = 0; r < g.ranges.size(); r += 2) { rangeStart.push_back(acc); acc += g.ranges[r + 1].second - g.ranges[r].second; }
    auto clean = [&](Corner k) {
      if (k.v < 0 || size_t(k.v) >= nv) k.v = 0;
      if (k.t < 0 || size_t(k.t) >= nt) k.t = -1;
      if (k.n < 0 || size_t(k.n) >= nn) k.n = -1;
      return k;
    };
    std::vector<std::vector<Corner>> uniq(parts);
    std::vector<char> missingN(parts, 0);
    const size_t expect = std::max<size_t>(16, std::min(nc, std::max({nv, nt, nn}) * 5 / 4) / parts);
    parallelFor(parts, [&](size_t p) {
      CornerMap map(expect);
      auto& u = uniq[p];
      for (size_t r = 0; r < g.ranges.size(); r += 2) {
        const auto& corners = chunks[g.ranges[r].first].corners;
        size_t base = rangeStart[r / 2] - g.ranges[r].second;
        for (size_t i = g.ranges[r].second; i < g.ranges[r + 1].second; ++i) {
          Corner k = clean(corners[i]);
          uint64_t h = cornerHash(k);
          if (parts > 1 && (h >> 40) % parts != p) continue;
          auto [id, inserted] = map.insert(k, h, uint32_t(u.size()));
          if (inserted) { u.push_back(k); if (k.n < 0) missingN[p] = 1; }
          m.indices[base + i] = id;
        }
      }
    }, 1);
    std::vector<size_t> off(parts + 1, 0);
    for (size_t p = 0; p < parts; ++p) off[p + 1] = off[p] + uniq[p].size();
    const size_t vc = off[parts];
    bool useN = hasN;
    for (char c : missingN) if (c) useN = false;
    m.positions.resize(vc);
    if (hasT) m.uv0.resize(vc);
    if (useN) m.normals.resize(vc);
    if (colors) m.colors.resize(vc);
    parallelFor(parts, [&](size_t p) {
      for (size_t i = 0; i < uniq[p].size(); ++i) {
        const Corner& k = uniq[p][i];
        size_t o = off[p] + i;
        m.positions[o] = V[k.v];
        if (hasT) m.uv0[o] = k.t >= 0 ? T[k.t] : Vec2{};
        if (useN) m.normals[o] = N[k.n];
        if (colors) m.colors[o] = C[k.v];
      }
      std::vector<Corner>().swap(uniq[p]);
    }, 1);
    if (parts > 1) {
      parallelRanges(g.ranges.size() / 2, 1, [&](size_t b, size_t e) {
        for (size_t r = b; r < e; ++r) {
          const auto& corners = chunks[g.ranges[r * 2].first].corners;
          size_t base = rangeStart[r] - g.ranges[r * 2].second;
          for (size_t i = g.ranges[r * 2].second; i < g.ranges[r * 2 + 1].second; ++i)
            m.indices[base + i] += uint32_t(off[(cornerHash(clean(corners[i])) >> 40) % parts]);
        }
      });
    }
    bool anyT = false;
    for (auto& t : m.uv0) if (t.x != 0 || t.y != 1) { anyT = true; break; }
    if (!anyT) m.uv0.clear();
  };
  // small groups: one thread each; large groups: all threads on one group
  std::vector<size_t> small, large;
  for (size_t gi = 0; gi < groups.size(); ++gi) (groups[gi].tris * 3 > (size_t(4) << 20) ? large : small).push_back(gi);
  parallelFor(small.size(), [&](size_t i) { buildGroup(small[i], 1); }, 1);
  for (size_t gi : large) buildGroup(gi, size_t(threadLimit()));
  logVerbose("OBJ meshes built");
  for (auto& m : meshes) {
    if (m.indices.empty()) continue;
    Node n; n.name = m.name; n.meshes = {int(s.meshes.size())};
    s.roots.push_back(int(s.nodes.size()));
    s.nodes.push_back(n);
    s.meshes.push_back(std::move(m));
  }
  s.sourceFormat = "obj";
  return true;
}

// ---------------------------------------------------------------------------------- writer
namespace {
inline char* putF(char* p, float v) {
  if (std::fabs(v) < 1e-30f) v = 0;
  auto r = std::to_chars(p, p + 32, v, std::chars_format::general, 7);
  return r.ptr;
}
inline char* putU(char* p, uint64_t v) { return std::to_chars(p, p + 24, v).ptr; }
}  // namespace

bool writeObj(const Scene& s, const std::string& path, const WriteSettings& w, std::string& err) {
  fs::path out = pathFromUtf8(path);
  std::string mtlName = pathToUtf8(out.stem()) + ".mtl";
  std::vector<std::string> rel = writeTextureFiles(s, textureDirFor(path), pathToUtf8(out.parent_path()));
  (void)w;
  // MTL
  {
    std::string t = "# Rx Twister\n";
    char buf[256];
    for (size_t i = 0; i < s.materials.size(); ++i) {
      const Material& m = s.materials[i];
      t += "\nnewmtl " + (m.name.empty() ? "material" + std::to_string(i) : m.name) + "\n";
      snprintf(buf, sizeof buf, "Kd %.6g %.6g %.6g\nKa 0 0 0\nKs 0.04 0.04 0.04\n", m.baseColor.x, m.baseColor.y, m.baseColor.z); t += buf;
      float r = std::max(m.roughness, 0.03f);
      snprintf(buf, sizeof buf, "Ns %.4g\nd %.6g\nPr %.6g\nPm %.6g\nillum 2\n", std::min(1000.0f, 2.0f / (r * r) - 2.0f), m.baseColor.w, m.roughness, m.metallic); t += buf;
      Vec3 e = m.emissive * m.emissiveStrength;
      if (e.x + e.y + e.z > 0) { snprintf(buf, sizeof buf, "Ke %.6g %.6g %.6g\n", e.x, e.y, e.z); t += buf; }
      auto tex = [&](const char* key, const TextureSlot& sl, const std::string& opt = "") {
        if (sl.valid() && size_t(sl.image) < rel.size() && !rel[sl.image].empty()) t += std::string(key) + " " + opt + rel[sl.image] + "\n";
      };
      tex("map_Kd", m.baseColorTex);
      if (m.alphaMode != AlphaMode::Opaque) tex("map_d", m.baseColorTex);
      if (m.normalTex.valid()) { snprintf(buf, sizeof buf, "-bm %.4g ", m.normalScale); tex("map_Bump", m.normalTex, buf); tex("norm", m.normalTex); }
      tex("map_Ke", m.emissiveTex);
      tex("map_Pr", m.roughnessTex);
      tex("map_Pm", m.metallicTex);
      if (m.occlusionTex.valid() && m.occlusionTex.image != m.metalRoughTex.image) tex("map_AO", m.occlusionTex);
    }
    if (!writeWholeFile(pathToUtf8(out.parent_path() / pathFromUtf8(mtlName)), reinterpret_cast<const uint8_t*>(t.data()), t.size())) {
      err = "cannot write mtl"; return false;
    }
  }
  FileWriter f(path);
  if (!f.ok()) { err = "cannot create " + path; return false; }
  f.writeStr("# Rx Twister\nmtllib " + mtlName + "\n");
  auto world = worldMatrices(s);
  uint64_t base = 1;
  std::vector<char> reach(s.nodes.size(), 0);
  { std::vector<int> st(s.roots); while (!st.empty()) { int n = st.back(); st.pop_back(); if (n < 0 || size_t(n) >= s.nodes.size() || reach[n]) continue; reach[n] = 1; for (int c : s.nodes[n].children) st.push_back(c); } }
  for (size_t ni = 0; ni < s.nodes.size(); ++ni) {
    if (!reach[ni]) continue;
    for (int mi : s.nodes[ni].meshes) {
      const Mesh& m = s.meshes[mi];
      if (m.indices.empty()) continue;
      const Mat4& M = world[ni];
      const bool ident = M.isIdentity(), flip = M.det3() < 0;
      Mesh tmp;
      const Mesh* src = &m;
      if (!ident) { tmp = m; transformMesh(tmp, M); src = &tmp; }
      const size_t vc = src->positions.size();
      const bool t = src->hasUV(), n = src->hasNormals(), c = src->colors.size() == vc;
      std::string name = s.nodes[ni].name.empty() ? m.name : s.nodes[ni].name;
      f.writeStr("o " + (name.empty() ? "mesh" + std::to_string(mi) : name) + "\n");
      // vertices, formatted in parallel blocks
      const size_t blk = 1 << 16;
      size_t nb = (vc + blk - 1) / blk;
      std::vector<std::string> parts(nb);
      parallelFor(nb, [&](size_t b) {
        std::string& o = parts[b];
        size_t e = std::min(vc, (b + 1) * blk);
        o.resize((e - b * blk) * (c ? 140 : 100) + (t ? (e - b * blk) * 40 : 0) + (n ? (e - b * blk) * 60 : 0));
        char* p = o.data();
        for (size_t i = b * blk; i < e; ++i) {
          Vec3 v = src->positions[i];
          *p++ = 'v'; *p++ = ' '; p = putF(p, v.x); *p++ = ' '; p = putF(p, v.y); *p++ = ' '; p = putF(p, v.z);
          if (c) { Color8 col = src->colors[i]; *p++ = ' '; p = putF(p, col.r / 255.f); *p++ = ' '; p = putF(p, col.g / 255.f); *p++ = ' '; p = putF(p, col.b / 255.f); }
          *p++ = '\n';
        }
        if (t) for (size_t i = b * blk; i < e; ++i) { *p++ = 'v'; *p++ = 't'; *p++ = ' '; p = putF(p, src->uv0[i].x); *p++ = ' '; p = putF(p, 1.0f - src->uv0[i].y); *p++ = '\n'; }
        if (n) for (size_t i = b * blk; i < e; ++i) { Vec3 v = src->normals[i]; *p++ = 'v'; *p++ = 'n'; *p++ = ' '; p = putF(p, v.x); *p++ = ' '; p = putF(p, v.y); *p++ = ' '; p = putF(p, v.z); *p++ = '\n'; }
        o.resize(size_t(p - o.data()));
      }, 1);
      for (auto& p : parts) f.writeStr(p);
      parts.clear();
      if (m.material >= 0 && size_t(m.material) < s.materials.size())
        f.writeStr("usemtl " + (s.materials[m.material].name.empty() ? "material" + std::to_string(m.material) : s.materials[m.material].name) + "\n");
      const size_t tc = src->triangleCount();
      nb = (tc + blk - 1) / blk;
      parts.resize(nb);
      parallelFor(nb, [&](size_t b) {
        std::string& o = parts[b];
        size_t e = std::min(tc, (b + 1) * blk);
        o.resize((e - b * blk) * 3 * 64 + 16);
        char* p = o.data();
        for (size_t tri = b * blk; tri < e; ++tri) {
          *p++ = 'f';
          for (int k = 0; k < 3; ++k) {
            int kk = flip && !ident ? k : k;  // transformMesh already fixed winding
            uint64_t id = base + src->indices[tri * 3 + kk];
            *p++ = ' '; p = putU(p, id);
            if (t || n) { *p++ = '/'; if (t) p = putU(p, id); if (n) { *p++ = '/'; p = putU(p, id); } }
          }
          *p++ = '\n';
        }
        o.resize(size_t(p - o.data()));
      }, 1);
      for (auto& p : parts) f.writeStr(p);
      base += vc;
    }
  }
  if (!f.close()) { err = "write failed"; return false; }
  return true;
}

}  // namespace rx
