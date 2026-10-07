// STL (binary / ASCII) and PLY (ASCII / binary LE / binary BE) readers and writers.
#include "io/io.h"
#include "core/par.h"
#include "core/util.h"
#include "geom/mesh_ops.h"
#include <charconv>
#include <cstring>
#include <map>

namespace rx {

namespace {
void addSingleMeshNode(Scene& s, Mesh&& m) {
  Node n; n.name = m.name; n.meshes = {int(s.meshes.size())};
  s.meshes.push_back(std::move(m));
  s.roots.push_back(int(s.nodes.size()));
  s.nodes.push_back(n);
}

// world space copy of every mesh instance, merged into one
Mesh mergedWorld(const Scene& s, bool keepUV) {
  Mesh out;
  out.name = "merged";
  auto world = worldMatrices(s);
  std::vector<char> reach(s.nodes.size(), 0);
  std::vector<int> st(s.roots);
  while (!st.empty()) { int n = st.back(); st.pop_back(); if (n < 0 || size_t(n) >= s.nodes.size() || reach[n]) continue; reach[n] = 1; for (int c : s.nodes[n].children) st.push_back(c); }
  bool allN = true, allT = true, anyC = false;
  for (size_t ni = 0; ni < s.nodes.size(); ++ni) if (reach[ni]) for (int mi : s.nodes[ni].meshes) {
    allN &= s.meshes[mi].hasNormals(); allT &= s.meshes[mi].hasUV(); anyC |= !s.meshes[mi].colors.empty();
  }
  for (size_t ni = 0; ni < s.nodes.size(); ++ni) {
    if (!reach[ni]) continue;
    for (int mi : s.nodes[ni].meshes) {
      Mesh m = s.meshes[mi];
      transformMesh(m, world[ni]);
      uint32_t base = uint32_t(out.positions.size());
      out.positions.insert(out.positions.end(), m.positions.begin(), m.positions.end());
      if (allN) out.normals.insert(out.normals.end(), m.normals.begin(), m.normals.end());
      if (allT && keepUV) out.uv0.insert(out.uv0.end(), m.uv0.begin(), m.uv0.end());
      if (anyC) { if (m.colors.size() != m.positions.size()) m.colors.assign(m.positions.size(), {255, 255, 255, 255}); out.colors.insert(out.colors.end(), m.colors.begin(), m.colors.end()); }
      for (uint32_t i : m.indices) out.indices.push_back(base + i);
    }
  }
  return out;
}
}  // namespace

// ------------------------------------------------------------------------------- STL
bool readStl(const std::string& path, Scene& s, std::string& err) {
  auto f = mapFile(path, &err);
  if (!f) return false;
  Mesh m;
  m.name = stemOf(path);
  uint32_t n = 0;
  if (f->size >= 84) memcpy(&n, f->data + 80, 4);
  if (f->size >= 84 && 84 + size_t(n) * 50 == f->size) {
    m.positions.resize(size_t(n) * 3);
    parallelFor(n, [&](size_t t) {
      const uint8_t* p = f->data + 84 + t * 50 + 12;
      memcpy(&m.positions[t * 3], p, 36);
    }, 1 << 14);
  } else {
    const char* d = reinterpret_cast<const char*>(f->data);
    const char* e = d + f->size;
    const char* p = d;
    while (p < e) {
      const char* v = findBytes(p, size_t(e - p), "vertex", 6);
      if (!v) break;
      p = v + 6;
      Vec3 q;
      float* c[3] = {&q.x, &q.y, &q.z};
      for (int k = 0; k < 3; ++k) {
        while (p < e && (*p == ' ' || *p == '\t')) ++p;
        auto r = std::from_chars(p, e, *c[k]);
        p = r.ptr;
      }
      m.positions.push_back(q);
    }
    m.positions.resize(m.positions.size() / 3 * 3);
    if (m.positions.empty()) { err = "no triangles found in STL"; return false; }
  }
  m.indices.resize(m.positions.size());
  for (size_t i = 0; i < m.indices.size(); ++i) m.indices[i] = uint32_t(i);
  weldMesh(m);
  addSingleMeshNode(s, std::move(m));
  s.sourceFormat = "stl";
  return true;
}

bool writeStl(const Scene& s, const std::string& path, const WriteSettings&, std::string& err) {
  Mesh m = mergedWorld(s, false);
  const size_t tc = m.triangleCount();
  if (tc > 0xFFFFFFFFull) { err = "too many triangles for STL"; return false; }
  std::vector<uint8_t> buf(84 + tc * 50, 0);
  const char hdr[] = "Rx Twister binary STL";
  memcpy(buf.data(), hdr, sizeof hdr - 1);
  uint32_t n = uint32_t(tc);
  memcpy(&buf[80], &n, 4);
  parallelFor(tc, [&](size_t t) {
    Vec3 a = m.positions[m.indices[t * 3]], b = m.positions[m.indices[t * 3 + 1]], c = m.positions[m.indices[t * 3 + 2]];
    Vec3 nn = normalize(cross(b - a, c - a));
    float v[12] = {nn.x, nn.y, nn.z, a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z};
    memcpy(&buf[84 + t * 50], v, 48);
  }, 1 << 14);
  if (!writeWholeFile(path, buf.data(), buf.size())) { err = "cannot write " + path; return false; }
  return true;
}

// ------------------------------------------------------------------------------- PLY
namespace {
enum class PT { I8, U8, I16, U16, I32, U32, F32, F64, None };
PT ptype(const std::string& t) {
  if (t == "char" || t == "int8") return PT::I8;
  if (t == "uchar" || t == "uint8") return PT::U8;
  if (t == "short" || t == "int16") return PT::I16;
  if (t == "ushort" || t == "uint16") return PT::U16;
  if (t == "int" || t == "int32") return PT::I32;
  if (t == "uint" || t == "uint32") return PT::U32;
  if (t == "float" || t == "float32") return PT::F32;
  if (t == "double" || t == "float64") return PT::F64;
  return PT::None;
}
size_t psize(PT t) {
  switch (t) { case PT::I8: case PT::U8: return 1; case PT::I16: case PT::U16: return 2; case PT::I32: case PT::U32: case PT::F32: return 4; case PT::F64: return 8; default: return 0; }
}
struct Prop { std::string name; PT type = PT::None; PT countType = PT::None; bool list = false; };
struct Elem { std::string name; size_t count = 0; std::vector<Prop> props; };

template <class T> T rd(const uint8_t* p, bool swap) {
  T v; memcpy(&v, p, sizeof v);
  if (swap) { uint8_t* b = reinterpret_cast<uint8_t*>(&v); std::reverse(b, b + sizeof v); }
  return v;
}
double rdBin(const uint8_t* p, PT t, bool sw) {
  switch (t) {
    case PT::I8: return int8_t(*p); case PT::U8: return *p;
    case PT::I16: return rd<int16_t>(p, sw); case PT::U16: return rd<uint16_t>(p, sw);
    case PT::I32: return rd<int32_t>(p, sw); case PT::U32: return rd<uint32_t>(p, sw);
    case PT::F32: return rd<float>(p, sw); case PT::F64: return rd<double>(p, sw);
    default: return 0;
  }
}
}  // namespace

bool readPly(const std::string& path, Scene& s, std::string& err) {
  auto f = mapFile(path, &err);
  if (!f) return false;
  const char* d = reinterpret_cast<const char*>(f->data);
  const char* he = findBytes(d, std::min<size_t>(f->size, 1 << 20), "end_header", 10);
  if (strncmp(d, "ply", 3) || !he) { err = "not a PLY file"; return false; }
  const char* body = static_cast<const char*>(memchr(he, '\n', size_t(d + f->size - he)));
  if (!body) { err = "truncated PLY"; return false; }
  ++body;
  std::string header(d, he);
  std::vector<Elem> elems;
  int fmt = 0;  // 0 ascii, 1 LE, 2 BE
  std::string textureFile;
  size_t pos = 0;
  while (pos < header.size()) {
    size_t e = header.find('\n', pos);
    if (e == std::string::npos) e = header.size();
    std::string line = header.substr(pos, e - pos);
    pos = e + 1;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    std::vector<std::string> t;
    for (size_t i = 0; i < line.size();) {
      while (i < line.size() && line[i] == ' ') ++i;
      size_t j = i;
      while (j < line.size() && line[j] != ' ') ++j;
      if (j > i) t.push_back(line.substr(i, j - i));
      i = j;
    }
    if (t.empty()) continue;
    if (t[0] == "format") fmt = t[1] == "ascii" ? 0 : t[1] == "binary_little_endian" ? 1 : 2;
    else if (t[0] == "element" && t.size() >= 3) elems.push_back({t[1], size_t(std::stoull(t[2])), {}});
    else if (t[0] == "property" && !elems.empty()) {
      Prop p;
      if (t.size() >= 5 && t[1] == "list") { p.list = true; p.countType = ptype(t[2]); p.type = ptype(t[3]); p.name = t[4]; }
      else if (t.size() >= 3) { p.type = ptype(t[1]); p.name = t[2]; }
      elems.back().props.push_back(p);
    } else if (t[0] == "comment" && t.size() >= 3 && (t[1] == "TextureFile" || t[1] == "texturefile")) {
      textureFile = line.substr(line.find(t[1]) + t[1].size() + 1);
    }
  }
  const bool sw = fmt == 2;  // host is little endian
  Mesh m;
  m.name = stemOf(path);
  std::vector<float> faceUV;  // per corner texcoords (MeshLab wedge)
  const uint8_t* p = reinterpret_cast<const uint8_t*>(body);
  const uint8_t* end = f->data + f->size;
  const char* tp = body;  // ascii cursor
  auto asciiNum = [&]() -> double {
    while (tp < reinterpret_cast<const char*>(end) && (*tp == ' ' || *tp == '\n' || *tp == '\r' || *tp == '\t')) ++tp;
    double v = 0;
    auto r = std::from_chars(tp, reinterpret_cast<const char*>(end), v);
    tp = r.ptr == tp ? tp + 1 : r.ptr;
    return v;
  };
  for (auto& el : elems) {
    const bool isV = el.name == "vertex", isF = el.name == "face";
    int ix = -1, iy = -1, iz = -1, inx = -1, iny = -1, inz = -1, iu = -1, iv = -1, ir = -1, ig = -1, ib = -1, ia = -1, ilist = -1, iuvlist = -1;
    bool colFloat = false;
    for (size_t k = 0; k < el.props.size(); ++k) {
      const std::string& n = el.props[k].name;
      int ki = int(k);
      if (n == "x") ix = ki; else if (n == "y") iy = ki; else if (n == "z") iz = ki;
      else if (n == "nx") inx = ki; else if (n == "ny") iny = ki; else if (n == "nz") inz = ki;
      else if (n == "s" || n == "u" || n == "texture_u" || n == "texture_s") iu = ki;
      else if (n == "t" || n == "v" || n == "texture_v" || n == "texture_t") iv = ki;
      else if (n == "red" || n == "r") { ir = ki; colFloat = el.props[k].type == PT::F32 || el.props[k].type == PT::F64; }
      else if (n == "green" || n == "g") ig = ki; else if (n == "blue" || n == "b") ib = ki; else if (n == "alpha" || n == "a") ia = ki;
      else if (el.props[k].list && (n == "vertex_indices" || n == "vertex_index")) ilist = ki;
      else if (el.props[k].list && n == "texcoord") iuvlist = ki;
    }
    bool fixed = true;
    size_t rec = 0;
    std::vector<size_t> offs;
    for (auto& pr : el.props) { offs.push_back(rec); if (pr.list) fixed = false; rec += psize(pr.type); }
    if (isV) {
      m.positions.resize(el.count);
      if (inx >= 0) m.normals.resize(el.count);
      if (iu >= 0 && iv >= 0) m.uv0.resize(el.count);
      if (ir >= 0 && ig >= 0 && ib >= 0) m.colors.resize(el.count);
    }
    auto store = [&](size_t i, const std::vector<double>& val) {
      if (!isV) return;
      m.positions[i] = {float(val[ix]), float(val[iy]), float(val[iz])};
      if (inx >= 0) m.normals[i] = {float(val[inx]), float(val[iny]), float(val[inz])};
      if (!m.uv0.empty()) m.uv0[i] = {float(val[iu]), 1.0f - float(val[iv])};
      if (!m.colors.empty()) {
        auto q = [&](double x) { return uint8_t(colFloat ? std::clamp(x, 0.0, 1.0) * 255 + 0.5 : std::clamp(x, 0.0, 255.0)); };
        m.colors[i] = {q(val[ir]), q(val[ig]), q(val[ib]), ia >= 0 ? q(val[ia]) : uint8_t(255)};
      }
    };
    if (fmt != 0 && fixed) {
      if (size_t(end - p) < rec * el.count) { err = "truncated PLY body"; return false; }
      if (isV) {
        if (ix < 0) { err = "PLY vertex without x/y/z"; return false; }
        parallelRanges(el.count, 1 << 15, [&](size_t b, size_t e) {
          std::vector<double> val(el.props.size());
          for (size_t i = b; i < e; ++i) {
            const uint8_t* r = p + i * rec;
            for (size_t k = 0; k < el.props.size(); ++k) val[k] = rdBin(r + offs[k], el.props[k].type, sw);
            store(i, val);
          }
        });
      }
      p += rec * el.count;
      continue;
    }
    std::vector<double> val(el.props.size());
    std::vector<uint32_t> poly;
    std::vector<float> uvs;
    for (size_t i = 0; i < el.count; ++i) {
      poly.clear(); uvs.clear();
      for (size_t k = 0; k < el.props.size(); ++k) {
        const Prop& pr = el.props[k];
        if (pr.list) {
          size_t cnt;
          if (fmt) { cnt = size_t(rdBin(p, pr.countType, sw)); p += psize(pr.countType); }
          else cnt = size_t(asciiNum());
          for (size_t c = 0; c < cnt; ++c) {
            double v;
            if (fmt) { if (p + psize(pr.type) > end) { err = "truncated PLY"; return false; } v = rdBin(p, pr.type, sw); p += psize(pr.type); }
            else v = asciiNum();
            if (int(k) == ilist) poly.push_back(uint32_t(v));
            else if (int(k) == iuvlist) uvs.push_back(float(v));
          }
        } else {
          if (fmt) { val[k] = rdBin(p, pr.type, sw); p += psize(pr.type); }
          else val[k] = asciiNum();
        }
      }
      if (isV) store(i, val);
      else if (isF) {
        for (size_t c = 2; c < poly.size(); ++c) {
          m.indices.push_back(poly[0]); m.indices.push_back(poly[c - 1]); m.indices.push_back(poly[c]);
          if (uvs.size() == poly.size() * 2)
            for (size_t q : {size_t(0), c - 1, c}) { faceUV.push_back(uvs[q * 2]); faceUV.push_back(1.0f - uvs[q * 2 + 1]); }
        }
      }
    }
  }
  for (auto& i : m.indices) if (i >= m.positions.size()) i = 0;
  if (!faceUV.empty() && faceUV.size() == m.indices.size() * 2) {
    unweld(m);
    m.uv0.resize(m.indices.size());
    memcpy(m.uv0.data(), faceUV.data(), faceUV.size() * 4);
    weldMesh(m);
  }
  if (m.indices.empty()) logWarn("PLY: no faces (point cloud) - nothing to convert");
  if (!textureFile.empty()) {
    std::string tp2 = resolveTexturePath(pathToUtf8(pathFromUtf8(path).parent_path()), textureFile);
    int im = tp2.empty() ? -1 : loadTextureFile(s, tp2);
    if (im >= 0) { Material mat; mat.name = m.name; mat.baseColorTex = {im, 0}; m.material = s.addMaterial(std::move(mat)); }
  }
  addSingleMeshNode(s, std::move(m));
  s.sourceFormat = "ply";
  return true;
}

bool writePly(const Scene& s, const std::string& path, const WriteSettings&, std::string& err) {
  Mesh m = mergedWorld(s, true);
  const size_t vc = m.positions.size(), tc = m.triangleCount();
  const bool n = m.hasNormals(), t = m.hasUV(), c = m.colors.size() == vc;
  std::string h = "ply\nformat binary_little_endian 1.0\ncomment Rx Twister\n";
  std::vector<std::string> rel;
  if (t) {
    for (auto& mat : s.materials)
      if (mat.baseColorTex.valid()) {
        rel = writeTextureFiles(s, textureDirFor(path), pathToUtf8(pathFromUtf8(path).parent_path()));
        h += "comment TextureFile " + rel[mat.baseColorTex.image] + "\n";
        break;
      }
  }
  h += "element vertex " + std::to_string(vc) + "\nproperty float x\nproperty float y\nproperty float z\n";
  if (n) h += "property float nx\nproperty float ny\nproperty float nz\n";
  if (t) h += "property float s\nproperty float t\n";
  if (c) h += "property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\n";
  h += "element face " + std::to_string(tc) + "\nproperty list uchar uint vertex_indices\nend_header\n";
  const size_t rec = 12 + (n ? 12 : 0) + (t ? 8 : 0) + (c ? 4 : 0);
  std::vector<uint8_t> vb(vc * rec), fb(tc * 13);
  parallelFor(vc, [&](size_t i) {
    uint8_t* p = &vb[i * rec];
    memcpy(p, &m.positions[i], 12); p += 12;
    if (n) { memcpy(p, &m.normals[i], 12); p += 12; }
    if (t) { float uv[2] = {m.uv0[i].x, 1.0f - m.uv0[i].y}; memcpy(p, uv, 8); p += 8; }
    if (c) memcpy(p, &m.colors[i], 4);
  }, 1 << 14);
  parallelFor(tc, [&](size_t i) { fb[i * 13] = 3; memcpy(&fb[i * 13 + 1], &m.indices[i * 3], 12); }, 1 << 14);
  FileWriter f(path);
  if (!f.ok()) { err = "cannot create " + path; return false; }
  f.writeStr(h); f.write(vb.data(), vb.size()); f.write(fb.data(), fb.size());
  if (!f.close()) { err = "write failed"; return false; }
  return true;
}

}  // namespace rx
