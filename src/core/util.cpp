#include "core/util.h"
#include "core/par.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#if defined(_WIN32)
#define NOMINMAX
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace rx {

// ---- Blob / threads --------------------------------------------------------
std::shared_ptr<Blob> Blob::fromVector(std::vector<uint8_t>&& v) {
  auto holder = std::make_shared<std::vector<uint8_t>>(std::move(v));
  auto b = std::make_shared<Blob>();
  b->data = holder->data();
  b->size = holder->size();
  b->owner = holder;
  return b;
}

static int g_threadLimit = 0;
int hardwareThreads() { return std::max(1u, std::thread::hardware_concurrency()); }
int threadLimit() { return g_threadLimit > 0 ? g_threadLimit : hardwareThreads(); }
void setThreadLimit(int n) {
  g_threadLimit = n;
#if defined(RX_HAVE_TBB)
  static std::unique_ptr<tbb::global_control> gc;
  gc.reset(new tbb::global_control(tbb::global_control::max_allowed_parallelism, size_t(threadLimit())));
#endif
}

// ---- logging -----------------------------------------------------------------
static LogLevel g_level = LogLevel::Info;
static std::mutex g_logMutex;
void setLogLevel(LogLevel l) { g_level = l; }
LogLevel logLevel() { return g_level; }
static void vlog(FILE* out, const char* tag, const char* fmt, va_list ap) {
  std::lock_guard<std::mutex> lk(g_logMutex);
  if (tag && *tag) fprintf(out, "%s", tag);
  vfprintf(out, fmt, ap);
  fputc('\n', out);
}
void logInfo(const char* fmt, ...) {
  if (g_level == LogLevel::Quiet) return;
  va_list ap; va_start(ap, fmt); vlog(stdout, "", fmt, ap); va_end(ap);
}
void logVerbose(const char* fmt, ...) {
  if (g_level != LogLevel::Verbose) return;
  char tag[48] = "  . ";
  if (size_t rss = residentMemoryBytes()) snprintf(tag, sizeof tag, "  . [%5.2f GB] ", double(rss) / double(1u << 30));
  va_list ap; va_start(ap, fmt); vlog(stdout, tag, fmt, ap); va_end(ap);
}
void logWarn(const char* fmt, ...) {
  if (g_level == LogLevel::Quiet) return;
  va_list ap; va_start(ap, fmt); vlog(stderr, "warning: ", fmt, ap); va_end(ap);
}
void logError(const char* fmt, ...) {
  va_list ap; va_start(ap, fmt); vlog(stderr, "error: ", fmt, ap); va_end(ap);
}

// ---- strings -------------------------------------------------------------------
std::string toLower(std::string s) {
  for (auto& c : s) c = char(std::tolower((unsigned char)c));
  return s;
}
std::string extOf(const std::string& path) {
  auto p = path.find_last_of("./\\");
  if (p == std::string::npos || path[p] != '.') return "";
  return toLower(path.substr(p + 1));
}
std::string stemOf(const std::string& path) { return pathToUtf8(pathFromUtf8(path).stem()); }
bool startsWith(std::string_view s, std::string_view p) { return s.size() >= p.size() && s.compare(0, p.size(), p) == 0; }
bool endsWith(std::string_view s, std::string_view p) { return s.size() >= p.size() && s.compare(s.size() - p.size(), p.size(), p) == 0; }
std::string sanitizeFileName(const std::string& s) {
  std::string r;
  for (unsigned char c : s) r += (std::isalnum(c) || c == '_' || c == '-' || c == '.' || c >= 0x80) ? char(c) : '_';
  if (r.empty()) r = "unnamed";
  return r;
}
std::string humanBytes(double b) {
  const char* u[] = {"B", "KB", "MB", "GB", "TB"};
  int i = 0;
  while (b >= 1024 && i < 4) { b /= 1024; ++i; }
  char buf[64]; snprintf(buf, sizeof buf, "%.1f %s", b, u[i]);
  return buf;
}
std::string pathToUtf8(const fs::path& p) {
#if defined(__cpp_char8_t)
  auto s = p.u8string();
  return std::string(s.begin(), s.end());
#else
  return p.u8string();
#endif
}
fs::path pathFromUtf8(const std::string& s) {
#if defined(__cpp_char8_t)
  return fs::path(std::u8string(s.begin(), s.end()));
#else
  return fs::u8path(s);
#endif
}
std::string percentDecode(const std::string& s) {
  std::string r;
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '%' && i + 2 < s.size() + 0 && std::isxdigit((unsigned char)s[i + 1]) && std::isxdigit((unsigned char)s[i + 2])) {
      r += char(std::stoi(s.substr(i + 1, 2), nullptr, 16));
      i += 2;
    } else r += s[i];
  }
  return r;
}
std::string percentEncodePath(const std::string& s) {
  std::string r;
  char buf[4];
  for (unsigned char c : s) {
    if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '/' || c == '~') r += char(c);
    else { snprintf(buf, sizeof buf, "%%%02X", c); r += buf; }
  }
  return r;
}
static const char* kB64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
std::vector<uint8_t> base64Decode(std::string_view s) {
  int8_t T[256]; memset(T, -1, sizeof T);
  for (int i = 0; i < 64; ++i) T[(unsigned char)kB64[i]] = int8_t(i);
  std::vector<uint8_t> out; out.reserve(s.size() * 3 / 4);
  uint32_t acc = 0; int bits = 0;
  for (unsigned char c : s) {
    if (c == '=') break;
    int8_t v = T[c]; if (v < 0) continue;
    acc = (acc << 6) | uint32_t(v); bits += 6;
    if (bits >= 8) { bits -= 8; out.push_back(uint8_t((acc >> bits) & 0xFF)); }
  }
  return out;
}
std::string base64Encode(const uint8_t* d, size_t n) {
  std::string r; r.reserve((n + 2) / 3 * 4);
  for (size_t i = 0; i < n; i += 3) {
    uint32_t v = uint32_t(d[i]) << 16 | (i + 1 < n ? uint32_t(d[i + 1]) << 8 : 0) | (i + 2 < n ? d[i + 2] : 0);
    r += kB64[(v >> 18) & 63]; r += kB64[(v >> 12) & 63];
    r += i + 1 < n ? kB64[(v >> 6) & 63] : '='; r += i + 2 < n ? kB64[v & 63] : '=';
  }
  return r;
}

const char* findBytes(const char* hay, size_t n, const char* needle, size_t m) {
  if (m == 0 || n < m) return nullptr;
  const char* end = hay + n - m + 1;
  for (const char* p = hay; p < end;) {
    p = static_cast<const char*>(memchr(p, needle[0], size_t(end - p)));
    if (!p) return nullptr;
    if (!memcmp(p, needle, m)) return p;
    ++p;
  }
  return nullptr;
}

// ---- files ----------------------------------------------------------------------
#if defined(_WIN32)
struct MapOwner { HANDLE f = INVALID_HANDLE_VALUE, m = nullptr; void* p = nullptr; ~MapOwner() { if (p) UnmapViewOfFile(p); if (m) CloseHandle(m); if (f != INVALID_HANDLE_VALUE) CloseHandle(f); } };
std::shared_ptr<Blob> mapFile(const std::string& path, std::string* err) {
  auto o = std::make_shared<MapOwner>();
  o->f = CreateFileW(pathFromUtf8(path).c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (o->f == INVALID_HANDLE_VALUE) { if (err) *err = "cannot open " + path; return nullptr; }
  LARGE_INTEGER sz; GetFileSizeEx(o->f, &sz);
  auto b = std::make_shared<Blob>();
  b->size = size_t(sz.QuadPart);
  if (b->size) {
    o->m = CreateFileMappingW(o->f, nullptr, PAGE_READONLY, 0, 0, nullptr);
    if (o->m) o->p = MapViewOfFile(o->m, FILE_MAP_READ, 0, 0, 0);
    if (!o->p) { if (err) *err = "cannot map " + path; return nullptr; }
  }
  b->data = (const uint8_t*)o->p; b->owner = o;
  return b;
}
size_t physicalMemoryBytes() { MEMORYSTATUSEX s{sizeof s}; GlobalMemoryStatusEx(&s); return size_t(s.ullTotalPhys); }
#else
struct MapOwner { void* p = nullptr; size_t n = 0; ~MapOwner() { if (p) munmap(p, n); } };
std::shared_ptr<Blob> mapFile(const std::string& path, std::string* err) {
  int fd = open(path.c_str(), O_RDONLY);
  if (fd < 0) { if (err) *err = "cannot open " + path; return nullptr; }
  struct stat st; fstat(fd, &st);
  auto b = std::make_shared<Blob>();
  b->size = size_t(st.st_size);
  auto o = std::make_shared<MapOwner>();
  if (b->size) {
    void* p = mmap(nullptr, b->size, PROT_READ, MAP_PRIVATE, fd, 0);
    if (p == MAP_FAILED) { close(fd); if (err) *err = "cannot mmap " + path; return nullptr; }
    madvise(p, b->size, MADV_SEQUENTIAL);
    o->p = p; o->n = b->size;
  }
  close(fd);
  b->data = (const uint8_t*)o->p; b->owner = o;
  return b;
}
size_t physicalMemoryBytes() {
  long pages = sysconf(_SC_PHYS_PAGES), ps = sysconf(_SC_PAGE_SIZE);
  return pages > 0 && ps > 0 ? size_t(pages) * size_t(ps) : size_t(8) << 30;
}
#endif

void releaseMappedRange(const void* p, size_t n) {
#if !defined(_WIN32)
  const uintptr_t ps = uintptr_t(sysconf(_SC_PAGE_SIZE));
  uintptr_t b = (uintptr_t(p) + ps - 1) & ~(ps - 1), e = (uintptr_t(p) + n) & ~(ps - 1);
  if (e > b) madvise(reinterpret_cast<void*>(b), e - b, MADV_DONTNEED);
#else
  (void)p; (void)n;
#endif
}

size_t residentMemoryBytes() {
#if defined(__linux__)
  if (FILE* f = fopen("/proc/self/statm", "r")) {
    unsigned long a = 0, b = 0;
    int n = fscanf(f, "%lu %lu", &a, &b);
    fclose(f);
    if (n == 2) return size_t(b) * size_t(sysconf(_SC_PAGE_SIZE));
  }
#endif
  return 0;
}

bool readWholeFile(const std::string& path, std::vector<uint8_t>& out) {
  auto b = mapFile(path);
  if (!b) return false;
  out.assign(b->data, b->data + b->size);
  return true;
}
bool writeWholeFile(const std::string& path, const uint8_t* d, size_t n) {
  FileWriter w(path);
  if (!w.ok()) return false;
  w.write(d, n);
  return w.close();
}

FileWriter::FileWriter(const std::string& path) {
  auto p = pathFromUtf8(path);
  if (p.has_parent_path()) { std::error_code ec; fs::create_directories(p.parent_path(), ec); }
#if defined(_WIN32)
  f_ = _wfopen(p.c_str(), L"wb");
#else
  f_ = fopen(p.c_str(), "wb");
#endif
  if (f_) setvbuf(f_, nullptr, _IOFBF, 1 << 20);
}
FileWriter::~FileWriter() { close(); }
void FileWriter::write(const void* d, size_t n) {
  if (!f_ || failed_ || n == 0) return;
  if (fwrite(d, 1, n, f_) != n) failed_ = true;
  pos_ += n;
}
void FileWriter::pad(size_t n, uint8_t v) {
  uint8_t buf[64]; memset(buf, v, sizeof buf);
  while (n) { size_t k = std::min<size_t>(n, sizeof buf); write(buf, k); n -= k; }
}
bool FileWriter::close() {
  if (!f_) return !failed_;
  bool ok = fclose(f_) == 0 && !failed_;
  f_ = nullptr;
  failed_ = !ok;
  return ok;
}

void MemoryGate::acquire(size_t bytes) {
  std::unique_lock<std::mutex> lk(m_);
  bytes = std::min(bytes, budget_);  // a single oversized job may run alone
  cv_.wait(lk, [&] { return used_ == 0 || used_ + bytes <= budget_; });
  used_ += bytes;
}
void MemoryGate::release(size_t bytes) {
  { std::lock_guard<std::mutex> lk(m_); used_ -= std::min(std::min(bytes, budget_), used_); }
  cv_.notify_all();
}

// ---- Scene helpers -----------------------------------------------------------
size_t Mesh::memoryBytes() const {
  return positions.size() * sizeof(Vec3) + normals.size() * sizeof(Vec3) + tangents.size() * sizeof(Vec4) +
         (uv0.size() + uv1.size()) * sizeof(Vec2) + colors.size() * 4 + indices.size() * 4;
}
size_t Scene::totalTriangles() const { size_t n = 0; for (auto& m : meshes) n += m.triangleCount(); return n; }
size_t Scene::totalVertices() const { size_t n = 0; for (auto& m : meshes) n += m.vertexCount(); return n; }
int Scene::addImage(Image&& im) { images.push_back(std::move(im)); return int(images.size()) - 1; }
int Scene::addMaterial(Material&& m) { materials.push_back(std::move(m)); return int(materials.size()) - 1; }

static void walkWorld(const Scene& s, int n, const Mat4& parent, std::vector<Mat4>& world, std::vector<char>& seen) {
  if (n < 0 || size_t(n) >= s.nodes.size() || seen[n]) return;
  seen[n] = 1;
  world[n] = parent * s.nodes[n].local;
  for (int c : s.nodes[n].children) walkWorld(s, c, world[n], world, seen);
}
Aabb Scene::bounds() const {
  Aabb b;
  std::vector<Mat4> world(nodes.size());
  std::vector<char> seen(nodes.size(), 0);
  for (int r : roots) walkWorld(*this, r, Mat4::identity(), world, seen);
  for (size_t i = 0; i < nodes.size(); ++i) {
    if (!seen[i]) continue;
    for (int mi : nodes[i].meshes) {
      if (mi < 0 || size_t(mi) >= meshes.size()) continue;
      const Mesh& m = meshes[mi];
      Aabb lb;
      for (auto& p : m.positions) lb.add(p);
      if (!lb.valid()) continue;
      for (int c = 0; c < 8; ++c)
        b.add(world[i].point({c & 1 ? lb.hi.x : lb.lo.x, c & 2 ? lb.hi.y : lb.lo.y, c & 4 ? lb.hi.z : lb.lo.z}));
    }
  }
  return b;
}
bool Scene::hasTransforms() const {
  std::vector<int> uses(meshes.size(), 0);
  for (auto& n : nodes) {
    if (!n.local.isIdentity() && !n.meshes.empty()) return true;
    for (int m : n.meshes) if (m >= 0 && size_t(m) < uses.size() && ++uses[m] > 1) return true;
  }
  // transforms on ancestors
  for (auto& n : nodes) if (!n.local.isIdentity() && !n.children.empty()) return true;
  return false;
}

}  // namespace rx
