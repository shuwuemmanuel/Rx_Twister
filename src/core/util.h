// Misc internal utilities: logging, files, strings, timers, memory budget gate.
#pragma once
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>
#include "rx/scene.h"

namespace rx {
namespace fs = std::filesystem;

#if defined(__GNUC__) || defined(__clang__)
#define RX_PRINTF(a, b) __attribute__((format(printf, a, b)))
#else
#define RX_PRINTF(a, b)
#endif

constexpr float kPi = 3.14159265358979323846f;

enum class LogLevel { Quiet, Info, Verbose };
void setLogLevel(LogLevel l);
LogLevel logLevel();
void logInfo(const char* fmt, ...) RX_PRINTF(1, 2);
void logWarn(const char* fmt, ...) RX_PRINTF(1, 2);
void logVerbose(const char* fmt, ...) RX_PRINTF(1, 2);
void logError(const char* fmt, ...) RX_PRINTF(1, 2);

struct Timer {
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  double seconds() const { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); }
};

// ---- strings -------------------------------------------------------------
std::string toLower(std::string s);
std::string extOf(const std::string& path);                 // lower-case, no dot
std::string stemOf(const std::string& path);
bool startsWith(std::string_view s, std::string_view p);
bool endsWith(std::string_view s, std::string_view p);
std::string sanitizeFileName(const std::string& s);
std::string humanBytes(double b);
std::string pathToUtf8(const fs::path& p);
fs::path pathFromUtf8(const std::string& s);
std::string percentDecode(const std::string& s);
std::string percentEncodePath(const std::string& s);
std::vector<uint8_t> base64Decode(std::string_view s);
std::string base64Encode(const uint8_t* d, size_t n);
// portable memmem
const char* findBytes(const char* hay, size_t n, const char* needle, size_t m);

// ---- files ---------------------------------------------------------------
// Memory maps a file read-only (or reads it on platforms without mmap).
std::shared_ptr<Blob> mapFile(const std::string& path, std::string* err = nullptr);
bool readWholeFile(const std::string& path, std::vector<uint8_t>& out);
bool writeWholeFile(const std::string& path, const uint8_t* d, size_t n);
size_t physicalMemoryBytes();
size_t residentMemoryBytes();   // current RSS (0 if unknown)
// Hint that a range of a read-only file mapping will not be read again (drops it from RAM).
void releaseMappedRange(const void* p, size_t n);

// Buffered binary writer that supports huge outputs.
class FileWriter {
 public:
  explicit FileWriter(const std::string& path);
  ~FileWriter();
  bool ok() const { return f_ != nullptr && !failed_; }
  void write(const void* d, size_t n);
  void writeStr(std::string_view s) { write(s.data(), s.size()); }
  void pad(size_t n, uint8_t v = 0);
  uint64_t position() const { return pos_; }
  bool close();
 private:
  FILE* f_ = nullptr;
  bool failed_ = false;
  uint64_t pos_ = 0;
};

// Byte-budget gate to keep concurrent image processing inside RAM.
class MemoryGate {
 public:
  explicit MemoryGate(size_t budget) : budget_(budget) {}
  void acquire(size_t bytes);
  void release(size_t bytes);
 private:
  std::mutex m_;
  std::condition_variable cv_;
  size_t budget_, used_ = 0;
};

}  // namespace rx
