#include "rx/c_api.h"
#include "rx/rx.h"
#include "cli_args.h"
#include <cstring>

static void copyOut(const std::string& s, char* out, size_t n) {
  if (!out || !n) return;
  size_t k = std::min(n - 1, s.size());
  memcpy(out, s.data(), k);
  out[k] = 0;
}

extern "C" int rx_convert(const char* input, const char* output, const char* args, rx_progress_fn progress, void* user,
                          char* error, size_t error_size) {
  rx::Options o;
  std::vector<std::string> pos;
  std::string err;
  if (!rx::parseArgs(rx::splitArgs(args ? args : ""), o, pos, err)) { copyOut(err, error, error_size); return 1; }
  o.input = input ? input : "";
  if (output) o.output = output;
  if (progress) o.onProgress = [=](const std::string& st, double f) { progress(st.c_str(), f, user); };
  rx::Result r = rx::convert(o);
  if (!r.ok) { copyOut(r.error, error, error_size); return 2; }
  copyOut("", error, error_size);
  return 0;
}

extern "C" int rx_describe(const char* input, char* out, size_t out_size) {
  rx::Scene s;
  rx::Options o;
  std::string err;
  if (!rx::loadScene(input ? input : "", s, o, err)) { copyOut("error: " + err, out, out_size); return 1; }
  copyOut(rx::describeScene(s), out, out_size);
  return 0;
}

extern "C" const char* rx_version(void) { return rx::version(); }
