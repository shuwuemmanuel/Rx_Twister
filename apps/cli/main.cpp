// rxtwister command line tool.
#include "rx/rx.h"
#include "cli_args.h"
#include "core/util.h"
#include "core/par.h"
#include <cstdio>
#include <cstring>
#include <mutex>

int main(int argc, char** argv) {
  std::vector<std::string> args(argv + 1, argv + argc);
  if (args.empty() || args[0] == "-h" || args[0] == "--help" || args[0] == "help") { fputs(rx::usageText(), stdout); return args.empty() ? 1 : 0; }
  if (args[0] == "--version" || args[0] == "version") { printf("Rx Twister %s\n", rx::version()); return 0; }
  rx::Options o;
  std::vector<std::string> pos;
  std::string err;
  if (!rx::parseArgs(args, o, pos, err)) { fprintf(stderr, "error: %s\n(see rxtwister --help)\n", err.c_str()); return 2; }
  if (!pos.empty() && pos[0] == "formats") {
    auto f = rx::supportedFormats(o.blenderPath);
    printf("%-8s %-5s %-6s %-6s %-16s %s\n", "ext", "read", "write", "embed", "backend", "description");
    for (auto& x : f)
      printf("%-8s %-5s %-6s %-6s %-16s %s\n", x.ext.c_str(), x.canRead ? "yes" : "", x.canWrite ? "yes" : "",
             x.canWrite && x.embedsTextures ? "yes" : "", x.backend.c_str(), x.description.c_str());
    return 0;
  }
  if (!pos.empty() && pos[0] == "info") {
    if (pos.size() < 2) { fprintf(stderr, "usage: rxtwister info <file>\n"); return 2; }
    rx::setLogLevel(o.verbose ? rx::LogLevel::Verbose : rx::LogLevel::Info);
    rx::setThreadLimit(o.threads);
    rx::Scene s;
    rx::Timer t;
    if (!rx::loadScene(pos[1], s, o, err)) { fprintf(stderr, "error: %s\n", err.c_str()); return 1; }
    printf("%s", rx::describeScene(s).c_str());
    printf("load time  : %.2fs\n", t.seconds());
    return 0;
  }
  if (!pos.empty() && pos[0] == "convert") pos.erase(pos.begin());
  if (pos.empty()) { fprintf(stderr, "error: no input file\n"); return 2; }
  o.input = pos[0];
  if (pos.size() > 1 && o.output.empty()) o.output = pos[1];
  if (pos.size() > 2) { fprintf(stderr, "error: unexpected argument %s\n", pos[2].c_str()); return 2; }
  if (!o.quiet) {
    static std::string last;
    static std::mutex mu;
    o.onProgress = [](const std::string& stage, double) {
      std::lock_guard<std::mutex> lk(mu);
      std::string top = stage.substr(0, stage.find(':'));
      if (top != last && rx::logLevel() == rx::LogLevel::Verbose) { fprintf(stdout, "[%s]\n", stage.c_str()); }
      last = top;
    };
  }
  rx::Result r = rx::convert(o);
  if (!r.ok) { fprintf(stderr, "error: %s\n", r.error.c_str()); return 1; }
#if defined(__linux__)
  if (o.verbose) {
    if (FILE* f = fopen("/proc/self/status", "r")) {
      char line[256];
      while (fgets(line, sizeof line, f)) if (!strncmp(line, "VmHWM:", 6)) printf("  . peak memory: %s", line + 6);
      fclose(f);
    }
  }
#endif
  if (!o.quiet) {
    printf("%s %s -> %s  (%s -> %s, %zu -> %zu triangles, %.2fs)\n", o.dryRun ? "dry run:" : "done:", o.input.c_str(), r.output.c_str(),
           rx::humanBytes(double(r.bytesIn)).c_str(), rx::humanBytes(double(r.bytesOut)).c_str(), r.trianglesIn, r.trianglesOut, r.seconds);
  }
  return 0;
}
