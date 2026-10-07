// Command line option parsing shared by the CLI and the C API.
#pragma once
#include <string>
#include <vector>
#include "rx/options.h"

namespace rx {
// Parses option arguments (no positional handling). Returns false and sets err on bad input.
// Positional arguments are appended to `positional`.
bool parseArgs(const std::vector<std::string>& args, Options& o, std::vector<std::string>& positional, std::string& err);
std::vector<std::string> splitArgs(const std::string& s);
const char* usageText();
}  // namespace rx
