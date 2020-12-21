#include "util.hpp"

#define B_RED(S) "\033[1;31m" + S + "\033[0m"
#define B_CYAN(S) "\033[1;36m" + S + "\033[0m"

std::stringstream report(std::string const &rom, int cycles, int instructions) {
  std::stringstream ss;
  ss << B_RED(rom) << std::endl
     << "\t" << B_CYAN(std::string("C")) << ": " << +cycles << std::endl
     << "\t" << B_CYAN(std::string("I")) << ": " << +instructions;
  return ss;
}
