#pragma once

#include "cpu.hpp"
#include "instruction.hpp"
#include "memory.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace dbg {
class BreakPoint;

class Debugger {
public:
  explicit Debugger(bool should_break = false);
  ~Debugger() = default;

  void step(instr::Instruction const &in, cpu::State const &state,
            mem::Bus<uint16_t> &address_bus, mem::Bus<uint8_t> &data_bus);

private:
  bool break_;
  bool step_;
  std::unique_ptr<instr::Instruction> prev_in_;
  std::vector<BreakPoint> breakpoints_;
  uint16_t extract_addr(std::string const &command);
};

class BreakPoint {
public:
  explicit BreakPoint(uint16_t pc) : pc_(pc) {}

  bool shouldBreak(instr::Instruction const &in) const;

private:
  uint16_t pc_;
};

} // namespace dbg
