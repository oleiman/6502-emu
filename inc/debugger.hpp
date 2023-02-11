#pragma once

#include "cpu.hpp"
#include "instruction.hpp"
#include "memory.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace dbg {
class BreakPoint;

// TODO(oren): would be nice to provide the cpu type as a template parameter
// instead of hard coding
class Debugger {

public:
  using AddressT = cpu::M6502::AddressT;
  using DataT = cpu::M6502::DataT;
  explicit Debugger(bool should_break, bool trace);
  virtual ~Debugger() = default;

  virtual instr::Instruction const &step(instr::Instruction const &in,
                                         cpu::CpuState const &state,
                                         mem::Mapper &mapper);

private:
  bool break_;
  bool trace_;
  bool step_;
  int iter_;
  std::unique_ptr<instr::Instruction> prev_in_;
  std::vector<BreakPoint> breakpoints_;
  AddressT extract_addr(std::string const &command);
};

class BreakPoint {
  using AddressT = Debugger::AddressT;
  using DataT = Debugger::DataT;

public:
  explicit BreakPoint(AddressT pc) : pc_(pc) {}

  bool shouldBreak(instr::Instruction const &in) const;

private:
  AddressT pc_;
};

} // namespace dbg
