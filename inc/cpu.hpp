#pragma once

#include "instruction.hpp"
#include "memory.hpp"

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace cpu {

struct CpuState {
  // TODO(oren): manual indicates that SP is initialized by
  // the programmer...
  explicit CpuState()
      : rA(0x00), rX(0x00), rY(0x00), sp(0xFF), pc(0x0000), status(0x0000) {}
  uint8_t rA;     // Accumulator
  uint8_t rX;     // Index Register X
  uint8_t rY;     // Index Reginster Y
  uint8_t sp;     // Stack Pointer
  uint16_t pc;    // Program Counter
  uint8_t status; // Status Regiser
};

class M6502 {
public:
  using AddressT = uint16_t;
  using DataT = uint8_t;
  using Callback = std::function<void(AddressT)>;

  explicit M6502(mem::Bus<AddressT> &abus, mem::Bus<DataT> &mbus,
                 std::function<void(void)> tick)
      : address_bus_(abus), data_bus_(mbus), tick_(tick) {}

  ~M6502() = default;
  bool loadRom(std::ifstream &infile, AddressT start);
  void initPc(AddressT val) { state_.pc = val; }
  AddressT pc() const { return state_.pc; }
  void step();

  template <class Debugger> void debugStep(Debugger &debugger) {
    // std::stringstream ss;
    // ss << std::hex << state_.pc << std::endl;
    // std::cerr << ss.str();
    DataT opcode = readByte(state_.pc);
    instr::Instruction in(
        opcode, state_.pc,
        std::bind(&M6502::calculateAddress, this, std::placeholders::_1));

    debugger.step(in, state_, address_bus_, data_bus_);

    fireCallbacks(in);
    state_.pc += in.size();
    dispatch(in);
  }

  void registerCallback(instr::Operation op, Callback c);

private:
  CpuState state_;

  mem::Bus<AddressT> &address_bus_;
  mem::Bus<DataT> &data_bus_;
  std::function<void(void)> tick_;

  AddressT code_start_;
  std::unordered_map<instr::Operation, std::vector<Callback>> callbacks_;

  void fireCallbacks(instr::Instruction const &in);
  DataT readByte(AddressT addr);
  void writeByte(AddressT addr, DataT data);

  AddressT calculateAddress(instr::AddressMode mode);

  // Compute offset address, tick clock if page boundary crossed
  AddressT penalizedOffset(AddressT base, uint8_t offset);
  AddressT wrapAroundOffset(AddressT base, uint8_t offset);
  void dispatch(instr::Instruction const &in);

  // TODO(oren): There's a good amount of code reuse here.
  // Good software engineering, not so good for the prospect
  // of migrating to a cycle-ticked architecture.
  void op_Illegal(DataT opcode);
  void op_LD(DataT &dest, AddressT source);
  void op_ST(AddressT dest, DataT data);
  void op_ADC(AddressT source);
  void op_SBC(AddressT source);
  void op_INC(AddressT source, int8_t val);
  void op_INR(DataT &reg, int8_t val);
  void op_ASL(AddressT source);
  void op_ASLV(DataT &val);
  void op_LSR(AddressT source);
  void op_LSRV(DataT &val);
  void op_ROL(AddressT source);
  void op_ROLV(DataT &val);
  void op_ROR(AddressT source);
  void op_RORV(DataT &val);
  void op_AND(AddressT source);
  void op_ORA(AddressT source);
  void op_EOR(AddressT source);
  void op_CMP(DataT reg, AddressT source);
  void op_BIT(AddressT source);
  void op_BRClear(uint8_t flag, AddressT target);
  void op_BRSet(uint8_t flag, AddressT target);
  void op_XFER(DataT source, DataT &dest);
  void op_TXS();
  void op_PHA();
  void op_PUSH(DataT source);
  void op_PLA();
  void op_JSR(AddressT target);
  void op_JMP(AddressT target);
  void op_RTS();
  void op_RTI();
  void op_PHP();
  void op_PLP();
  void op_BRK();
  void op_ClearFlag(uint8_t select);
  void op_SetFlag(uint8_t select);
  void op_NOP();

  void setOrClearStatus(bool pred, uint8_t mask);

  AddressT addr_Accumulator();
  AddressT addr_Immediate();
  AddressT addr_Implicit();
  AddressT addr_Absolute();
  AddressT addr_ZeroPage();
  AddressT addr_Relative();
  AddressT addr_Indirect();
  AddressT addr_AbsoluteX();
  AddressT addr_AbsoluteY();
  AddressT addr_ZeroPageX();
  AddressT addr_ZeroPageY();
  AddressT addr_IndexedIndirect(); // Indirect,X ($FF, X)
  AddressT addr_IndirectIndexed(); // Indirect,Y ($FF),Y
};

} // namespace cpu

inline std::string status_to_str(uint8_t status) {
  std::stringstream result;
  for (uint8_t mask = 0x80; mask > 0; mask /= 2) {
    // if (GET(status, mask))
    if (status & mask) {
      result << "1";
    } else {
      result << "0";
    }
  }
  return result.str();
}

inline std::ostream &operator<<(std::ostream &os, cpu::CpuState const &state) {
  std::stringstream ss;
  ss << "A:      0x" << std::hex << std::setfill('0') << std::setw(2)
     << +state.rA << std::endl
     << "X:      0x" << std::hex << std::setfill('0') << std::setw(2)
     << +state.rX << std::endl
     << "Y:      0x" << std::hex << std::setfill('0') << std::setw(2)
     << +state.rY << std::endl
     << "SP:     0x" << std::hex << std::setfill('0') << std::setw(2)
     << +state.sp << std::endl
     << "PC:     0x" << std::hex << std::setfill('0') << std::setw(4)
     << +state.pc << std::endl
     << "Status: " << status_to_str(state.status);

  os << ss.str();
  return os;
}
