#pragma once

#include "instruction.hpp"
#include "memory.hpp"

#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stack>
#include <string>
#include <unordered_map>
#include <vector>

namespace dbg {
class Debugger;
}

namespace cpu {

struct CpuState {
  // TODO(oren): manual indicates that SP is initialized by
  // the programmer...
  explicit CpuState()
      : rA(0x00), rX(0x00), rY(0x00), sp(0x00), pc(0x0000), status(0x0000),
        cycle(0) {}
  uint8_t rA;               // Accumulator
  uint8_t rX;               // Index Register X
  uint8_t rY;               // Index Reginster Y
  uint8_t sp;               // Stack Pointer
  uint16_t pc;              // Program Counter
  uint8_t status;           // Status Regiser
  unsigned long long cycle; // cycle counter
};

class M6502 {
public:
  using AddressT = uint16_t;
  using DataT = uint8_t;
  // TODO(oren): Callback hack is used only for EhBASIC, and I think
  // we can cover this use case with the memory mapper abstraction.
  using Callback = std::function<void(AddressT)>;
  using TickHandler = std::function<void(void)>;

  explicit M6502(mem::Mapper &mapper, bool enable_bcd = true)
      : mapper_(mapper), enable_bcd_(enable_bcd) {}

  ~M6502() = default;
  bool loadRom(std::ifstream &infile, AddressT start);
  void initPc(AddressT val) { state_.pc = val; }
  uint8_t step();
  void reset(bool force = false);
  void reset(AddressT init);
  bool &nmiPin() { return pending_nmi_; }
  bool &irqPin() { return pending_irq_; }
  CpuState const &state() { return state_; }

  uint8_t debugStep(dbg::Debugger &debugger);

  void registerCallback(instr::Operation op, Callback c);

  void registerTickHandler(TickHandler f) { _tick_handlers.push_back(f); }

private:
  enum class IntSource {
    INSTRUCTION = 0x00,
    INTLINE = 0x01,
  };
  void tick() {
    auto nmi_prev = pending_nmi_;
    for (auto &h : _tick_handlers) {
      h();
    }
    ++step_cycles_;
    if (!nmi_prev && pending_nmi_) {
      nmi_cycle_ = state_.cycle + step_cycles_;
    }
  }

  std::vector<TickHandler> _tick_handlers;

  void disableInterrupts();
  void restoreInterrupts();

  struct FreezeState {
    FreezeState() = delete;
    FreezeState(CpuState &curr) : target_(curr), stored(curr) {}
    ~FreezeState() {
      target_.rA = stored.rA;
      target_.rY = stored.rY;
      target_.rX = stored.rX;
      target_.status = stored.status;
    }

  private:
    CpuState &target_;
    CpuState stored;
  };

  const static AddressT NMI_VEC = 0xFFFA;
  const static AddressT RST_VEC = 0xFFFC;
  const static AddressT IRQ_VEC = 0xFFFE;

  CpuState state_;
  uint8_t step_cycles_;

  mem::Mapper &mapper_;

  bool pending_reset_ = false;
  bool force_reset_ = false;
  bool pending_nmi_ = false;
  bool nmi_ready_ = false;
  bool pending_irq_ = false;
  bool irq_ready_ = false;
  unsigned long nmi_cycle_ = 0;

  std::stack<bool> iflag_prev_;
  bool enable_bcd_;
  bool rst_override_ = false;
  AddressT init_pc_ = 0;

  AddressT code_start_;
  std::unordered_map<instr::Operation, std::vector<Callback>> callbacks_;

  void fireCallbacks(instr::Instruction const &in);

  // TODO(oren): let's add primitives for reading a whole 16-bit address from
  // the bus. This would get rid of a _ton_ of repeated bit twiddling
  DataT readByte(AddressT addr);
  void writeByte(AddressT addr, DataT data);

  AddressT calculateAddress(instr::Instruction const &in);

  // Compute offset address, tick clock if page boundary crossed
  AddressT penalizedOffset(AddressT base, uint8_t offset);
  AddressT wrapAroundOffset(AddressT base, uint8_t offset);
  void dispatch(instr::Instruction const &in);

  // TODO(oren): There's a good amount of code reuse here.
  // Good software engineering, not so good for the prospect
  // of migrating to a cycle-ticked architecture.
  void op_Illegal(instr::Instruction const &in);
  void op_Interrupt(AddressT vec, IntSource src);
  void op_LD(DataT &dest, AddressT source);
  void op_ST(AddressT dest, DataT data);
  void op_ADC(AddressT source);
  void op_ADCV(DataT addend);
  void op_SBC(AddressT source);
  void op_SBCV(DataT subtrahend);
  DataT op_INC(AddressT source, int8_t val);
  void op_INR(DataT &reg, int8_t val);
  DataT op_ASL(AddressT source);
  void op_ASLV(DataT &val);
  DataT op_LSR(AddressT source);
  void op_LSRA();
  void op_LSRV(DataT &val);
  DataT op_ROL(AddressT source);
  void op_ROLV(DataT &val);
  DataT op_ROR(AddressT source);
  void op_RORA();
  void op_RORV(DataT &val);
  void op_AND(AddressT source);
  void op_ANDV(DataT val);
  void op_ORA(AddressT source);
  void op_ORAV(DataT val);
  void op_EOR(AddressT source);
  void op_EORV(DataT val);
  void op_CMP(DataT reg, AddressT source);
  void op_CMPV(DataT reg, DataT val);
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
  void op_DOP(AddressT target);

  void setOrClearStatus(bool pred, uint8_t mask);
  DataT doBinaryAdc(DataT reg, DataT addend);

  AddressT addr_Accumulator();
  AddressT addr_Immediate();
  AddressT addr_Implicit();
  AddressT addr_Absolute();
  AddressT addr_ZeroPage();
  AddressT addr_Relative();
  AddressT addr_Indirect();
  AddressT addr_AbsoluteX(instr::Operation op);
  AddressT addr_AbsoluteY(instr::Operation op);
  AddressT addr_ZeroPageX();
  AddressT addr_ZeroPageY();
  AddressT addr_IndexedIndirect();                    // Indirect,X ($FF, X)
  AddressT addr_IndirectIndexed(instr::Operation op); // Indirect,Y ($FF),Y
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
