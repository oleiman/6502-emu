#include "cpu.hpp"
#include "debugger.hpp"
#include "instruction.hpp"

#include <bitset>
#include <iostream>

// bit manipulation
#define SET(X, MASK) (X |= MASK)
#define GET(X, MASK) (X & MASK)
#define CLEAR(X, MASK) (X &= ~MASK)

// base address for stack
#define STACK_BASE 0x0100
#define STACK_POINTER(X) (STACK_BASE | X)

namespace cpu {

constexpr uint8_t BIT0 = 0b00000001u;
constexpr uint8_t BIT1 = 0b00000010u;
constexpr uint8_t BIT2 = 0b00000100u;
constexpr uint8_t BIT3 = 0b00001000u;
constexpr uint8_t BIT4 = 0b00010000u;
constexpr uint8_t BIT5 = 0b00100000u;
constexpr uint8_t BIT6 = 0b01000000u;
constexpr uint8_t BIT7 = 0b10000000u;

// status byte masks
constexpr uint8_t CARRY_M = BIT0;
constexpr uint8_t ZERO_M = BIT1;
constexpr uint8_t INT_DISABLE_M = BIT2;
constexpr uint8_t DECIMAL_M = BIT3;
constexpr uint8_t BIT4_M = BIT4;
constexpr uint8_t BIT5_M = BIT5;
constexpr uint8_t OVERFLOW_M = BIT6;
constexpr uint8_t NEGATIVE_M = BIT7;

bool M6502::loadRom(std::ifstream &infile, M6502::AddressT start) {
  code_start_ = start;
  if (!infile) {
    std::cerr << "Bad ROM File!" << std::endl;
    return false;
  }
  char next;
  uint32_t curr = code_start_;
  while (infile) {
    infile.get(next);
    try {
      mapper_.write(static_cast<AddressT>(curr++), next);
    } catch (std::runtime_error &e) {
      std::cerr << e.what() << std::endl;
      return false;
    }
  }
  return true;
}

// TODO(oren): Reset should restore CPU to reset state, currently does nothing
// but set reset line
void M6502::reset(bool force) {
  pending_reset_ = true;
  rst_override_ = false;
  force_reset_ = force;
}

void M6502::reset(AddressT init) {
  pending_reset_ = true;
  // TODO(oren): pair...
  rst_override_ = true;
  init_pc_ = init;
}

uint8_t M6502::step() {
  step_cycles_ = 0;
  if (nmi_ready_) {
    op_Interrupt(NMI_VEC, IntSource::INTLINE);
    pending_nmi_ = false;
  } else if (pending_reset_) {
    op_Interrupt(RST_VEC, IntSource::INTLINE);
    pending_reset_ = false;
    force_reset_ = false;
  } else if (irq_ready_) {
    op_Interrupt(IRQ_VEC, IntSource::INTLINE);
  }

  bool irq_b4 = pending_irq_;
  bool nmi_b4 = pending_nmi_;

  dispatch([&]() {
    instr::Instruction in(readByte(state_.pc), state_.pc,
                          state_.cycle + step_cycles_);
    in.address = calculateAddress(in);
    return in;
  }());

  irq_ready_ = pending_irq_ && irq_b4;
  nmi_ready_ = pending_nmi_ && nmi_b4;
  state_.cycle += step_cycles_;
  return step_cycles_;
}

uint8_t M6502::debugStep(dbg::Debugger &debugger) {
  step_cycles_ = 0;
  if (nmi_ready_) {
    op_Interrupt(NMI_VEC, IntSource::INTLINE);
    pending_nmi_ = false;
  } else if (pending_reset_) {
    op_Interrupt(RST_VEC,
                 (force_reset_ ? IntSource::INSTRUCTION : IntSource::INTLINE));
    pending_reset_ = false;
    force_reset_ = false;
  } else if (irq_ready_) {
    op_Interrupt(IRQ_VEC, IntSource::INTLINE);
  }

  bool irq_b4 = pending_irq_;
  bool nmi_b4 = pending_nmi_;

  dispatch(debugger.step(
      [&]() {
        instr::Instruction in(readByte(state_.pc), state_.pc,
                              state_.cycle + step_cycles_);
        in.address = calculateAddress(in);
        return in;
      }(),
      state_, mapper_));

  irq_ready_ = pending_irq_ && irq_b4;
  nmi_ready_ = pending_nmi_ && nmi_b4;
  state_.cycle += step_cycles_;
  return step_cycles_;
}

void M6502::registerCallback(instr::Operation op, Callback c) {
  callbacks_[op].push_back(c);
}

void M6502::fireCallbacks(instr::Instruction const &in) {
  if (callbacks_.count(in.operation) > 0) {
    for (auto &c : callbacks_[in.operation]) {
      c(in.address);
    }
  }
}

M6502::DataT M6502::readByte(AddressT addr) {
  auto b = mapper_.read(addr);
  tick();
  return b;
}

void M6502::writeByte(AddressT addr, DataT data) {
  mapper_.write(addr, data);
  tick();
}

void M6502::disableInterrupts() {
  iflag_prev_.push(GET(state_.status, INT_DISABLE_M));
  SET(state_.status, INT_DISABLE_M);
}

// NOTE(oren): expectation for this is not well understood
void M6502::restoreInterrupts() {
  if (iflag_prev_.empty() || !iflag_prev_.top()) {
    CLEAR(state_.status, INT_DISABLE_M);
  } else {
    SET(state_.status, INT_DISABLE_M);
  }
  // NOTE(oren): sketchy...do i need a stack here or something?
  if (!iflag_prev_.empty()) {
    iflag_prev_.pop();
  }
}

M6502::AddressT M6502::calculateAddress(instr::Instruction const &in) {
  using instr::AddressMode;
  switch (in.addressMode) {
  case AddressMode::accumulator:
    return addr_Accumulator();
  case AddressMode::immediate:
    return addr_Immediate();
  case AddressMode::absolute:
    return addr_Absolute();
  case AddressMode::absoluteX:
    return addr_AbsoluteX(in.operation);
  case AddressMode::absoluteY:
    return addr_AbsoluteY(in.operation);
  case AddressMode::zeroPage:
    return addr_ZeroPage();
  case AddressMode::zeroPageX:
    return addr_ZeroPageX();
  case AddressMode::zeroPageY:
    return addr_ZeroPageY();
  case AddressMode::relative:
    return addr_Relative();
  case AddressMode::indirect:
    return addr_Indirect();
  case AddressMode::indexedIndirect:
    return addr_IndexedIndirect();
  case AddressMode::indirectIndexed:
    return addr_IndirectIndexed(in.operation);
  default:
    return addr_Implicit();
  }
}

M6502::AddressT M6502::penalizedOffset(AddressT base, uint8_t offset) {
  AddressT result = base + offset;

  // crossing page boundary?
  if ((base >> 8) ^ (result >> 8)) {
    tick();
  }
  return result;
}

M6502::AddressT M6502::wrapAroundOffset(AddressT base, uint8_t offset) {
  AddressT result = base + offset;
  result &= 0xFF;
  tick();
  return result;
}

void M6502::dispatch(instr::Instruction const &in) {
  if (in.discard) {
    return;
  }

  state_.pc += in.size;
  using instr::Operation;
  switch (in.operation) {
  case Operation::LDA:
    op_LD(state_.rA, in.address);
    break;
  case Operation::LDX:
    op_LD(state_.rX, in.address);
    break;
  case Operation::LDY:
    op_LD(state_.rY, in.address);
    break;
  case Operation::LAX:
    op_LD(state_.rA, in.address);
    state_.rX = state_.rA;
    break;
  case Operation::LAS:
    op_LAS(in.address);
    break;
  case Operation::STA:
    op_ST(in.address, state_.rA);
    break;
  case Operation::STX:
    op_ST(in.address, state_.rX);
    break;
  case Operation::STY:
    op_ST(in.address, state_.rY);
    break;
  case Operation::SAX:
    op_ST(in.address, state_.rA & state_.rX);
    break;
  case Operation::ADC:
    op_ADC(in.address);
    break;
  case Operation::SBC:
    op_SBC(in.address);
    break;
  case Operation::INC:
    op_INC(in.address, 1);
    break;
  case Operation::INX:
    op_INR(state_.rX, 1);
    break;
  case Operation::INY:
    op_INR(state_.rY, 1);
    break;
  case Operation::ISC:
    op_SBCV(op_INC(in.address, 1));
    break;
  case Operation::DEC:
    op_INC(in.address, -1);
    break;
  case Operation::DEX:
    op_INR(state_.rX, -1);
    break;
  case Operation::DEY:
    op_INR(state_.rY, -1);
    break;
  case Operation::DCP:
    op_CMPV(state_.rA, op_INC(in.address, -1));
    break;
  case Operation::ASL:
    op_ASL(in.address);
    break;
  case Operation::ASLA:
    op_ASLV(state_.rA);
    break;
  case Operation::SLO:
    op_ORAV(op_ASL(in.address));
    break;
  case Operation::LSR:
    op_LSR(in.address);
    break;
  case Operation::LSRA:
    op_LSRA();
    break;
  case Operation::SRE:
    op_EORV(op_LSR(in.address));
    break;
  case Operation::ROL:
    op_ROL(in.address);
    break;
  case Operation::ROLA:
    op_ROLV(state_.rA);
    break;
  case Operation::RLA:
    op_ANDV(op_ROL(in.address));
    break;
  case Operation::ROR:
    op_ROR(in.address);
    break;
  case Operation::RORA:
    op_RORA();
    break;
  case Operation::RRA:
    op_ADCV(op_ROR(in.address));
    break;
  case Operation::AND:
    op_AND(in.address);
    break;
  case Operation::ANC:
    op_AND(in.address);
    setOrClearStatus(GET(state_.rA, NEGATIVE_M), CARRY_M);
    break;
  case Operation::ALR:
    op_AND(in.address);
    op_LSRV(state_.rA);
    break;
  case Operation::ARR:
    op_ARR(in.address);
    break;
  case Operation::XAA:
    state_.rA = state_.rX;
    op_AND(in.address);
    break;
  case Operation::AXA: {
    FreezeState s(state_);
    op_ANDV(state_.rX);
    // AND with unindexed address
    // NOTE(oren): assuming that for the indirect indexed case the "target"
    // address refers to the address _stored_ at the zero-page pointer
    // possible that this should be 00 + 1 (as in upper byte of zero page addr)
    op_ANDV((((in.address - state_.rY) >> 8) & 0xFF) + 1);
    op_ST(in.address, state_.rA);
  } break;
  case Operation::AXS:
    op_AXS(readByte(in.address));
    break;
  case Operation::TAS: {
    FreezeState s(state_);
    op_ANDV(state_.rX);
    state_.sp = state_.rA;
    op_ANDV((((in.address - state_.rY) >> 8) & 0xFF) + 1);
    op_ST(in.address, state_.rA);
  } break;
  case Operation::SAY:
    op_SAY(in.address);
    break;
  case Operation::XAS: {
    op_XAS(in.address);
  } break;
  case Operation::ORA:
    op_ORA(in.address);
    break;
  case Operation::EOR:
    op_EOR(in.address);
    break;
  case Operation::CMP:
    op_CMP(state_.rA, in.address);
    break;
  case Operation::CPX:
    op_CMP(state_.rX, in.address);
    break;
  case Operation::CPY:
    op_CMP(state_.rY, in.address);
    break;
  case Operation::BIT:
    op_BIT(in.address);
    break;
  case Operation::BPL:
    op_BRClear(NEGATIVE_M, in.address);
    break;
  case Operation::BVC:
    op_BRClear(OVERFLOW_M, in.address);
    break;
  case Operation::BCC:
    op_BRClear(CARRY_M, in.address);
    break;
  case Operation::BNE:
    op_BRClear(ZERO_M, in.address);
    break;
  case Operation::BMI:
    op_BRSet(NEGATIVE_M, in.address);
    break;
  case Operation::BVS:
    op_BRSet(OVERFLOW_M, in.address);
    break;
  case Operation::BCS:
    op_BRSet(CARRY_M, in.address);
    break;
  case Operation::BEQ:
    op_BRSet(ZERO_M, in.address);
    break;
  case Operation::TAX:
    op_XFER(state_.rA, state_.rX);
    break;
  case Operation::TAY:
    op_XFER(state_.rA, state_.rY);
    break;
  case Operation::TSX:
    op_XFER(state_.sp, state_.rX);
    break;
  case Operation::TXA:
    op_XFER(state_.rX, state_.rA);
    break;
  case Operation::TXS:
    op_TXS();
    break;
  case Operation::TYA:
    op_XFER(state_.rY, state_.rA);
    break;
  case Operation::JMP:
    state_.pc = in.address;
    break;
  case Operation::JSR:
    op_JSR(in.address);
    break;
  case Operation::RTS:
    op_RTS();
    break;
  case Operation::RTI:
    op_RTI();
    break;
  case Operation::PHA:
    op_PHA();
    break;
  case Operation::PLA:
    op_PLA();
    break;
  case Operation::PHP:
    op_PHP();
    break;
  case Operation::PLP:
    op_PLP();
    break;
  case Operation::BRK:
    op_BRK();
    break;
  case Operation::CLC:
    op_ClearFlag(CARRY_M);
    break;
  case Operation::CLI:
    op_ClearFlag(INT_DISABLE_M);
    break;
  case Operation::CLV:
    op_ClearFlag(OVERFLOW_M);
    break;
  case Operation::CLD:
    op_ClearFlag(DECIMAL_M);
    break;
  case Operation::SEC:
    op_SetFlag(CARRY_M);
    break;
  case Operation::SEI:
    op_SetFlag(INT_DISABLE_M);
    break;
  case Operation::SED:
    op_SetFlag(DECIMAL_M);
    break;
  case Operation::NOP:
    op_NOP();
    break;
  case Operation::DOP:
    op_DOP(in.address);
    break;
  default:
    op_Illegal(in);
    break;
  }
}

void M6502::op_Illegal(instr::Instruction const &in) {
  std::stringstream ss;
  ss << "Illegal Opcode: " << in;
  // ss << "[0x" << std::hex << state_.pc << "] "
  //    << "Illegal Opcode: <0x" << std::hex << std::setfill('0') <<
  //    std::setw(2)
  //    << +opcode << ">";
  throw std::runtime_error(ss.str());
}

// TODO(oren): a bit gross, but adding NMI and RESET fairly easily
// technically reset would consume the 7 cycles with dummy stack reads
// but it shouldn't be timing critical so who care
void M6502::op_Interrupt(AddressT vec, IntSource src) {
  // NOTE(oren): Coding to Klaus test...am i reading the spec correctly?

  // INT_DISABLE cuases all non-NMI interrupts to be ignored
  // IntSource::INSTRUCTION indicates a BRK triggered interrupt,
  // which is non-maskable but targets the IRQ vector
  if (vec != NMI_VEC && src != IntSource::INSTRUCTION &&
      GET(state_.status, INT_DISABLE_M)) {
    return;
  }

  // TODO(oren): gross hack to ensure exact return address is pushed onto the
  // stack
  if (vec == IRQ_VEC && src == IntSource::INSTRUCTION) {
    ++state_.pc;
  }
  if (vec != RST_VEC) {
    AddressT returnAddr = state_.pc;
    DataT pc_hi = (returnAddr >> 8) & 0xFF;
    DataT pc_lo = returnAddr & 0xFF;
    op_PUSH(pc_hi); // tick 2
    op_PUSH(pc_lo); // tick 3

    DataT tmp = state_.status;
    SET(tmp, BIT5_M);
    if (src == IntSource::INSTRUCTION) {
      SET(tmp, BIT4_M);
    } else {
      CLEAR(tmp, BIT4_M);
    }
    op_PUSH(tmp); // tick 4
  } else {
    state_ = CpuState();
    tick();
    op_PUSH(0x00);
    op_PUSH(0x00);
    op_PUSH(0x00);
  }

  DataT target_lo = readByte(vec);                               // tick 5
  AddressT target_hi = static_cast<AddressT>(readByte(vec + 1)); // tick 6
  AddressT target = (target_hi << 8) | target_lo;
  if (vec != RST_VEC) {
    // TODO(oren): disable on NMI or just IRQ?
    disableInterrupts();
  } else if (rst_override_) {
    target = init_pc_;
  }
  op_JMP(target); // tick 7
}

// load
void M6502::op_LD(DataT &dest, AddressT source) {
  dest = readByte(source);
  setOrClearStatus(GET(dest, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(dest == 0, ZERO_M);
}

void M6502::op_LAS(AddressT source) {
  state_.rA = state_.sp;
  op_AND(source);
  state_.rX = state_.rA;
  state_.sp = state_.rA;
}

// store
void M6502::op_ST(AddressT dest, DataT data) {
  writeByte(dest, data);

  // TODO(oren): indexed stores ALWAYS get the one cycle penalty, regardless of
  // page boundary tick_();
}

// add memory to accumulator with carry
void M6502::op_ADC(AddressT source) {
  DataT addend = readByte(source);
  op_ADCV(addend);
}

void M6502::op_ADCV(DataT addend) {
  // TODO(oren): not used? logic error?
  // DataT input_carry = GET(_state.status, CARRY_M);
  uint16_t result = 0;

  if (GET(state_.status, DECIMAL_M) && enable_bcd_) {
    DataT accum_hi = (state_.rA & 0xF0) >> 4;
    DataT accum_lo = state_.rA & 0x0F;
    DataT accum_dec = accum_hi * 10 + accum_lo;

    DataT addend_hi = (addend & 0xF0) >> 4;
    DataT addend_lo = (addend & 0x0F);
    DataT addend_dec = addend_hi * 10 + addend_lo + GET(state_.status, CARRY_M);

    if (accum_dec > 99 || addend_dec > 100) {
      return;
    }
    accum_dec += addend_dec;
    setOrClearStatus(accum_dec > 99, CARRY_M);
    accum_dec %= 100;
    result = ((accum_dec / 10) << 4) | (accum_dec % 10);
    setOrClearStatus(result == 0, ZERO_M);
    setOrClearStatus(GET(result, NEGATIVE_M), NEGATIVE_M);
  } else {
    result = doBinaryAdc(state_.rA, addend);
  }
  state_.rA = static_cast<DataT>(result);
}

// subtract memory from accumulator with borrow
void M6502::op_SBC(AddressT source) {
  // take two's complement with borrow
  DataT subtrahend = readByte(source);
  op_SBCV(subtrahend);
}

void M6502::op_SBCV(DataT subtrahend) {
  // TODO(oren): not used? logic error?
  // DataT input_carry = GET(_state.status, CARRY_M);
  uint16_t result = 0;

  if (GET(state_.status, DECIMAL_M) && enable_bcd_) {

    DataT accum_hi = (state_.rA & 0xF0) >> 4;
    DataT accum_lo = state_.rA & 0x0F;
    DataT accum_dec = accum_hi * 10 + accum_lo;

    DataT subtrahend_hi = (subtrahend & 0xF0) >> 4;
    DataT subtrahend_lo = (subtrahend & 0x0F);
    DataT subtrahend_dec = subtrahend_hi * 10 + subtrahend_lo;

    subtrahend_dec += GET(state_.status, CARRY_M) ? 0 : 1;

    if (accum_dec > 99 || subtrahend_dec > 100) {
      return;
    }

    setOrClearStatus(accum_dec >= subtrahend_dec, CARRY_M);
    if (accum_dec >= subtrahend_dec) {
      accum_dec -= subtrahend_dec;
    } else {
      accum_dec = 100 - (subtrahend_dec - accum_dec);
    }
    result = ((accum_dec / 10) << 4) | (accum_dec % 10);
  } else {
    result = doBinaryAdc(state_.rA, ~subtrahend);
  }
  state_.rA = static_cast<DataT>(result);
}

// increment memory
M6502::DataT M6502::op_INC(AddressT source, int8_t val) {
  DataT result = readByte(source);
  writeByte(source, result); // phantom write
  result += val;
  setOrClearStatus(GET(result, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(result == 0, ZERO_M);
  writeByte(source, result);
  return result;
}

// increment register
void M6502::op_INR(DataT &reg, int8_t val) {
  reg += val;
  setOrClearStatus(GET(reg, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(reg == 0, ZERO_M);
  tick();
}

// arithmetic shift left (read/modify/write)
M6502::DataT M6502::op_ASL(AddressT source) {
  DataT result = readByte(source);
  // writeByte(source, result);
  op_ASLV(result);
  writeByte(source, result);
  return result;
}

void M6502::op_ASLV(DataT &val) {
  setOrClearStatus(GET(val, NEGATIVE_M), CARRY_M);
  val <<= 1;
  setOrClearStatus(GET(val, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(val == 0, ZERO_M);
  tick();
}

// logical shift right on memory
M6502::DataT M6502::op_LSR(AddressT source) {
  DataT result = readByte(source);
  // dummy write
  // writeByte(source, result);
  op_LSRV(result);
  tick();
  writeByte(source, result);
  return result;
}

void M6502::op_LSRA() {
  op_LSRV(state_.rA);
  tick();
}

// logical shift right on value
void M6502::op_LSRV(DataT &val) {
  setOrClearStatus(GET(val, CARRY_M), CARRY_M);
  val >>= 1;
  setOrClearStatus(GET(val, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(val == 0, ZERO_M);
}

// rotate left on memory
M6502::DataT M6502::op_ROL(AddressT source) {
  DataT result = readByte(source);
  op_ROLV(result);
  writeByte(source, result);
  return result;
}

// rotate left on value
void M6502::op_ROLV(DataT &val) {
  DataT input_carry = GET(state_.status, CARRY_M);
  setOrClearStatus(GET(val, NEGATIVE_M), CARRY_M);
  val <<= 1;
  if (input_carry) {
    SET(val, CARRY_M);
  }
  setOrClearStatus(GET(val, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(val == 0, ZERO_M);
  tick();
}

// rotate right on memory
M6502::DataT M6502::op_ROR(AddressT source) {
  DataT result = readByte(source);
  op_RORV(result);
  tick();
  writeByte(source, result);
  return result;
}

void M6502::op_RORA() {
  op_RORV(state_.rA);
  tick();
}

void M6502::op_RORV(DataT &val) {
  DataT input_carry = GET(state_.status, CARRY_M);
  setOrClearStatus(GET(val, CARRY_M), CARRY_M);
  val >>= 1;
  if (input_carry) {
    SET(val, NEGATIVE_M);
  }
  setOrClearStatus(GET(val, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(val == 0, ZERO_M);
}

// AND memory with accumulator
void M6502::op_AND(AddressT source) { op_ANDV(readByte(source)); }
void M6502::op_ANDV(DataT val) {
  state_.rA &= val;
  setOrClearStatus(GET(state_.rA, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(state_.rA == 0, ZERO_M);
}

void M6502::op_ARR(AddressT source) {
  op_AND(source);
  op_RORV(state_.rA);
  uint8_t b5 = GET(state_.rA, BIT5);
  uint8_t b6 = GET(state_.rA, BIT6);
  /*
    If both bits are 1: set C, clear V.
    If both bits are 0: clear C and V.
    If only bit 5 is 1: set V, clear C.
    If only bit 6 is 1: set C and V.
   */
  if (b5 && b6) {
    SET(state_.status, CARRY_M);
    CLEAR(state_.status, OVERFLOW_M);
  } else if (!b5 && !b6) {
    CLEAR(state_.status, CARRY_M);
    CLEAR(state_.status, OVERFLOW_M);
  } else if (b5) {
    CLEAR(state_.status, CARRY_M);
    SET(state_.status, OVERFLOW_M);
  } else if (b6) {
    SET(state_.status, CARRY_M);
    SET(state_.status, OVERFLOW_M);
  }
}

void M6502::op_AXS(DataT val) {
  state_.rX &= state_.rA;
  op_CMPV(state_.rX, val);
  state_.rX -= val;
}

void M6502::op_SAY(AddressT target) {
  FreezeState s(state_);
  auto base = target - state_.rX;
  AddressT value = (base & 0xFF00) | (target & 0x00FF);
  bool page_crossed = ((value >> 8) ^ (target >> 8));
  if (page_crossed) {
    value &= (state_.rX << 8);
  }
  state_.rA = state_.rY;
  op_ANDV(((value >> 8) & 0xFF) + 1);
  op_ST(value, state_.rA);
}

void M6502::op_XAS(AddressT target) {
  FreezeState s(state_);
  auto base = target - state_.rY;
  AddressT value = (base & 0xFF00) | (target & 0x00FF);
  bool page_crossed = ((value >> 8) ^ (target >> 8));
  if (page_crossed) {
    value &= (state_.rY << 8);
  }
  state_.rA = state_.rX;
  op_ANDV(((value >> 8) & 0xFF) + 1);
  op_ST(value, state_.rA);
}

// OR memory with accumulator
void M6502::op_ORA(AddressT source) { op_ORAV(readByte(source)); }
void M6502::op_ORAV(DataT val) {
  state_.rA |= val;
  setOrClearStatus(GET(state_.rA, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(state_.rA == 0, ZERO_M);
}

// XOR memory with accumulator
void M6502::op_EOR(AddressT source) { op_EORV(readByte(source)); }
void M6502::op_EORV(DataT val) {
  state_.rA ^= val;
  setOrClearStatus(GET(state_.rA, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(state_.rA == 0, ZERO_M);
}

// compare memory and register
void M6502::op_CMP(DataT reg, AddressT source) {
  DataT val = readByte(source);
  op_CMPV(reg, val);
}

void M6502::op_CMPV(DataT reg, DataT val) {
  if (reg != val) {
    uint8_t tmp = reg - val;
    setOrClearStatus(GET(tmp, NEGATIVE_M), NEGATIVE_M);
  } else {
    CLEAR(state_.status, NEGATIVE_M);
  }

  if (reg < val) {
    // SET(state_.status, NEGATIVE_M);
    CLEAR(state_.status, ZERO_M);
    CLEAR(state_.status, CARRY_M);
  } else if (reg == val) {
    // CLEAR(state_.status, NEGATIVE_M);
    SET(state_.status, ZERO_M);
    SET(state_.status, CARRY_M);
  } else if (reg > val) {
    // CLEAR(state_.status, NEGATIVE_M);
    CLEAR(state_.status, ZERO_M);
    SET(state_.status, CARRY_M);
  }
}

// test bits in memory with accumulator
void M6502::op_BIT(AddressT source) {
  DataT acc = state_.rA;
  DataT mem = readByte(source);
  setOrClearStatus(GET(mem, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(GET(mem, OVERFLOW_M), OVERFLOW_M);
  setOrClearStatus((acc & mem) == 0, ZERO_M);
}

// branch if flag is clear
void M6502::op_BRClear(uint8_t flag, AddressT target) {
  if (GET(state_.status, flag) == 0) {
    // page crossing penalty
    if ((state_.pc >> 8) ^ (target >> 8)) {
      tick();
    }
    op_JMP(target);
  }
}

// branch if flag is set
void M6502::op_BRSet(uint8_t flag, AddressT target) {
  if (GET(state_.status, flag)) {
    // page crossing penalty
    if ((state_.pc >> 8) ^ (target >> 8)) {
      tick();
    }
    op_JMP(target);
  }
}

void M6502::op_TXS() {
  state_.sp = state_.rX;
  tick();
}

// transfer from source to dest
void M6502::op_XFER(DataT source, DataT &dest) {
  dest = source;
  setOrClearStatus(GET(dest, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(dest == 0, ZERO_M);
  tick();
}

void M6502::op_PHA() {
  op_PUSH(state_.rA);
  tick();
}

// push value onto stack
void M6502::op_PUSH(DataT source) {
  writeByte(STACK_POINTER(state_.sp), source);
  state_.sp--;
  // if (state_.sp == 0xFF) {
  //   std::cerr << "WARNING: Stack overflow" << std::endl;
  // }
}

// pop from the stack into dest
void M6502::op_PLA() {
  state_.sp++;
  tick();
  state_.rA = readByte(STACK_POINTER(state_.sp));
  setOrClearStatus(GET(state_.rA, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(state_.rA == 0, ZERO_M);
  tick();
}

// jump to subroutine
void M6502::op_JSR(AddressT target) {
  // note that we push the pc of the last byte of the
  // jump instruction
  AddressT returnAddr = state_.pc - 1;
  DataT ra_hi = (returnAddr >> 8) & 0xFF;
  DataT ra_lo = returnAddr & 0xFF;
  op_PUSH(ra_hi);
  op_PUSH(ra_lo);
  op_JMP(target);
}

// jump and tick the clock
void M6502::op_JMP(AddressT target) {
  state_.pc = target;
  tick();
}

// return from subroutine
void M6502::op_RTS() {
  state_.sp++;
  // TODO(oren): consider consequences of ticking here rather than at the end
  tick();                                                 // tick 2
  DataT target_lo = readByte(STACK_POINTER(state_.sp++)); // tick 3
  AddressT target_hi =
      static_cast<AddressT>(readByte(STACK_POINTER(state_.sp))); // tick 4
  op_JMP((target_hi << 8) | target_lo);
  state_.pc++;
  tick();
}

// return from interrupt
void M6502::op_RTI() {
  state_.sp++;
  // TODO(oren): consider consequences of ticking here rather than at the end
  tick();                                                 // tick 2
  state_.status = readByte(STACK_POINTER(state_.sp++));   // tick 3
  DataT target_lo = readByte(STACK_POINTER(state_.sp++)); // tick 4
  AddressT target_hi =
      static_cast<AddressT>(readByte(STACK_POINTER(state_.sp))); // tick 5
  // clear interrupt disable flag
  // NOTE(oren): spec says "restore to previous state"
  // However...if I do this it fails some instruction tests
  // restoreInterrupts();
  // no pc increment here, address on stack is precise return address
  op_JMP((target_hi << 8) | target_lo); // tick 6
}

void M6502::op_PHP() {
  DataT tmp = state_.status;
  SET(tmp, BIT5_M);
  SET(tmp, BIT4_M);
  tick();       // tick 2
  op_PUSH(tmp); // tick 3
}

void M6502::op_PLP() {
  state_.sp++;
  tick();                                         // tick 2
  DataT tmp = readByte(STACK_POINTER(state_.sp)); // tick 3
  // ignore the "B Flag"
  CLEAR(tmp, BIT5_M);
  CLEAR(tmp, BIT4_M);
  state_.status = tmp;
  tick(); // tick 4
}

// force interrupt
void M6502::op_BRK() { op_Interrupt(IRQ_VEC, IntSource::INSTRUCTION); }

void M6502::op_ClearFlag(uint8_t select) {
  CLEAR(state_.status, select);
  tick();
}

void M6502::op_SetFlag(uint8_t select) {
  SET(state_.status, select);
  tick();
}

void M6502::op_NOP() { tick(); }
void M6502::op_DOP(AddressT target) { readByte(target); }

void M6502::setOrClearStatus(bool pred, uint8_t mask) {
  if (pred) {
    SET(state_.status, mask);
  } else {
    CLEAR(state_.status, mask);
  }
}

M6502::DataT M6502::doBinaryAdc(DataT reg, DataT addend) {
  uint16_t result;
  result = static_cast<uint16_t>(reg);
  result += static_cast<uint16_t>(addend);
  result += GET(state_.status, CARRY_M);
  setOrClearStatus(result & 0xFF00, CARRY_M);

  // mask off the carry
  result &= 0xFF;

  setOrClearStatus(GET(result, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(result == 0, ZERO_M);

  if (GET(reg, NEGATIVE_M) == GET(addend, NEGATIVE_M) &&
      GET(reg, NEGATIVE_M) != GET(result, NEGATIVE_M)) {
    SET(state_.status, OVERFLOW_M);
  } else {
    CLEAR(state_.status, OVERFLOW_M);
  }
  return static_cast<DataT>(result);
}

M6502::AddressT M6502::addr_Implicit() {
  // not used
  return 0;
}

M6502::AddressT M6502::addr_Accumulator() {
  // not used
  return 0;
}

// immediate adressing
// read from the very next byte relative to current PC
M6502::AddressT M6502::addr_Immediate() { return state_.pc + 1; }

M6502::AddressT M6502::addr_Absolute() {
  DataT addr_lo = readByte(state_.pc + 1);
  AddressT addr_hi = static_cast<AddressT>(readByte(state_.pc + 2));
  return (addr_hi << 8) | addr_lo;
}

M6502::AddressT M6502::addr_ZeroPage() {
  AddressT addr = static_cast<AddressT>(readByte(state_.pc + 1));
  return addr;
}

M6502::AddressT M6502::addr_Relative() {
  int8_t offset = static_cast<int8_t>(readByte(state_.pc + 1));
  AddressT base = state_.pc + 2;
  AddressT target = base + offset;
  if (offset < 0) {
    offset = -offset;
    target = base - static_cast<uint8_t>(offset);
  }
  return target;
}

M6502::AddressT M6502::addr_Indirect() {
  DataT i_addr_lo = readByte(state_.pc + 1);
  AddressT i_addr_hi = static_cast<AddressT>(readByte(state_.pc + 2));
  AddressT i_addr = (i_addr_hi << 8) | i_addr_lo;
  DataT addr_lo = readByte(i_addr);

  if ((i_addr | 0xFF) == i_addr) {
    i_addr &= 0xFF00;
  } else {
    i_addr++;
  }

  AddressT addr_hi = static_cast<AddressT>(readByte(i_addr));
  AddressT addr = (addr_hi << 8) | addr_lo;
  return addr;
}

M6502::AddressT M6502::addr_AbsoluteX(instr::Operation op) {
  using instr::Operation;
  DataT addr_lo = readByte(state_.pc + 1);
  AddressT addr_hi = static_cast<AddressT>(readByte(state_.pc + 2));
  AddressT base = (addr_hi << 8) | addr_lo;

  if (op == Operation::ASL || op == Operation::LSR || op == Operation::ROL ||
      op == Operation::ROR || op == Operation::DEC || op == Operation::INC ||
      op == Operation::STA || op == Operation::SAY || op == Operation::ISC ||
      op == Operation::DCP || op == Operation::SLO || op == Operation::RLA ||
      op == Operation::SRE || op == Operation::RRA) {
    tick();
    return base + state_.rX;
  }
  return penalizedOffset(base, state_.rX);
}

M6502::AddressT M6502::addr_AbsoluteY(instr::Operation op) {
  using instr::Operation;
  DataT addr_lo = readByte(state_.pc + 1);
  AddressT addr_hi = static_cast<AddressT>(readByte(state_.pc + 2));
  AddressT base = (addr_hi << 8) | addr_lo;

  if (op == Operation::STA || op == Operation::ISC || op == Operation::DCP ||
      op == Operation::SLO || op == Operation::RLA || op == Operation::SRE ||
      op == Operation::RRA || op == Operation::AXA || op == Operation::TAS ||
      op == Operation::XAS) {
    tick();
    return base + state_.rY;
  }
  return penalizedOffset(base, state_.rY);
}

M6502::AddressT M6502::addr_ZeroPageX() {
  AddressT base = static_cast<AddressT>(readByte(state_.pc + 1));
  return wrapAroundOffset(base, state_.rX);
}

M6502::AddressT M6502::addr_ZeroPageY() {
  AddressT base = static_cast<AddressT>(readByte(state_.pc + 1));
  return wrapAroundOffset(base, state_.rY);
}

M6502::AddressT M6502::addr_IndexedIndirect() {
  AddressT zp_addr = static_cast<AddressT>(readByte(state_.pc + 1));
  zp_addr = wrapAroundOffset(zp_addr, state_.rX);
  DataT addr_lo = readByte(zp_addr++);
  zp_addr &= 0xFF;
  AddressT addr_hi = static_cast<AddressT>(readByte(zp_addr));
  AddressT addr = (addr_hi << 8) | addr_lo;
  return addr;
}

M6502::AddressT M6502::addr_IndirectIndexed(instr::Operation op) {
  using instr::Operation;
  AddressT zp_addr = static_cast<AddressT>(readByte(state_.pc + 1));
  DataT addr_lo = readByte(zp_addr++);
  zp_addr &= 0xFF;
  AddressT addr_hi = static_cast<AddressT>(readByte(zp_addr));
  AddressT base = ((addr_hi << 8) | addr_lo);
  if (op == Operation::STA || op == Operation::ISC || op == Operation::DCP ||
      op == Operation::SLO || op == Operation::RLA || op == Operation::SRE ||
      op == Operation::RRA || op == Operation::AXA) {
    tick();
    return base + state_.rY;
  }
  return penalizedOffset(base, state_.rY);
}

} // namespace cpu
