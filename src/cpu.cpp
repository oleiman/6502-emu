#include "cpu.hpp"

#include <iostream>

#define C_O 0
#define Z_O 1
#define I_O 2
#define D_O 3
#define B4_O 4
#define B5_O 5
#define V_O 6
#define N_O 7

// status byte masks
#define CARRY_M (0x01u << C_O)
#define ZERO_M (0x01u << Z_O)
#define INT_DISABLE_M (0x01u << I_O)
#define DECIMAL_M (0x01u << D_O)
#define BIT4_M (0x01u << B4_O)
#define BIT5_M (0x01u << B5_O)
#define OVERFLOW_M (0x01u << V_O)
#define NEGATIVE_M (0x01u << N_O)

// bit manipulation
#define SET(X, MASK) (X |= MASK)
#define GET(X, MASK) (X & MASK)
#define CLEAR(X, MASK) (X &= ~MASK)

// base address for stack
#define STACK_BASE 0x0100
#define STACK_POINTER(X) (STACK_BASE | X)

#define BRK_VEC 0xFFFE

namespace cpu {

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
      writeByte(static_cast<AddressT>(curr++), next);
    } catch (std::runtime_error &e) {
      std::cerr << e.what() << std::endl;
      return false;
    }
  }
  return true;
}

void M6502::step() {
  DataT opcode = readByte(state_.pc);
  instr::Instruction in(
      opcode, state_.pc,
      std::bind(&M6502::calculateAddress, this, std::placeholders::_1));
  fireCallbacks(in);
  state_.pc += in.size();
  dispatch(in);
}

void M6502::registerCallback(instr::Operation op, Callback c) {
  callbacks_[op].push_back(c);
}

void M6502::fireCallbacks(instr::Instruction const &in) {
  if (callbacks_.count(in.operation()) > 0) {
    for (auto &c : callbacks_[in.operation()]) {
      c(in.address());
    }
  }
}

M6502::DataT M6502::readByte(AddressT addr) {
  address_bus_.put(addr);
  return data_bus_.get();
}

void M6502::writeByte(AddressT addr, DataT data) {
  address_bus_.put(addr);
  data_bus_.put(data);
}

M6502::AddressT M6502::calculateAddress(instr::AddressMode mode) {
  using instr::AddressMode;
  switch (mode) {
  case AddressMode::accumulator:
    return addr_Accumulator();
  case AddressMode::immediate:
    return addr_Immediate();
  case AddressMode::absolute:
    return addr_Absolute();
  case AddressMode::absoluteX:
    return addr_AbsoluteX();
  case AddressMode::absoluteY:
    return addr_AbsoluteY();
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
    return addr_IndirectIndexed();
  default:
    return addr_Implicit();
    ;
  }
}

void M6502::dispatch(instr::Instruction const &in) {
  using instr::Operation;
  switch (in.operation()) {
  case Operation::loadA:
    op_LD(state_.rA, in.address());
    break;
  case Operation::loadX:
    op_LD(state_.rX, in.address());
    break;
  case Operation::loadY:
    op_LD(state_.rY, in.address());
    break;
  case Operation::storeA:
    op_ST(in.address(), state_.rA);
    break;
  case Operation::storeX:
    op_ST(in.address(), state_.rX);
    break;
  case Operation::storeY:
    op_ST(in.address(), state_.rY);
    break;
  case Operation::add:
    op_ADC(in.address());
    break;
  case Operation::subtract:
    op_SBC(in.address());
    break;
  case Operation::increment:
    op_INC(in.address(), 1);
    break;
  case Operation::incrementX:
    op_INR(state_.rX, 1);
    break;
  case Operation::incrementY:
    op_INR(state_.rY, 1);
    break;
  case Operation::decrement:
    op_INC(in.address(), -1);
    break;
  case Operation::decrementX:
    op_INR(state_.rX, -1);
    break;
  case Operation::decrementY:
    op_INR(state_.rY, -1);
    break;
  case Operation::shiftL:
    op_ASL(in.address());
    break;
  case Operation::shiftLA:
    op_ASLV(state_.rA);
    break;
  case Operation::shiftR:
    op_LSR(in.address());
    break;
  case Operation::shiftRA:
    op_LSRV(state_.rA);
    break;
  case Operation::rotateL:
    op_ROL(in.address());
    break;
  case Operation::rotateLA:
    op_ROLV(state_.rA);
    break;
  case Operation::rotateR:
    op_ROR(in.address());
    break;
  case Operation::rotateRA:
    op_RORV(state_.rA);
    break;
  case Operation::bwAND:
    op_AND(in.address());
    break;
  case Operation::bwOR:
    op_ORA(in.address());
    break;
  case Operation::bwXOR:
    op_EOR(in.address());
    break;
  case Operation::compare:
    op_CMP(state_.rA, in.address());
    break;
  case Operation::compareX:
    op_CMP(state_.rX, in.address());
    break;
  case Operation::compareY:
    op_CMP(state_.rY, in.address());
    break;
  case Operation::bitTest:
    op_BIT(in.address());
    break;
  case Operation::branchPos:
    op_BRClear(NEGATIVE_M, in.address());
    break;
  case Operation::branchVC:
    op_BRClear(OVERFLOW_M, in.address());
    break;
  case Operation::branchCC:
    op_BRClear(CARRY_M, in.address());
    break;
  case Operation::branchNE:
    op_BRClear(ZERO_M, in.address());
    break;
  case Operation::branchNeg:
    op_BRSet(NEGATIVE_M, in.address());
    break;
  case Operation::branchVS:
    op_BRSet(OVERFLOW_M, in.address());
    break;
  case Operation::branchCS:
    op_BRSet(CARRY_M, in.address());
    break;
  case Operation::branchEQ:
    op_BRSet(ZERO_M, in.address());
    break;
  case Operation::transferAX:
    op_XFER(state_.rA, state_.rX);
    break;
  case Operation::transferAY:
    op_XFER(state_.rA, state_.rY);
    break;
  case Operation::transferSX:
    op_XFER(state_.sp, state_.rX);
    break;
  case Operation::transferXA:
    op_XFER(state_.rX, state_.rA);
    break;
  case Operation::transferXS:
    op_XFER(state_.rX, state_.sp);
    break;
  case Operation::transferYA:
    op_XFER(state_.rY, state_.rA);
    break;
  case Operation::jump:
    op_JMP(in.address());
    break;
  case Operation::jumpSR:
    op_JMP(in.address(), true);
    break;
  case Operation::returnSR:
    op_RTS();
    break;
  case Operation::returnINT:
    op_RTI();
    break;
  case Operation::pushA:
    op_PUSH(state_.rA);
    break;
  case Operation::pullA:
    op_PULL(state_.rA);
    break;
  case Operation::pushS:
    op_PHP();
    break;
  case Operation::pullS:
    op_PLP();
    break;
  case Operation::forceBreak:
    op_BRK();
    break;
  case Operation::clearC:
    CLEAR(state_.status, CARRY_M);
    break;
  case Operation::clearI:
    CLEAR(state_.status, INT_DISABLE_M);
    break;
  case Operation::clearV:
    CLEAR(state_.status, OVERFLOW_M);
    break;
  case Operation::clearD:
    CLEAR(state_.status, DECIMAL_M);
    break;
  case Operation::setC:
    SET(state_.status, CARRY_M);
    break;
  case Operation::setI:
    SET(state_.status, INT_DISABLE_M);
    break;
  case Operation::setD:
    SET(state_.status, DECIMAL_M);
    break;
  case Operation::nop:
    // no operation
    break;
  default:
    op_Illegal(in.opcode());
    break;
  }
}

void M6502::op_Illegal(DataT opcode) {
  std::cerr << "Illegal Opcode: <0x" << std::hex << std::setfill('0')
            << std::setw(2) << +opcode << ">" << std::endl;
  // TODO(oren): maybe should bail here, but useful to press on
  // for debugging purposes
}

// load
void M6502::op_LD(DataT &dest, AddressT source) {

  dest = readByte(source);
  setOrClearStatus(GET(dest, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(dest == 0, ZERO_M);
}

// store
void M6502::op_ST(AddressT dest, DataT data) { writeByte(dest, data); }

// add memory to accumulator with carry
void M6502::op_ADC(AddressT source) {
  DataT addend = readByte(source);
  // TODO(oren): not used? logic error?
  // DataT input_carry = GET(_state.status, CARRY_M);
  AddressT result = 0;

  if (GET(state_.status, DECIMAL_M)) {
    DataT accum_dec = ((state_.rA & 0xF0) >> 4) * 10 + state_.rA & 0x0F;
    DataT addend_dec = ((addend & 0xF0) >> 4) * 10 + addend & 0x0F;

    if (accum_dec > 99 || addend_dec > 99) {
      return;
    }
    accum_dec += addend_dec;
    setOrClearStatus(accum_dec > 99, CARRY_M);
    result = ((accum_dec / 10) << 4) | (accum_dec % 10);
    setOrClearStatus(result == 0, ZERO_M);
    setOrClearStatus(GET(result, NEGATIVE_M), NEGATIVE_M);
  } else {
    result = static_cast<uint16_t>(state_.rA);
    result += static_cast<uint16_t>(addend);
    result += GET(state_.status, CARRY_M);
    setOrClearStatus(result & 0xFF00, CARRY_M);

    // mask off the carry
    result &= 0xFF;

    setOrClearStatus(GET(result, NEGATIVE_M), NEGATIVE_M);
    setOrClearStatus(result == 0, ZERO_M);

    if (GET(state_.rA, NEGATIVE_M) == GET(addend, NEGATIVE_M) &&
        GET(state_.rA, NEGATIVE_M) != GET(result, NEGATIVE_M)) {
      SET(state_.status, OVERFLOW_M);
    } else {
      CLEAR(state_.status, OVERFLOW_M);
    }
  }
  state_.rA = static_cast<DataT>(result);
}

// subtract memory from accumulator with borrow
void M6502::op_SBC(AddressT source) {
  // take two's complement with borrow
  DataT subtrahend = readByte(source);
  // TODO(oren): not used? logic error?
  // DataT input_carry = GET(_state.status, CARRY_M);
  uint16_t result = 0;

  if (GET(state_.status, DECIMAL_M)) {
    DataT accum_dec = ((state_.rA & 0xF0) >> 4) * 10 + state_.rA & 0x0F;
    DataT subtrahend_dec = ((subtrahend & 0xF0) >> 4) * 10 + subtrahend & 0x0F;

    subtrahend_dec -= GET(state_.status, CARRY_M) ? 0 : 1;

    if (accum_dec > 99 || subtrahend_dec > 99) {
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
    DataT complement = ~subtrahend + GET(state_.status, CARRY_M);
    result = static_cast<uint16_t>(state_.rA);
    result += complement;
    setOrClearStatus(result & 0xFF00, CARRY_M);
    // mask off the carry
    result &= 0xFF;
    setOrClearStatus(GET(result, NEGATIVE_M), NEGATIVE_M);
    setOrClearStatus(result == 0, ZERO_M);
    if (GET(state_.rA, NEGATIVE_M) != GET(subtrahend, NEGATIVE_M) &&
        GET(state_.rA, NEGATIVE_M) != GET(result, NEGATIVE_M)) {
      SET(state_.status, OVERFLOW_M);
    } else {
      CLEAR(state_.status, OVERFLOW_M);
    }
  }
  state_.rA = static_cast<DataT>(result);
}

// increment memory
void M6502::op_INC(AddressT source, int8_t val) {
  DataT result = readByte(source);
  result += val;
  setOrClearStatus(GET(result, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(result == 0, ZERO_M);
  writeByte(source, result);
}

// increment register
void M6502::op_INR(DataT &reg, int8_t val) {
  reg += val;
  setOrClearStatus(GET(reg, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(reg == 0, ZERO_M);
}

// arithmetic shift left
void M6502::op_ASL(AddressT source) {
  DataT result = readByte(source);
  op_ASLV(result);
  writeByte(source, result);
}

void M6502::op_ASLV(DataT &val) {
  setOrClearStatus(GET(val, NEGATIVE_M), CARRY_M);
  val <<= 1;
  setOrClearStatus(GET(val, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(val == 0, ZERO_M);
}

// logical shift right on memory
void M6502::op_LSR(AddressT source) {
  DataT result = readByte(source);
  op_LSRV(result);
  writeByte(source, result);
}

// logical shift right on value
void M6502::op_LSRV(DataT &val) {
  setOrClearStatus(GET(val, CARRY_M), CARRY_M);
  val >>= 1;
  setOrClearStatus(GET(val, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(val == 0, ZERO_M);
}

// rotate left on memory
void M6502::op_ROL(AddressT source) {
  DataT result = readByte(source);
  op_ROLV(result);
  writeByte(source, result);
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
}

// rotate right on memory
void M6502::op_ROR(AddressT source) {
  DataT result = readByte(source);
  op_RORV(result);
  writeByte(source, result);
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
void M6502::op_AND(AddressT source) {
  state_.rA &= readByte(source);
  setOrClearStatus(GET(state_.rA, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(state_.rA == 0, ZERO_M);
}

// OR memory with accumulator
void M6502::op_ORA(AddressT source) {
  state_.rA |= readByte(source);
  setOrClearStatus(GET(state_.rA, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(state_.rA == 0, ZERO_M);
}

// XOR memory with accumulator
void M6502::op_EOR(AddressT source) {
  state_.rA ^= readByte(source);
  setOrClearStatus(GET(state_.rA, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(state_.rA == 0, ZERO_M);
}

// compare memory and register
void M6502::op_CMP(DataT reg, AddressT source) {
  DataT val = readByte(source);
  if (reg < val) {
    SET(state_.status, NEGATIVE_M);
    CLEAR(state_.status, ZERO_M);
    CLEAR(state_.status, CARRY_M);
  } else if (reg == val) {
    CLEAR(state_.status, NEGATIVE_M);
    SET(state_.status, ZERO_M);
    SET(state_.status, CARRY_M);
  } else if (reg > val) {
    CLEAR(state_.status, NEGATIVE_M);
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
    state_.pc = target;
  }
}

// branch if flag is set
void M6502::op_BRSet(uint8_t flag, AddressT target) {
  if (GET(state_.status, flag)) {
    state_.pc = target;
  }
}

// transfer from source to dest
void M6502::op_XFER(DataT source, DataT &dest) {
  dest = source;
  setOrClearStatus(GET(dest, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(dest == 0, ZERO_M);
}

// push value onto stack
void M6502::op_PUSH(DataT source) {
  writeByte(STACK_POINTER(state_.sp), source);
  state_.sp--;
}

// pop from the stack into dest
void M6502::op_PULL(DataT &dest) {
  state_.sp++;
  dest = readByte(STACK_POINTER(state_.sp));
  setOrClearStatus(GET(dest, NEGATIVE_M), NEGATIVE_M);
  setOrClearStatus(dest == 0, ZERO_M);
}

// jump. maybe to a subroutine
void M6502::op_JMP(AddressT target, bool sr) {
  // jump to subroutine
  if (sr) {
    // note that we push the pc of the last byte of the
    // jump instruction
    AddressT returnAddr = state_.pc - 1;
    DataT pc_hi = (returnAddr >> 8) & 0xFF;
    DataT pc_lo = returnAddr & 0xFF;
    op_PUSH(pc_hi);
    op_PUSH(pc_lo);
  }
  state_.pc = target;
}

// return from subroutine
void M6502::op_RTS() {
  state_.sp++;
  DataT target_lo = readByte(STACK_POINTER(state_.sp++));
  AddressT target_hi =
      static_cast<AddressT>(readByte(STACK_POINTER(state_.sp)));
  state_.pc = (target_hi << 8) | target_lo;
  state_.pc++;
}

// return from interrupt
void M6502::op_RTI() {
  state_.sp++;
  state_.status = readByte(STACK_POINTER(state_.sp++));
  DataT target_lo = readByte(STACK_POINTER(state_.sp++));
  AddressT target_hi =
      static_cast<AddressT>(readByte(STACK_POINTER(state_.sp)));

  state_.pc = (target_hi << 8) | target_lo;
  // no pc increment here
}

void M6502::op_PHP() {
  DataT tmp = state_.status;
  SET(tmp, BIT5_M);
  SET(tmp, BIT4_M);
  op_PUSH(tmp);
}

void M6502::op_PLP() {
  DataT tmp = readByte(STACK_POINTER(++state_.sp));
  // ignore the "B Flag"
  CLEAR(tmp, BIT5_M);
  CLEAR(tmp, BIT4_M);
  state_.status = tmp;
}

// force interrupt
void M6502::op_BRK() {
  AddressT returnAddr = state_.pc + 2;
  DataT pc_hi = (returnAddr >> 8) & 0xFF;
  DataT pc_lo = returnAddr & 0xFF;
  op_PUSH(pc_hi);
  op_PUSH(pc_lo);

  DataT tmp = state_.status;
  SET(tmp, BIT5_M);
  SET(tmp, BIT4_M);
  op_PUSH(tmp);

  DataT target_lo = readByte(BRK_VEC);
  AddressT target_hi = static_cast<AddressT>(readByte(BRK_VEC + 1));
  state_.pc = (target_hi << 8) | target_lo;
}

void M6502::setOrClearStatus(bool pred, uint8_t mask) {
  if (pred) {
    SET(state_.status, mask);
  } else {
    CLEAR(state_.status, mask);
  }
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
  ;
  AddressT addr = state_.pc + 2 + offset;

  return addr;
}

M6502::AddressT M6502::addr_Indirect() {
  DataT i_addr_lo = readByte(state_.pc + 1);
  AddressT i_addr_hi = static_cast<AddressT>(readByte(state_.pc + 2));
  AddressT i_addr = (i_addr_hi << 8) | i_addr_lo;

  DataT addr_lo = readByte(i_addr++);
  AddressT addr_hi = static_cast<AddressT>(readByte(i_addr++));

  AddressT addr = (addr_hi << 8) | addr_lo;

  return addr;
}

M6502::AddressT M6502::addr_AbsoluteX() {
  DataT addr_lo = readByte(state_.pc + 1);
  AddressT addr_hi = static_cast<AddressT>(readByte(state_.pc + 2));

  AddressT addr = (addr_hi << 8) | addr_lo;
  addr += state_.rX;

  return addr;
}

M6502::AddressT M6502::addr_AbsoluteY() {
  DataT addr_lo = readByte(state_.pc + 1);
  AddressT addr_hi = static_cast<AddressT>(readByte(state_.pc + 2));

  AddressT addr = (addr_hi << 8) | addr_lo;
  addr += state_.rY;

  return addr;
}

M6502::AddressT M6502::addr_ZeroPageX() {
  DataT addr = readByte(state_.pc + 1);
  addr += state_.rX;
  return static_cast<AddressT>(addr);
}

M6502::AddressT M6502::addr_ZeroPageY() {
  DataT addr = readByte(state_.pc + 1);
  addr += state_.rY;
  return static_cast<AddressT>(addr);
}

M6502::AddressT M6502::addr_IndexedIndirect() {
  AddressT zp_addr = static_cast<AddressT>(readByte(state_.pc + 1)) + state_.rX;
  DataT addr_lo = readByte(zp_addr++);
  AddressT addr_hi = static_cast<AddressT>(readByte(zp_addr));

  AddressT addr = (addr_hi << 8) | addr_lo;

  return addr;
}

M6502::AddressT M6502::addr_IndirectIndexed() {
  AddressT zp_addr = static_cast<AddressT>(readByte(state_.pc + 1));

  DataT addr_lo = readByte(zp_addr++);
  AddressT addr_hi = static_cast<AddressT>(readByte(zp_addr));
  AddressT addr = ((addr_hi << 8) | addr_lo) + state_.rY;

  return addr;
}

} // namespace cpu
