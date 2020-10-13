#pragma once

#include "instruction.hpp"
#include "memory.hpp"

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

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

struct State {
  // TODO(oren): manual indicates that SP is initialized by
  // the programmer...
  explicit State()
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
  // TODO(oren): do away with this awful pointer to Debugger
  explicit M6502(mem::Bus<AddressT> &abus, mem::Bus<DataT> &mbus)
      : address_bus_(abus), data_bus_(mbus) {}
  ~M6502() = default;

  bool loadRomFromFile(const std::string &fname, AddressT start) {
    std::ifstream infile(fname, std::ios::binary);
    if (!infile) {
      std::cerr << "Bad File: " << fname << std::endl;
      return false;
    }
    char next;
    uint32_t curr = start;
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

  void initPc(AddressT val) { state_.pc = val; }
  AddressT pc() const { return state_.pc; }

  template <class Debugger> void debugStep(Debugger &debugger) {
    DataT opcode = readByte(state_.pc);
    instr::Instruction in(
        opcode, state_.pc,
        std::bind(&M6502::memoryAddress, this, std::placeholders::_1));

    // TODO(oren): code smell...null check on every iteration
    debugger.step(in, state_, address_bus_, data_bus_);
    state_.pc += in.size();
    dispatch(in);
  }

  void step() {
    DataT opcode = readByte(state_.pc);
    instr::Instruction in(
        opcode, state_.pc,
        std::bind(&M6502::memoryAddress, this, std::placeholders::_1));

    state_.pc += in.size();
    dispatch(in);
  }

private:
  State state_;

  mem::Bus<AddressT> &address_bus_;
  mem::Bus<DataT> &data_bus_;

  DataT readByte(AddressT addr) {
    address_bus_.put(addr);
    return data_bus_.get();
  }

  void writeByte(AddressT addr, DataT data) {
    address_bus_.put(addr);
    data_bus_.put(data);
  }

  AddressT memoryAddress(instr::AddressMode mode) {
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

  void dispatch(instr::Instruction const &in) {
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

  // TODO(oren): clean up numeric types to distinguish addresses from data, etc
  // START HERE
  void op_Illegal(uint8_t opcode) {
    std::cerr << "Illegal Opcode: <0x" << std::hex << std::setfill('0')
              << std::setw(2) << +opcode << ">" << std::endl;
    ;
    // TODO(oren): maybe should bail here, but useful to press on
    // for debugging purposes
  }

  // load
  void op_LD(uint8_t &dest, uint16_t source) {
    dest = readByte(source);
    setOrClearStatus(GET(dest, NEGATIVE_M), NEGATIVE_M);
    setOrClearStatus(dest == 0, ZERO_M);
  }

  // store
  void op_ST(uint16_t dest, uint8_t data) { writeByte(dest, data); }

  // add memory to accumulator with carry
  void op_ADC(uint16_t source) {
    uint8_t addend = readByte(source);
    // TODO(oren): not used? logic error?
    // uint8_t input_carry = GET(_state.status, CARRY_M);
    uint16_t result = 0;

    if (GET(state_.status, DECIMAL_M)) {
      uint8_t accum_dec = ((state_.rA & 0xF0) >> 4) * 10 + state_.rA & 0x0F;
      uint8_t addend_dec = ((addend & 0xF0) >> 4) * 10 + addend & 0x0F;

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
    state_.rA = static_cast<uint8_t>(result);
  }

  // subtract memory from accumulator with borrow
  void op_SBC(uint16_t source) {
    // take two's complement with borrow
    uint8_t subtrahend = readByte(source);
    // TODO(oren): not used? logic error?
    // uint8_t input_carry = GET(_state.status, CARRY_M);
    uint16_t result = 0;

    if (GET(state_.status, DECIMAL_M)) {
      uint8_t accum_dec = ((state_.rA & 0xF0) >> 4) * 10 + state_.rA & 0x0F;
      uint8_t subtrahend_dec =
          ((subtrahend & 0xF0) >> 4) * 10 + subtrahend & 0x0F;

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
      uint8_t complement = ~subtrahend + GET(state_.status, CARRY_M);
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
    state_.rA = static_cast<uint8_t>(result);
  }

  // increment memory
  void op_INC(uint16_t source, int8_t val) {
    uint8_t result = readByte(source);
    result += val;
    setOrClearStatus(GET(result, NEGATIVE_M), NEGATIVE_M);
    setOrClearStatus(result == 0, ZERO_M);
    writeByte(source, result);
  }

  // increment register
  void op_INR(uint8_t &reg, int8_t val) {
    reg += val;
    setOrClearStatus(GET(reg, NEGATIVE_M), NEGATIVE_M);
    setOrClearStatus(reg == 0, ZERO_M);
  }

  // arithmetic shift left
  void op_ASL(uint16_t source) {
    uint8_t result = readByte(source);
    op_ASLV(result);
    writeByte(source, result);
  }

  void op_ASLV(uint8_t &val) {
    setOrClearStatus(GET(val, NEGATIVE_M), CARRY_M);
    val <<= 1;
    setOrClearStatus(GET(val, NEGATIVE_M), NEGATIVE_M);
    setOrClearStatus(val == 0, ZERO_M);
  }

  // logical shift right on memory
  void op_LSR(uint16_t source) {
    uint8_t result = readByte(source);
    op_LSRV(result);
    writeByte(source, result);
  }

  // logical shift right on value
  void op_LSRV(uint8_t &val) {
    setOrClearStatus(GET(val, CARRY_M), CARRY_M);
    val >>= 1;
    setOrClearStatus(GET(val, NEGATIVE_M), NEGATIVE_M);
    setOrClearStatus(val == 0, ZERO_M);
  }

  // rotate left on memory
  void op_ROL(uint16_t source) {
    uint8_t result = readByte(source);
    op_ROLV(result);
    writeByte(source, result);
  }

  // rotate left on value
  void op_ROLV(uint8_t &val) {
    uint8_t input_carry = GET(state_.status, CARRY_M);
    setOrClearStatus(GET(val, NEGATIVE_M), CARRY_M);
    val <<= 1;
    if (input_carry) {
      SET(val, CARRY_M);
    }
    setOrClearStatus(GET(val, NEGATIVE_M), NEGATIVE_M);
    setOrClearStatus(val == 0, ZERO_M);
  }

  // rotate right on memory
  void op_ROR(uint16_t source) {
    uint8_t result = readByte(source);
    op_RORV(result);
    writeByte(source, result);
  }

  void op_RORV(uint8_t &val) {
    uint8_t input_carry = GET(state_.status, CARRY_M);
    setOrClearStatus(GET(val, CARRY_M), CARRY_M);
    val >>= 1;
    if (input_carry) {
      SET(val, NEGATIVE_M);
    }
    setOrClearStatus(GET(val, NEGATIVE_M), NEGATIVE_M);
    setOrClearStatus(val == 0, ZERO_M);
  }

  // AND memory with accumulator
  void op_AND(uint16_t source) {
    state_.rA &= readByte(source);
    setOrClearStatus(GET(state_.rA, NEGATIVE_M), NEGATIVE_M);
    setOrClearStatus(state_.rA == 0, ZERO_M);
  }

  // OR memory with accumulator
  void op_ORA(uint16_t source) {
    state_.rA |= readByte(source);
    setOrClearStatus(GET(state_.rA, NEGATIVE_M), NEGATIVE_M);
    setOrClearStatus(state_.rA == 0, ZERO_M);
  }

  // XOR memory with accumulator
  void op_EOR(uint16_t source) {
    state_.rA ^= readByte(source);
    setOrClearStatus(GET(state_.rA, NEGATIVE_M), NEGATIVE_M);
    setOrClearStatus(state_.rA == 0, ZERO_M);
  }

  // compare memory and register
  void op_CMP(uint8_t reg, uint16_t source) {
    uint8_t val = readByte(source);
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
  void op_BIT(uint16_t source) {
    uint8_t acc = state_.rA;
    uint8_t mem = readByte(source);
    setOrClearStatus(GET(mem, NEGATIVE_M), NEGATIVE_M);
    setOrClearStatus(GET(mem, OVERFLOW_M), OVERFLOW_M);
    setOrClearStatus((acc & mem) == 0, ZERO_M);
  }

  // branch if flag is clear
  void op_BRClear(uint8_t flag, uint16_t target) {
    if (GET(state_.status, flag) == 0) {
      state_.pc = target;
    }
  }

  // branch if flag is set
  void op_BRSet(uint8_t flag, uint16_t target) {
    if (GET(state_.status, flag)) {
      state_.pc = target;
    }
  }

  // transfer from source to dest
  void op_XFER(uint8_t source, uint8_t &dest) {
    dest = source;
    setOrClearStatus(GET(dest, NEGATIVE_M), NEGATIVE_M);
    setOrClearStatus(dest == 0, ZERO_M);
  }

  // push value onto stack
  void op_PUSH(uint8_t source) {
    writeByte(STACK_POINTER(state_.sp), source);
    state_.sp--;
  }

  // pop from the stack into dest
  void op_PULL(uint8_t &dest) {
    state_.sp++;
    dest = readByte(STACK_POINTER(state_.sp));
    setOrClearStatus(GET(dest, NEGATIVE_M), NEGATIVE_M);
    setOrClearStatus(dest == 0, ZERO_M);
  }

  // jump. maybe to a subroutine
  void op_JMP(uint16_t target, bool sr = false) {
    // jump to subroutine
    if (sr) {
      // note that we push the pc of the last byte of the
      // jump instruction
      uint16_t returnAddr = state_.pc - 1;
      uint8_t pc_hi = (returnAddr >> 8) & 0xFF;
      uint8_t pc_lo = returnAddr & 0xFF;
      op_PUSH(pc_hi);
      op_PUSH(pc_lo);
    }
    state_.pc = target;
  }

  // return from subroutine
  void op_RTS() {
    state_.sp++;
    uint8_t target_lo = readByte(STACK_POINTER(state_.sp++));
    uint16_t target_hi =
        static_cast<uint16_t>(readByte(STACK_POINTER(state_.sp)));
    state_.pc = (target_hi << 8) | target_lo;
    state_.pc++;
  }

  // return from interrupt
  void op_RTI() {
    state_.sp++;
    state_.status = readByte(STACK_POINTER(state_.sp++));
    uint8_t target_lo = readByte(STACK_POINTER(state_.sp++));
    uint16_t target_hi =
        static_cast<uint16_t>(readByte(STACK_POINTER(state_.sp)));

    state_.pc = (target_hi << 8) | target_lo;
    // no pc increment here
  }

  void op_PHP() {
    uint8_t tmp = state_.status;
    SET(tmp, BIT5_M);
    SET(tmp, BIT4_M);
    op_PUSH(tmp);
  }

  void op_PLP() {
    uint8_t tmp = readByte(STACK_POINTER(++state_.sp));
    // ignore the "B Flag"
    CLEAR(tmp, BIT5_M);
    CLEAR(tmp, BIT4_M);
    state_.status = tmp;
  }

  // force interrupt
  void op_BRK() {
    uint16_t returnAddr = state_.pc + 2;
    uint8_t pc_hi = (returnAddr >> 8) & 0xFF;
    uint8_t pc_lo = returnAddr & 0xFF;
    op_PUSH(pc_hi);
    op_PUSH(pc_lo);

    uint8_t tmp = state_.status;
    SET(tmp, BIT5_M);
    SET(tmp, BIT4_M);
    op_PUSH(tmp);

    uint8_t target_lo = readByte(BRK_VEC);
    uint16_t target_hi = static_cast<uint16_t>(readByte(BRK_VEC + 1));
    state_.pc = (target_hi << 8) | target_lo;
  }

  void setOrClearStatus(bool pred, uint8_t mask) {
    if (pred) {
      SET(state_.status, mask);
    } else {
      CLEAR(state_.status, mask);
    }
  }

  uint16_t addr_Accumulator() {
    // not used
    return 0;
  }

  uint16_t addr_Immediate() { return state_.pc + 1; }

  uint16_t addr_Implicit() {
    // not used
    return 0;
  }

  uint16_t addr_Absolute() {
    uint8_t addr_lo = readByte(state_.pc + 1);
    uint16_t addr_hi = static_cast<uint16_t>(readByte(state_.pc + 2));

    return (addr_hi << 8) | addr_lo;
  }

  uint16_t addr_ZeroPage() {
    uint16_t addr = static_cast<uint16_t>(readByte(state_.pc + 1));
    return addr;
  }

  uint16_t addr_Relative() {
    int8_t offset = static_cast<int8_t>(readByte(state_.pc + 1));
    ;
    uint16_t addr = state_.pc + 2 + offset;

    return addr;
  }

  uint16_t addr_Indirect() {
    uint8_t i_addr_lo = readByte(state_.pc + 1);
    uint16_t i_addr_hi = static_cast<uint16_t>(readByte(state_.pc + 2));
    uint16_t i_addr = (i_addr_hi << 8) | i_addr_lo;

    uint8_t addr_lo = readByte(i_addr++);
    uint16_t addr_hi = static_cast<uint16_t>(readByte(i_addr++));

    return (addr_hi << 8) | addr_lo;
  }

  uint16_t addr_AbsoluteX() {
    uint8_t addr_lo = readByte(state_.pc + 1);
    uint16_t addr_hi = static_cast<uint16_t>(readByte(state_.pc + 2));

    uint16_t addr = (addr_hi << 8) | addr_lo;
    addr += state_.rX;

    return addr;
  }

  uint16_t addr_AbsoluteY() {
    uint8_t addr_lo = readByte(state_.pc + 1);
    uint16_t addr_hi = static_cast<uint16_t>(readByte(state_.pc + 2));

    uint16_t addr = (addr_hi << 8) | addr_lo;
    addr += state_.rY;

    return addr;
  }

  uint16_t addr_ZeroPageX() {
    uint8_t addr = readByte(state_.pc + 1);
    addr += state_.rX;
    return static_cast<uint16_t>(addr);
  }

  uint16_t addr_ZeroPageY() {
    uint8_t addr = readByte(state_.pc + 1);
    addr += state_.rY;
    return static_cast<uint16_t>(addr);
  }

  uint16_t addr_IndexedIndirect() {
    uint16_t zp_addr =
        static_cast<uint16_t>(readByte(state_.pc + 1)) + state_.rX;
    uint8_t addr_lo = readByte(zp_addr++);
    uint16_t addr_hi = static_cast<uint16_t>(readByte(zp_addr));

    uint16_t addr = (addr_hi << 8) | addr_lo;

    return addr;
  }

  uint16_t addr_IndirectIndexed() {
    uint16_t zp_addr = static_cast<uint16_t>(readByte(state_.pc + 1));

    uint8_t addr_lo = readByte(zp_addr++);
    uint16_t addr_hi = static_cast<uint16_t>(readByte(zp_addr));
    uint16_t addr = ((addr_hi << 8) | addr_lo) + state_.rY;

    return addr;
  }
};

} // namespace cpu

inline std::string status_to_str(uint8_t status) {
  std::stringstream result;
  for (uint8_t mask = 0x80; mask > 0; mask /= 2) {
    if (GET(status, mask)) {
      result << "1";
    } else {
      result << "0";
    }
  }
  return result.str();
}

inline std::ostream &operator<<(std::ostream &os, cpu::State const &state) {
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
