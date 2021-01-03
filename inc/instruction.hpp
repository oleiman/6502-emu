#pragma once

#include <cstdint>
#include <functional>

namespace instr {

enum class AddressMode {
  implicit,
  accumulator,
  immediate,
  absolute,
  absoluteX,
  absoluteY,
  zeroPage,
  zeroPageX,
  zeroPageY,
  relative,
  indirect,
  indexedIndirect,
  indirectIndexed,
  nAddressModes
};

enum class Operation {
  illegal,
  loadA,
  loadX,
  loadY,
  storeA,
  storeX,
  storeY,
  add,
  subtract,
  increment,
  incrementX,
  incrementY,
  decrement,
  decrementX,
  decrementY,
  shiftL,
  shiftLA,
  shiftR,
  shiftRA,
  rotateL,
  rotateLA,
  rotateR,
  rotateRA,
  bwAND,
  bwOR,
  bwXOR,
  compare,
  compareX,
  compareY,
  bitTest,
  branchPos,
  branchVC,
  branchCC,
  branchNE,
  branchNeg,
  branchVS,
  branchCS,
  branchEQ,
  transferAX,
  transferAY,
  transferSX,
  transferXA,
  transferXS,
  transferYA,
  jump,
  jumpSR,
  returnSR,
  returnINT,
  pushA,
  pullA,
  pushS,
  pullS,
  forceBreak,
  clearC,
  clearI,
  clearV,
  clearD,
  setC,
  setI,
  setD,
  nop,
  nOperations
};

class Instruction {
  using AddressT = uint16_t;
  using DataT = uint8_t;

public:
  explicit Instruction(DataT opcode, AddressT pc, unsigned long long cycle);
  ~Instruction() = default;

  Operation operation() const { return operation_; }
  AddressMode addressMode() const { return address_mode_; }
  DataT size() const { return size_; }
  DataT opcode() const { return opcode_; }
  /***Hack for easier debug output***/
  AddressT address() const { return address_; }
  void setAddress(AddressT const addr) { address_ = addr; }
  /**********************************/
  AddressT pc() const { return pc_; }
  unsigned long long cycle() const { return start_cycle_; }

  friend std::ostream &operator<<(std::ostream &os, const Instruction &in);

private:
  DataT opcode_;
  DataT size_;
  unsigned long long start_cycle_;
  AddressMode address_mode_;
  Operation operation_;
  AddressT pc_;
  AddressT address_;

  void decodeAddressMode();
  void decodeOperation();
};

} // namespace instr
