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
  loadAX,
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

struct Instruction {
  using AddressT = uint16_t;
  using DataT = uint8_t;

  explicit Instruction(DataT opcode, AddressT pc, unsigned long long cycle);
  ~Instruction() = default;

  friend std::ostream &operator<<(std::ostream &os, const Instruction &in);

  const DataT opcode;
  const unsigned long long issueCycle;
  const AddressT pc;
  const AddressMode addressMode;
  const DataT size;
  const Operation operation;
  AddressT address = 0x0000;

private:
  AddressMode decodeAddressMode();
  Operation decodeOperation();
};

} // namespace instr
