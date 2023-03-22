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
  loadAS,
  storeA,
  storeX,
  storeY,
  storeAX,
  add,
  subtract,
  increment,
  incrementX,
  incrementY,
  incrementSbc,
  decrement,
  decrementX,
  decrementY,
  decrementCmp,
  shiftL,
  shiftLA,
  shiftLOrA,
  shiftR,
  shiftRA,
  shiftRXor,
  rotateL,
  rotateLA,
  rotateLAnd,
  rotateR,
  rotateRA,
  rotateRAdc,
  bwAND,
  bwANC,
  bwALR,
  bwARR,
  bwXAA,
  bwAXA,
  bwAXS,
  bwTAS,
  bwSAY,
  bwXAS,
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
  dop,
  top,
  nOperations
};

const std::string &GetMnemonic(Operation op);
const std::string &GetAModeMnemonic(AddressMode am);

struct Instruction {
  using AddressT = uint16_t;
  using DataT = uint8_t;

  explicit Instruction(DataT opcode, AddressT pc, unsigned long long cycle);
  ~Instruction() = default;

  Instruction(const Instruction &other);
  Instruction &operator=(const Instruction &other) = delete;

  friend std::ostream &operator<<(std::ostream &os, const Instruction &in);

  const DataT opcode;
  const Operation operation;
  const AddressT pc;
  const AddressMode addressMode;
  AddressT address = 0x0000;
  const unsigned long long issueCycle;
  const DataT size;

  // mutable to allow flagging by debuggers
  mutable bool discard = false;

private:
  AddressMode decodeAddressMode();
  Operation decodeOperation();
};

} // namespace instr
