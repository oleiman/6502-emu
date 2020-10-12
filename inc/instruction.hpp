#pragma once

#include <cstdint>
#include <functional>
#include <iostream>

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
public:
  explicit Instruction(uint8_t opcode, uint16_t pc,
                       std::function<uint16_t(AddressMode)> calcAddr);
  ~Instruction() = default;

  Operation operation() const { return operation_; }
  AddressMode addressMode() const { return address_mode_; }
  uint8_t size() const { return size_; }
  uint8_t opcode() const { return opcode_; }
  uint16_t address() const { return address_; }
  uint16_t pc() const { return pc_; }

  friend std::ostream &operator<<(std::ostream &os, const Instruction &in);

private:
  uint8_t opcode_;
  uint8_t size_;
  AddressMode address_mode_;
  Operation operation_;
  uint16_t pc_;
  uint16_t address_;

  void decodeAddressMode();
  void decodeOperation();
};

} // namespace instr
