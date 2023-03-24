#pragma once

#include <cstdint>
#include <functional>
#include <string>

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
  LDA,
  LDX,
  LDY,
  LAX,
  LAS,
  STA,
  STX,
  STY,
  SAX,
  ADC,
  SBC,
  INC,
  INX,
  INY,
  ISC,
  DEC,
  DEX,
  DEY,
  DCP,
  ASL,
  ASLA,
  SLO,
  LSR,
  LSRA,
  SRE,
  ROL,
  ROLA,
  RLA,
  ROR,
  RORA,
  RRA,
  AND,
  ANC,
  ALR,
  ARR,
  XAA,
  AXA,
  AXS,
  TAS,
  SAY,
  XAS,
  ORA,
  EOR,
  CMP,
  CPX,
  CPY,
  BIT,
  BPL,
  BVC,
  BCC,
  BNE,
  BMI,
  BVS,
  BCS,
  BEQ,
  TAX,
  TAY,
  TSX,
  TXA,
  TXS,
  TYA,
  JMP,
  JSR,
  RTS,
  RTI,
  PHA,
  PLA,
  PHP,
  PLP,
  BRK,
  CLC,
  CLI,
  CLV,
  CLD,
  SEC,
  SEI,
  SED,
  NOP,
  DOP,
  TOP,
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
