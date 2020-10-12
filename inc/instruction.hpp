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
    explicit Instruction(
        uint8_t opcode,
        uint16_t pc,
        std::function<uint16_t (AddressMode)> calcAddr);
    ~Instruction() = default;

    Operation operation() const { return _op; }
    AddressMode addressMode() const { return _addr_mode; }
    uint8_t size() const { return _size; }
    uint8_t opcode() const { return _opcode; }
    uint16_t address() const { return _address; }
    uint16_t pc() const { return _pc; }

    friend std::ostream& operator<<(std::ostream& os, const Instruction& in);

private:
    uint8_t _opcode;
    uint8_t _size;
    AddressMode _addr_mode;    
    Operation _op;
    uint16_t _pc;
    uint16_t _address;

    void decodeAMode();
    void decodeOp();
};

} // namespace instr
