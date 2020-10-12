#pragma once

#include "cpu.hpp"
#include "instruction.hpp"
#include "memory.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace dbg {
class BreakPoint;
    
class Debugger {
public:
    explicit Debugger(
        std::shared_ptr<mem::Bus<uint16_t>> abus,
        std::shared_ptr<mem::Bus<uint8_t>> mbus,
        bool brek = false);
    ~Debugger() = default;

    void step(instr::Instruction const& in, cpu::State const& state);
private:
    std::shared_ptr<mem::Bus<uint16_t>> _address_bus;
    std::shared_ptr<mem::Bus<uint8_t>> _memory_bus;
    bool _break;
    bool _step;
    std::unique_ptr<instr::Instruction> _prev_in;
    std::vector<BreakPoint> _breakpoints;
    uint16_t extract_addr(std::string const& command);
};

class BreakPoint {
public:
    explicit BreakPoint(uint16_t pc)
        : _pc(pc)
    {}

    bool shouldBreak(instr::Instruction const& in);
private:
    uint16_t _pc;
};

} // namespace dbg
