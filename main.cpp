#include "cpu.hpp"
#include "debugger.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>

using std::make_shared;

using cpu::Cpu;
using mem::Ram;
using mem::Bus;
using dbg::Debugger;

constexpr int memory_size = 0x10000;
using ABusWidth = uint16_t;
using MBusWidth = uint8_t;
using Memory = Ram<memory_size, ABusWidth, MBusWidth>;

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "USAGE: 6502-emu /path/to/app" << std::endl;
        exit(1);
    }

    auto abus = std::make_shared<Bus<ABusWidth>>();
    auto mbus = std::make_shared<Bus<MBusWidth>>();
    auto debugger = std::make_shared<Debugger>(abus, mbus, true);
    Memory mem(abus, mbus);

    Cpu<Debugger> cpu(abus, mbus, debugger);

    // 0x4000 specific to test program from interwebs
    cpu.loadRomFromFile(argv[1], 0x4000);
    cpu.initPc(0x4000);

    while(true) {
        // if (cpu.pc() == 0x45c0) break;
        cpu.step();
    }

    // abus->put(0x0210);
    // std::cout << "[0x0210]:  0x"
    //           << std::hex << std::setfill('0') << std::setw(2)
    //           << +mbus->get() << std::endl;
}
