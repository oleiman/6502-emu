#define CATCH_CONFIG_MAIN

#include "cpu.hpp"
#include "catch.hpp"
#include "debugger.hpp"
#include "util.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>

#define xstr(s) str(s)
#define str(s) #s

using std::make_shared;
using std::filesystem::current_path;

using cpu::Cpu;
using dbg::Debugger;
using mem::Bus;
using mem::Ram;

constexpr int memory_size = 0x10000;
using ABusWidth = uint16_t;
using MBusWidth = uint8_t;
using Memory = Ram<memory_size, ABusWidth, MBusWidth>;

TEST_CASE("AllSuiteA", "[integration][cpu]") {
  // auto abus = std::make_shared<Bus<ABusWidth>>();
  // auto mbus = std::make_shared<Bus<MBusWidth>>();
  // auto debugger = std::make_shared<Debugger>(abus, mbus, true);
  Ram<memory_size, uint16_t, uint8_t> memory;

  Cpu cpu(memory.addressBus(), memory.dataBus());

  // 0x4000 specific to test program from interwebs
  current_path(xstr(SOURCE_DIR));
  REQUIRE(cpu.loadRomFromFile(AllSuitePath, 0x4000));
  cpu.initPc(0x4000);

  while (true) {
    if (cpu.pc() == 0x45c0)
      break;
    cpu.step();
  }

  memory.addressBus().put(0x0210);
  REQUIRE(memory.dataBus().get() == 0xff);
}
