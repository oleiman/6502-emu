#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "cpu.hpp"
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
using mem::Ram;
using mem::Bus;
using dbg::Debugger;

constexpr int memory_size = 0x10000;
using ABusWidth = uint16_t;
using MBusWidth = uint8_t;
using Memory = Ram<memory_size, ABusWidth, MBusWidth>;

TEST_CASE("AllSuiteA", "[integration][cpu]") {
  auto abus = std::make_shared<Bus<ABusWidth>>();
  auto mbus = std::make_shared<Bus<MBusWidth>>();
  // auto debugger = std::make_shared<Debugger>(abus, mbus, true);
  Memory mem(abus, mbus);

  Cpu<Debugger> cpu(abus, mbus);

  // 0x4000 specific to test program from interwebs
  current_path(xstr(SOURCE_DIR));
  REQUIRE(cpu.loadRomFromFile(AllSuitePath, 0x4000));
  cpu.initPc(0x4000);

  while(true) {
    if (cpu.pc() == 0x45c0) break;
    cpu.step();
  }

  abus->put(0x0210);
  REQUIRE(mbus->get() == 0xff);
  // std::cout << "[0x0210]:  0x"
  //           << std::hex << std::setfill('0') << std::setw(2)
  //           << +mbus->get() << std::endl;
}
