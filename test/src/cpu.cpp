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

using cpu::M6502;
using mem::Ram;

constexpr int memory_size = 0x10000;

TEST_CASE("AllSuiteA", "[integration][cpu]") {
  Ram<memory_size, M6502::AddressT, M6502::DataT> memory;

  M6502 cpu(memory.addressBus(), memory.dataBus());

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
