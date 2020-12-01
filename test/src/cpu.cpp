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
  int cycles = 0;
  auto tick = [&cycles]() { ++cycles; };

  M6502 cpu(memory.addressBus(), memory.dataBus(), tick);

  // 0x4000 specific to test program from interwebs
  current_path(xstr(SOURCE_DIR));
  std::ifstream infile(AllSuitePath, std::ios::binary);
  REQUIRE(cpu.loadRom(infile, 0x4000));
  infile.close();
  cpu.initPc(0x4000);

  while (true) {
    if (cpu.pc() == 0x45c0)
      break;
    cpu.step();
  }

  memory.addressBus().put(0x0210);
  REQUIRE(memory.dataBus().get() == 0xff);

  std::cerr << cycles << std::endl;
}

TEST_CASE("KlausFunctional", "[integration][cpu]") {
  Ram<memory_size, M6502::AddressT, M6502::DataT> memory;
  int cycles = 0;
  auto tick = [&cycles]() { ++cycles; };

  M6502 cpu(memory.addressBus(), memory.dataBus(), tick);

  // 0x400 specific to test program from interwebs
  current_path(xstr(SOURCE_DIR));
  std::ifstream infile(KlausFunctionalPath, std::ios::binary);
  REQUIRE(cpu.loadRom(infile, 0x0000u));
  infile.close();
  cpu.initPc(0x400u);

  // dbg::Debugger d(true, false);

  // BENCHMARK("Klaus Functional Tests") {
  try {
    do {
      // cpu.debugStep(d);
      cpu.step();
    } while (cpu.pc() != 0x3469);
  } catch (std::runtime_error e) {
    std::cerr << e.what() << std::endl;
  }
  // }

  // memory.addressBus().put(0x0210);
  REQUIRE(cpu.pc() == 0x3469);
  std::cerr << cycles << std::endl;
}

TEST_CASE("KlausDecimal", "[integration][cpu]") {
  Ram<memory_size, M6502::AddressT, M6502::DataT> memory;
  int cycles = 0;
  auto tick = [&cycles]() { ++cycles; };

  M6502 cpu(memory.addressBus(), memory.dataBus(), tick);

  // 0x400 specific to test program from interwebs
  current_path(xstr(SOURCE_DIR));
  std::ifstream infile(KlausDecimalPath, std::ios::binary);
  REQUIRE(cpu.loadRom(infile, 0x0200u));
  infile.close();
  cpu.initPc(0x200u);

  // dbg::Debugger d(true, false);

  // BENCHMARK("Klaus Functional Tests") {
  try {
    do {
      // cpu.debugStep(d);
      cpu.step();
    } while (cpu.pc() != 0x024b);
  } catch (std::runtime_error e) {
    std::cerr << e.what() << std::endl;
  }
  // }

  // memory.addressBus().put(0x0210);
  REQUIRE(cpu.pc() == 0x024b);
  // REQUIRE(cycles == 3807);
  std::cerr << cycles << std::endl;
}
