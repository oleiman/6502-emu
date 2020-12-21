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

#define xestr(s) estr(s)
#define estr(s) #s

using std::make_shared;
using std::filesystem::current_path;

using cpu::M6502;
using mem::Ram;

constexpr int memory_size = 0x10000;

TEST_CASE("AllSuiteA", "[integration][cpu]") {
  Ram<memory_size, M6502::AddressT, M6502::DataT> memory;
  int cycles = 0;
  int instructions = 0;
  auto tick = [&cycles]() { ++cycles; };

  M6502 cpu(memory.addressBus(), memory.dataBus(), tick);

  // 0x4000 specific to test program from interwebs
  current_path(xestr(SOURCE_DIR));
  std::ifstream infile(AllSuiteA, std::ios::binary);
  REQUIRE(cpu.loadRom(infile, 0x4000));
  infile.close();
  cpu.initPc(0x4000);

  do {
    ++instructions;
    cpu.step();
  } while (cpu.pc() != 0x45c0);

  memory.addressBus().put(0x0210);
  REQUIRE(memory.dataBus().get() == 0xff);

  std::cerr << report(AllSuiteA, cycles, instructions).str() << std::endl;
}

TEST_CASE("KlausFunctional", "[integration][cpu]") {
  Ram<memory_size, M6502::AddressT, M6502::DataT> memory;
  int cycles = 0;
  int instructions = 0;
  auto tick = [&cycles]() { ++cycles; };

  M6502 cpu(memory.addressBus(), memory.dataBus(), tick);

  current_path(xestr(SOURCE_DIR));
  std::ifstream infile(KlausFunctional, std::ios::binary);
  REQUIRE(cpu.loadRom(infile, 0x0000u));
  infile.close();
  cpu.initPc(0x400u);

  // dbg::Debugger d(true, false);

  try {
    do {
      // cpu.debugStep(d);
      ++instructions;
      cpu.step();
    } while (cpu.pc() != 0x3469);
  } catch (std::runtime_error e) {
    std::cerr << e.what() << std::endl;
  }

  REQUIRE(cpu.pc() == 0x3469);
  std::cerr << report(KlausFunctional, cycles, instructions).str() << std::endl;
}

TEST_CASE("BruceClarkDecimal", "[integration][cpu]") {
  Ram<memory_size, M6502::AddressT, M6502::DataT> memory;
  int cycles = 0;
  int instructions = 0;
  auto tick = [&cycles]() { ++cycles; };

  M6502 cpu(memory.addressBus(), memory.dataBus(), tick);

  // 0x400 specific to test program from interwebs
  current_path(xestr(SOURCE_DIR));
  std::ifstream infile(BruceClarkDecimal, std::ios::binary);
  REQUIRE(cpu.loadRom(infile, 0x0200u));
  infile.close();
  cpu.initPc(0x200u);

  // dbg::Debugger d(true, false);

  try {
    do {
      // cpu.debugStep(d);
      ++instructions;
      cpu.step();
    } while (cpu.pc() != 0x025b);
  } catch (std::runtime_error e) {
    std::cerr << e.what() << std::endl;
  }

  REQUIRE(cpu.pc() == 0x025b);
  // REQUIRE(cycles == 7915081);
  std::cerr << report(BruceClarkDecimal, cycles, instructions).str()
            << std::endl;
}

TEST_CASE("Timing", "[cpu][timing]") {
  Ram<memory_size, M6502::AddressT, M6502::DataT> memory;
  int cycles = 0;
  int instructions = 0;
  auto tick = [&cycles]() { ++cycles; };

  M6502 cpu(memory.addressBus(), memory.dataBus(), tick);

  // 0x400 specific to test program from interwebs
  current_path(xestr(SOURCE_DIR));
  std::ifstream infile(Timing, std::ios::binary);
  REQUIRE(cpu.loadRom(infile, 0x1000));
  infile.close();
  cpu.initPc(0x1000);

  // dbg::Debugger d(true, false);

  try {
    do {
      // cpu.debugStep(d);
      ++instructions;
      cpu.step();
    } while (cpu.pc() != 0x1000);
  } catch (std::runtime_error e) {
    std::cerr << e.what() << std::endl;
  }

  REQUIRE(cpu.pc() == 0x1000);

  // cycle count here is somewhat arbitrary in that it just reflects the current
  // state of this emulation.
  // Big Ed's listing reports expected cycle counts as follows
  //    1130 cycles from the Kowalski emulator
  //    1141 cycles from visual6502
  // with the latter recommended as a reasonable source of "truth"
  // I'm fairly certain the four missing cycles are from certain
  // instructions reaping the no-page-crossing optimization during
  // address calculation when they shouldn't. see ASL, etc.
  REQUIRE(cycles == 1137);
  std::cerr << report(Timing, cycles, instructions).str() << std::endl;
}

// TODO(oren): assemble this test
// TEST_CASE("KlausInterrupt", "[integration][cpu]") {
//   Ram<memory_size, M6502::AddressT, M6502::DataT> memory;
//   int cycles = 0;
//   int instructions = 0;
//   auto tick = [&cycles]() { ++cycles; };

//   M6502 cpu(memory.addressBus(), memory.dataBus(), tick);

//   // 0x400 specific to test program from interwebs
//   current_path(xestr(SOURCE_DIR));
//   std::ifstream infile(KlausInterrupt, std::ios::binary);
//   REQUIRE(cpu.loadRom(infile, 0x0000u));
//   infile.close();
//   cpu.initPc(0x400u);

//   // dbg::Debugger d(true, false);

//   try {
//     do {
//       // cpu.debugStep(d);
//       ++instructions;
//       cpu.step();
//     } while (cpu.pc() != 0x025b);
//   } catch (std::runtime_error e) {
//     std::cerr << e.what() << std::endl;
//   }

//   REQUIRE(cpu.pc() == 0x025b);
//   std::cerr << report(KlausInterrupt, cycles, instructions).str() <<
//   std::endl;
// }
