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
using mem::ArrayMapper;

TEST_CASE("AllSuiteA", "[integration][cpu]") {
  ArrayMapper mp;
  int instructions = 0;

  M6502 cpu(mp);

  current_path(xestr(SOURCE_DIR));
  std::ifstream infile(AllSuiteA, std::ios::binary);
  REQUIRE(cpu.loadRom(infile, 0x4000));
  infile.close();
  cpu.initPc(0x4000);

  do {
    ++instructions;
    cpu.step();
  } while (cpu.state().pc != 0x45c0);

  REQUIRE(mp.read(0x0210) == 0xff);

  std::cerr << report(AllSuiteA, cpu.state().cycle, instructions).str()
            << std::endl;
}

TEST_CASE("KlausFunctional", "[integration][cpu]") {
  ArrayMapper mp;
  int instructions = 0;

  M6502 cpu(mp);

  current_path(xestr(SOURCE_DIR));
  std::ifstream infile(KlausFunctional, std::ios::binary);
  REQUIRE(cpu.loadRom(infile, 0x0000u));
  infile.close();
  cpu.initPc(0x400u);

  dbg::Debugger d(true, false);

  try {
    do {
      // cpu.debugStep(d);
      ++instructions;
      cpu.step();
    } while (cpu.state().pc != 0x3469);
  } catch (std::runtime_error e) {
    std::cerr << e.what() << std::endl;
  }

  REQUIRE(cpu.state().pc == 0x3469);
  std::cerr << report(KlausFunctional, cpu.state().cycle, instructions).str()
            << std::endl;
}

TEST_CASE("BruceClarkDecimal", "[integration][cpu]") {
  ArrayMapper mp;
  int instructions = 0;

  M6502 cpu(mp);

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
    } while (cpu.state().pc != 0x025b);
  } catch (std::runtime_error e) {
    std::cerr << e.what() << std::endl;
  }

  REQUIRE(cpu.state().pc == 0x025b);
  // REQUIRE(cpu.state().cycle == 7915081);
  std::cerr << report(BruceClarkDecimal, cpu.state().cycle, instructions).str()
            << std::endl;
}

TEST_CASE("Timing", "[cpu][timing]") {
  ArrayMapper mp;
  int instructions = 0;

  M6502 cpu(mp);

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
    } while (cpu.state().pc != 0x1269);
  } catch (std::runtime_error e) {
    std::cerr << e.what() << std::endl;
  }

  REQUIRE(cpu.state().pc == 0x1269);

  // cycle count here is somewhat arbitrary in that it just reflects the current
  // state of this emulation.
  // Big Ed's listing reports expected cycle counts as follows
  //    1130 cycles from the Kowalski emulator
  //    1141 cycles from visual6502
  // with the latter recommended as a reasonable source of "truth"
  // fairly certain this doesn't include the final jump back to the start
  // so I'm going to say we've hit our target here
  REQUIRE(cpu.state().cycle == 1141);
  std::cerr << report(Timing, cpu.state().cycle, instructions).str()
            << std::endl;
}

// TODO(oren): assemble this test
// TEST_CASE("KlausInterrupt", "[integration][cpu]") {
//   Ram<ArrayMapper> memory;
//   int instructions = 0;
//   M6502 cpu(memory.addressBus(), memory.dataBus());

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
//     } while (cpu.state().pc != 0x025b);
//   } catch (std::runtime_error e) {
//     std::cerr << e.what() << std::endl;
//   }

//   REQUIRE(cpu.state().pc == 0x025b);
//   std::cerr << report(KlausInterrupt, cpu.state().cycle, instructions).str()
//   << std::endl;
// }
