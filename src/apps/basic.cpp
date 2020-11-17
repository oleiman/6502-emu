#include "cpu.hpp"
#include "debugger.hpp"
#include "instruction.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>

// TODO(oren): yuck
#define xstr(s) str(s)
#define str(s) #s

inline constexpr uint8_t operator"" _uint8(unsigned long long v) {
  return static_cast<uint8_t>(v);
}

// TODO(oren): need to set input and output vectors!
// implement cpu::WriteRAM to write between zero page and code
// store character to F001
// vector should go in 0x0207

#define CODE_SECTION 0xC000

#define OBUF 0xF001
#define IBUF 0xF004

#define VEC_P 0x0205
uint8_t vec_addr[4] = {0x00, 0xB0, 0x10, 0xB0};

#define IO_ROUTINE_P 0xB000
uint8_t io_routines[20] = {
    // INPUT
    0x18,             // CLC clear carry
    0xAD, 0x04, 0xF0, // LDA $F004
    0xC9, 0x00,       // CMP #00
    0xF0, 0x07,       // BEQ 0x08
    // 0x38,          // SEC set carry
    0xA0, 0x00,       // LDY #00
    0x8C, 0x04, 0xF0, // STY $F004
    0xC9, 0x00,       // CMP #00 (Z = 0, C = 1)
    0x60,             // RTS
    // OUTPUT
    0x8D, 0x01, 0xF0, // STA $F001
    0x60              // RTS
};

// #define VEC_IN_P 0x0209
// uint8_t vec_in[2] = {0x04, 0xB0};

class EhBASIC {
  constexpr static int MemorySize = 0x10000;

public:
  EhBASIC(std::ifstream &infile)
      : cpu(memory.addressBus(), memory.dataBus()), debugger(true) {

    cpu.registerCallback(
        instr::Operation::loadA,
        std::bind(&EhBASIC::loadCB, this, std::placeholders::_1));
    cpu.registerCallback(
        instr::Operation::storeA,
        std::bind(&EhBASIC::storeCB, this, std::placeholders::_1));
    cpu.loadRom(infile, CODE_SECTION);
    cpu.initPc(CODE_SECTION);
    cpu.writeRAM(VEC_P, vec_addr, sizeof(vec_addr));
    cpu.writeRAM(IO_ROUTINE_P, io_routines, sizeof(io_routines));
  }

  void storeCB(cpu::M6502::AddressT addr) {
    if (addr == OBUF) {
      // cpu::M6502::AddressT tmp = memory.addressBus().get();
      memory.addressBus().put(addr);
      std::cout << memory.dataBus().get();
      // memory.addressBus().put(tmp);
    }
  }

  void loadCB(cpu::M6502::AddressT addr) {
    if (addr == IBUF) {
      // cpu::M6502::AddressT tmp = memory.addressBus().get();
      memory.addressBus().put(addr);
      // uint8_t c = getchar();
      // if (count++ < 10) {
      // std::cout << "woot" << std::endl;
      // memory.dataBus().put('0' + count);

      // need to be in raw mode
      if (count < 10) {
        memory.dataBus().put('0' + count++);
      } else {
        memory.dataBus().put((char)0xD);
        count = 0;
      }
      // std::cout << "got: " << c << std::endl;

      // } else {
      //   memory.dataBus().put('\n');
      // }
      // if (c == 0x0d) should_debug = true;
      // memory.addressBus().put(tmp);
    }
  }

  void run() {
    // bool should_debug = true;
    while (true) {
      if (should_debug) {
        cpu.debugStep(debugger);
      } else {
        cpu.step();
      }
      // std::cout << std::hex << +memory.addressBus().get() << std::endl;
      // if (memory.addressBus().get() == OBUF) {
      //   std::cout << memory.dataBus().get();
      // } else if (memory.addressBus().get() == IBUF) {
      //   uint8_t c = '1'; // getchar();
      //   std::cout << c << std::endl;
      //   // cpu.writeRAM(IBUF, &c, 1);
      //   should_debug = true;
      //   memory.dataBus().put(c);
      // }
    }
  }

private:
  mem::Ram<MemorySize, cpu::M6502::AddressT, cpu::M6502::DataT> memory;
  cpu::M6502 cpu;
  dbg::Debugger debugger;
  bool should_debug = true;
  int count = 0;
};

int main(int argc, char **argv) {
  std::filesystem::current_path(xstr(SOURCE_DIR));
  std::ifstream infile("test/rom/ehBASIC/ehbasic_alt.bin", std::ios::binary);
  EhBASIC basic(infile);
  basic.run();
}
