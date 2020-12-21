#include "cpu.hpp"
#include "instruction.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <termios.h>
#include <thread>
#include <unistd.h>

// TODO(oren): yuck
#define xstr(s) str(s)
#define str(s) #s

#define CODE_SECTION 0xC000

#define OBUF 0xF001
#define IBUF 0xF004

#define VEC_P 0x0205
uint8_t vec_addr[4] = {0x00, 0xF1, 0x10, 0xF1};

// EhBASIC expects to find system-specific I/O routines *somewhere* in memory
// We're sticking them in an unused portion of the code section here to
// guarantee they don't get wiped out at run time.
#define IO_ROUTINE_P 0xF100
uint8_t io_routines[20] = {
    // INPUT
    0x18,             // CLC clear carry
    0xAD, 0x04, 0xF0, // LDA $F004
    0xC9, 0x00,       // CMP #00
    0xF0, 0x07,       // BEQ 0x08
    0xA0, 0x00,       // LDY #00
    0x8C, 0x04, 0xF0, // STY $F004
    0xC9, 0x00,       // CMP #00 (Z = 0, C = 1)
    0x60,             // RTS
    // OUTPUT
    0x8D, 0x01, 0xF0, // STA $F001
    0x60              // RTS
};

class EhBASIC {
  constexpr static int MemorySize = 0x10000;

public:
  EhBASIC(std::ifstream &romfile, std::function<void(void)> tick)
      : cpu(memory.addressBus(), memory.dataBus(), tick),
        input_buffer_(32, '\0'), input_thread_(&EhBASIC::readInput, this) {
    cpu.registerCallback(
        instr::Operation::loadA,
        std::bind(&EhBASIC::loadCB, this, std::placeholders::_1));
    cpu.registerCallback(
        instr::Operation::storeA,
        std::bind(&EhBASIC::storeCB, this, std::placeholders::_1));
    cpu.loadRom(romfile, CODE_SECTION);
    cpu.initPc(CODE_SECTION);
    initIOVectors();
  }

  ~EhBASIC() { input_thread_.join(); }

  void storeCB(cpu::M6502::AddressT addr) {
    if (addr == OBUF) {
      memory.addressBus().put(addr);
      std::cout << memory.dataBus().get();
    }
  }

  void loadCB(cpu::M6502::AddressT addr) {
    if (addr == IBUF) {
      memory.addressBus().put(addr);
      std::lock_guard<std::mutex> lg(im_);
      if (buf_read_ == buf_write_) {
        memory.dataBus().put('\0');
      } else {
        memory.dataBus().put(input_buffer_[buf_read_++]);
        buf_read_ %= input_buffer_.size();
      }
    }
  }

  void run() {
    while (true) {
      cpu.step();
    }
  }

private:
  mem::Ram<MemorySize, cpu::M6502::AddressT, cpu::M6502::DataT> memory;
  cpu::M6502 cpu;
  std::vector<char> input_buffer_;
  int buf_read_ = 0;
  int buf_write_ = 0;
  std::mutex im_;
  std::thread input_thread_;

  void readInput() {
    char c;
    while ((c = getchar()) != EOF) {
      std::lock_guard<std::mutex> lg(im_);
      if (c == '\n') {
        c = '\r';
      }
      input_buffer_[buf_write_] = c;
      buf_write_ = (buf_write_ + 1) % input_buffer_.size();
    }
  }

  void initIOVectors() {
    for (int i = 0; i < sizeof(vec_addr); i++) {
      memory.addressBus().put(VEC_P + i);
      memory.dataBus().put(vec_addr[i]);
    }
    for (int i = 0; i < sizeof(io_routines); i++) {
      memory.addressBus().put(IO_ROUTINE_P + i);
      memory.dataBus().put(io_routines[i]);
    }
  }
};

struct termios orig_termios;

void disableRawMode() { tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios); }

void enableRawMode() {
  tcgetattr(STDIN_FILENO, &orig_termios);
  atexit(disableRawMode);
  struct termios raw = orig_termios;
  tcgetattr(STDIN_FILENO, &raw);
  raw.c_lflag &= ~(ECHO | ICANON);

  tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

int main(int argc, char **argv) {
  std::filesystem::current_path(xstr(SOURCE_DIR));
  std::ifstream rom("test/rom/ehBASIC/ehbasic_alt.bin", std::ios::binary);
  int cycles = 0;
  auto tick = [&cycles]() { ++cycles; };
  EhBASIC basic(rom, tick);
  basic.run();
}
