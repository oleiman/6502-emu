#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <sstream>

namespace mem {

class Mapper {
public:
  virtual void write(uint16_t addr, uint8_t data) = 0;
  virtual uint8_t read(uint16_t addr, bool dbg = false) = 0;
  virtual void tick(uint16_t) = 0;
};

class ArrayMapper : public Mapper {
public:
  constexpr static size_t size = 1ul << (sizeof(uint16_t) * 8);
  void write(uint16_t addr, uint8_t data) override {
    if (addr < mem_.size()) {
      mem_[addr] = data;
    } else {
      std::stringstream ss;
      ss << "Invalid Write: Address (0x" << std::hex << addr
         << ") out of bounds.";
      throw std::runtime_error(ss.str());
    }
  }

  uint8_t read(uint16_t addr, bool dbg) override {
    if (addr < mem_.size()) {
      return mem_[addr];
    } else {
      std::stringstream ss;
      ss << "Invalid Read: Address (0x" << std::hex << addr
         << ") out of bounds.";
      throw std::runtime_error(ss.str());
    }
  }

  void tick(uint16_t) override {}

private:
  std::array<uint8_t, size> mem_;
};

} // namespace mem
