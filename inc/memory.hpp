#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <sstream>

namespace mem {

template <typename A, typename D> class ArrayMapper {
public:
  using AddressT = A;
  using DataT = D;
  // TODO(oren): This isn't quite right. For example, a memory map for the
  // NES PPU will still represent addresses as 16-bit ints in **code**, but
  // the address space itself is only 14 bits wide.
  // AddressT is really an implementation detail and probably shouldn't be
  // included in these templates. Instead, could use the cpu type as template
  // parameter and pull these types straight from there (where they are
  // hard-coded anyway). Something like that.
  constexpr static size_t size = 1ul << (sizeof(AddressT) * 8);
  // static_cast<size_t>(pow(2, sizeof(AddressT) * 8));
  void write(AddressT addr, DataT data) {
    if (addr < mem_.size()) {
      mem_[addr] = data;
    } else {
      std::stringstream ss;
      ss << "Invalid Write: Address (0x" << std::hex << addr
         << ") out of bounds.";
      throw std::runtime_error(ss.str());
    }
  }

  DataT read(AddressT addr) {
    if (addr < mem_.size()) {
      return mem_[addr];
    } else {
      std::stringstream ss;
      ss << "Invalid Read: Address (0x" << std::hex << addr
         << ") out of bounds.";
      throw std::runtime_error(ss.str());
    }
  }

private:
  std::array<DataT, size> mem_;
};

template <typename T> class Bus {
public:
  explicit Bus() : data_(0) {}

  void put(T data) { put_callback_(data); }
  T get() { return get_callback_(); }
  void clear() { data_ = 0; }

  void setPutCallback(std::function<void(T)> cb) { put_callback_ = cb; }

  void setGetCallback(std::function<T(void)> cb) { get_callback_ = cb; }

private:
  T data_;
  std::function<void(T)> put_callback_ = [this](T data) { data_ = data; };
  std::function<T(void)> get_callback_ = [this]() { return data_; };
};

template <typename Mapper> class VRam {
public:
  using AddressT = typename Mapper::AddressT;
  using DataT = typename Mapper::DataT;
  explicit VRam() {
    data_bus_.setPutCallback(
        std::bind(&VRam::write, this, std::placeholders::_1));
    data_bus_.setGetCallback(std::bind(&VRam::read, this));
  }
  ~VRam() = default;
  size_t capacity() { return Mapper::size; }

  Bus<AddressT> &addressBus() { return address_bus_; }
  Bus<DataT> &dataBus() { return data_bus_; }

private:
  Mapper mapper_;
  Bus<AddressT> address_bus_;
  Bus<DataT> data_bus_;

  DataT read() {
    auto addr = address_bus_.get();
    return mapper_.read(addr);
  }

  void write(DataT data) {
    auto addr = address_bus_.get();
    mapper_.write(addr, data);
  }
};
} // namespace mem
