#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>

#define MAX_MEM_SIZE 65536

namespace mem {

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

template <int size, typename AddressT, typename DataT> class Ram {
public:
  explicit Ram() {
    data_bus_.setPutCallback(
        std::bind(&Ram::write, this, std::placeholders::_1));
    data_bus_.setGetCallback(std::bind(&Ram::read, this));
  }
  ~Ram() = default;
  uint32_t capacity() { return buffer_.size(); }

  Bus<AddressT> &addressBus() { return address_bus_; }
  Bus<DataT> &dataBus() { return data_bus_; }

private:
  std::array<uint8_t, std::min(size, MAX_MEM_SIZE)> buffer_;
  Bus<AddressT> address_bus_;
  Bus<DataT> data_bus_;

  DataT read() {
    AddressT addr = address_bus_.get();
    if (addr < buffer_.size()) {
      return buffer_[addr];
    } else {
      std::stringstream ss;
      ss << "Invalid Read: Address (0x" << std::hex << addr
         << ") out of bounds.";
      throw std::runtime_error(ss.str());
    }
  }

  void write(DataT data) {
    AddressT addr = address_bus_.get();
    if (addr < buffer_.size()) {
      buffer_[addr] = data;
    } else {
      std::stringstream ss;
      ss << "Invalid Write: Address (0x" << std::hex << addr
         << ") out of bounds.";
      throw std::runtime_error(ss.str());
    }
  }
}; // namespace mem

} // namespace mem
