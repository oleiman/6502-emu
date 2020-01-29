#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <iostream>

#define MAX_MEM_SIZE 65536

namespace mem {

template<typename T>
class Bus {
public:
    explicit Bus()
        : _data(0)
    {}

    void put(T data) { _put_callback(data); }
    T get() { return _get_callback(); }
    void clear() { _data = 0; }

    void setPutCallback(std::function<void (T)> cb)
    {
        _put_callback = cb;
    }

    void setGetCallback(std::function<T (void)> cb)
    {
        _get_callback = cb;
    }

private:
    T _data;
    std::function<void (T)> _put_callback = [this](T data) {
        _data = data;
    };
    std::function<T (void)> _get_callback = [this]() {
        return _data;
    };

};

template<
    int size,
    typename AddressWidth,
    typename DataWidth
>
class Ram {
 public:
    explicit Ram(
        std::shared_ptr<Bus<AddressWidth>> abus,
        std::shared_ptr<Bus<DataWidth>> mbus)
        : _address_bus(abus)
        , _memory_bus(mbus)
    {
        _memory_bus->setPutCallback(std::bind(&Ram::write, this, std::placeholders::_1));
        _memory_bus->setGetCallback(std::bind(&Ram::read, this));
    }
    ~Ram() = default;
    uint32_t capacity() { return _mem.size(); }
  
 private:
    std::array<uint8_t, std::min(size, MAX_MEM_SIZE)> _mem;
    std::shared_ptr<Bus<AddressWidth>> _address_bus;
    std::shared_ptr<Bus<DataWidth>> _memory_bus;

    DataWidth read()
    {
        AddressWidth addr = _address_bus->get();
        if (addr < _mem.size()) {
            return _mem[addr];
        } else {
            std::stringstream ss;
            ss << "Invalid Read: Address (0x" << std::hex << addr << ") out of bounds.";
            throw std::runtime_error(ss.str());
        }
    }

    void write(DataWidth data)
    {
        AddressWidth addr = _address_bus->get();
        if (addr < _mem.size()) {
            _mem[addr] = data;
        } else {
            std::stringstream ss;
            ss << "Invalid Write: Address (0x" << std::hex << addr << ") out of bounds.";
            throw std::runtime_error(ss.str());
        }
            
    }
    
};

}
