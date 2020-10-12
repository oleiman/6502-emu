#include "debugger.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>

using cpu::State;
using instr::Instruction;

using std::any_of;
using std::cin;
using std::cout;
using std::endl;
using std::getline;
using std::hex;
using std::make_unique;
using std::setfill;
using std::setw;
using std::stoi;
using std::string;

namespace dbg {
Debugger::Debugger(std::shared_ptr<mem::Bus<uint16_t>> abus,
                   std::shared_ptr<mem::Bus<uint8_t>> mbus, bool brek)
    : _address_bus(abus), _memory_bus(mbus), _break(brek), _step(false) {}

void Debugger::step(Instruction const &in, State const &state) {
  _step = false;
  _break =
      _break || any_of(_breakpoints.begin(), _breakpoints.end(),
                       [&in](BreakPoint bp) { return bp.shouldBreak(in); });

  while (_break && !_step) {
    string command;
    cout << "> ";
    getline(cin, command);
    if (command == "run" || command == "continue") {
      _break = false;
    } else if (command == "where") {
      if (_prev_in) {
        cout << *_prev_in << endl;
      }
      cout << in << endl;
    } else if (command.find("break") == 0) {
      auto addr = extract_addr(command);
      cout << "setting breakpoint at 0x" << hex << setfill('0') << setw(4)
           << +addr << endl;
      _breakpoints.emplace_back(addr);
    } else if (command == "registers") {
      cout << state << endl;
    } else if (command == "step") {
      _step = true;
    } else if (command.find("examine") == 0) {
      auto addr = extract_addr(command);
      _address_bus->put(addr);
      cout << "[" << hex << setfill('0') << setw(4) << +addr << "]\t" << hex
           << setfill('0') << setw(2) << +_memory_bus->get() << endl;
    } else {
      cout << "Unrecognized command: " << command << endl;
    }
  }
  _prev_in = make_unique<Instruction>(in);
}

uint16_t Debugger::extract_addr(string const &command) {
  // TODO(oren): unsafe, consider std::optional
  auto addr_pos = command.find("0x") + 2;
  auto addr_s = command.substr(addr_pos);
  return stoi(addr_s, nullptr, 16);
}

bool BreakPoint::shouldBreak(Instruction const &in) {
  if (in.pc() == _pc) {
    return true;
  }
  return false;
}

} // namespace dbg
