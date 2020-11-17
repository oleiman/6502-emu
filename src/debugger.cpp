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
Debugger::Debugger(bool should_break) : break_(should_break), step_(false) {}

void Debugger::step(Instruction const &in, State const &state,
                    mem::Bus<AddressT> &address_bus,
                    mem::Bus<DataT> &data_bus) {
  step_ = false;
  break_ =
      break_ || any_of(breakpoints_.begin(), breakpoints_.end(),
                       [&in](BreakPoint bp) { return bp.shouldBreak(in); });

  while (break_ && !step_) {
    string command;
    cout << "> ";
    getline(cin, command);
    if (command == "run" || command == "continue") {
      break_ = false;
    } else if (command == "where" || command[0] == 'w') {
      if (prev_in_) {
        cout << *prev_in_ << endl;
      }
      cout << in << endl;
    } else if (command.find("break") == 0 || command[0] == 'b') {
      auto addr = extract_addr(command);
      cout << "setting breakpoint at 0x" << hex << setfill('0') << setw(4)
           << +addr << endl;
      breakpoints_.emplace_back(addr);
    } else if (command == "registers" || command[0] == 'r') {
      cout << state << endl;
    } else if (command == "step" || command[0] == 's') {
      step_ = true;
    } else if (command.find("examine") == 0 || command[0] == 'e') {
      auto addr = extract_addr(command);
      address_bus.put(addr);
      cout << "[" << hex << setfill('0') << setw(4) << +addr << "]\t" << hex
           << setfill('0') << setw(2) << +data_bus.get() << endl;
    } else {
      cout << "Unrecognized command: " << command << endl;
    }
  }
  prev_in_ = make_unique<Instruction>(in);
}

Debugger::AddressT Debugger::extract_addr(string const &command) {
  // TODO(oren): unsafe, consider std::optional
  auto addr_pos = command.find("0x") + 2;
  auto addr_s = command.substr(addr_pos);
  return stoi(addr_s, nullptr, 16);
}

bool BreakPoint::shouldBreak(Instruction const &in) const {
  if (in.pc() == pc_) {
    return true;
  }
  return false;
}

} // namespace dbg
