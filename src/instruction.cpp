#include "instruction.hpp"

#include <array>
#include <iomanip>
#include <sstream>
#include <string>

using std::array;
using std::function;
using std::hex;
using std::ostream;
using std::setfill;
using std::setw;
using std::string;
using std::stringstream;

namespace instr {

// instruction sizes for the various addressing modes
array<uint8_t, static_cast<int>(AddressMode::nAddressModes)> aModeSizes = {
    1,       // implicit
    1,       // accumulator
    2,       // immediate
    3, 3, 3, // absolute
    2, 2, 2, // zero page
    2,       // relative
    3,       // indirect
    2,       // indexed indirect
    2        // indirect indexed
};

array<string, static_cast<int>(Operation::nOperations)> opMnemonics = {
    "ILLEGAL", "LDA", "LDX", "LDY", "LAX", "STA", "STX", "STY", "ADC",
    "SBC",     "INC", "INX", "INY", "DEC", "DEX", "DEY", "ASL", "ASL",
    "LSR",     "LSR", "ROL", "ROL", "ROR", "ROR", "AND", "ORA", "EOR",
    "CMP",     "CPX", "CPY", "BIT", "BPL", "BVC", "BCC", "BNE", "BMI",
    "BVS",     "BCS", "BEQ", "TAX", "TAY", "TSX", "TXA", "TXS", "TYA",
    "JMP",     "JSR", "RTS", "RTI", "PHA", "PLA", "PHP", "PLP", "BRK",
    "CLC",     "CLI", "CLV", "CLD", "SEC", "SEI", "SED", "NOP"};

array<string, static_cast<int>(AddressMode::nAddressModes)> aModeMnemonics = {
    "Imp",   "Accum", "Imm", "Abs",   "AbsX",     "AbsY",     "Zero",
    "ZeroX", "ZeroY", "Rel", "Indir", "IdxIndir", "IndirIdx",
};

Instruction::Instruction(DataT opcode, AddressT pc, unsigned long long cycle)
    : opcode(opcode), issueCycle(cycle), pc(pc),
      addressMode(decodeAddressMode()),
      size(aModeSizes[static_cast<int>(addressMode)]),
      operation(decodeOperation()) {}

// TODO(oren): This is a very expensive lookup table. Refactor.
AddressMode Instruction::decodeAddressMode() {
  DataT lo = opcode & 0x0F;
  DataT hi = (opcode & 0xF0) >> 4;

  switch (lo) {
  case 0x00:
    if (hi == 0x00 || hi == 0x04 || hi == 0x06) {
      return AddressMode::implicit;
    } else if (hi == 0x02) {
      return AddressMode::absolute;
    } else if ((hi & 0x01) == 0) {
      // NOTE: 0x80 is unofficial nop
      return AddressMode::immediate;
    } else {
      return AddressMode::relative;
    }
  case 0x01:
    if ((hi & 0x01) == 0) {
      return AddressMode::indexedIndirect;
    } else {
      return AddressMode::indirectIndexed;
    }
  case 0x02:
    if (hi == 0x0A) {
      return AddressMode::immediate;
    } else {
      // NOTE: rest are illegal
      return AddressMode::implicit;
    }
  case 0x03:
    // NOTE: all illegal
    return AddressMode::implicit;
  case 0x04:
    if ((hi & 0x01) == 0) {
      return AddressMode::zeroPage;
    } else {
      // includes 0x14, unofficial zpx nop
      return AddressMode::zeroPageX;
    }
  case 0x05:
    if ((hi & 0x01) == 0) {
      return AddressMode::zeroPage;
    } else {
      return AddressMode::zeroPageX;
    }
  case 0x06:
    if (hi == 0x09 || hi == 0x0B) {
      return AddressMode::zeroPageY;
    } else if ((hi & 0x01) == 0) {
      return AddressMode::zeroPage;
    } else {
      return AddressMode::zeroPageX;
    }
  case 0x07:
    // NOTE: all illegal
    return AddressMode::implicit;
  case 0x08:
    return AddressMode::implicit;
  case 0x09:
    if (hi == 0x08) {
      // NOTE: illegal
      return AddressMode::implicit;
    } else if ((hi & 0x01) == 0) {
      return AddressMode::immediate;
    } else {
      return AddressMode::absoluteY;
    }
  case 0x0A:
    if ((hi & 0x01) == 0 && hi < 0x08) {
      return AddressMode::accumulator;
    } else {
      // NOTE: 1, 3, 5, 7, D, F unofficial
      return AddressMode::implicit;
    }
  case 0x0B:
    // NOTE: all either illegal or unofficial
    if (hi == 0x0E) { // unofficial SBC immediate
      return AddressMode::immediate;
    } else {
      return AddressMode::implicit;
    }

  case 0x0C:
    if (hi == 0x06) {
      return AddressMode::indirect;
    } else if (hi & 0x01) {
      // some are unofficial
      return AddressMode::absoluteX;
    } else {
      // NOTE: some are unofficial, but all are absolute
      return AddressMode::absolute;
    }
  case 0x0D:
    if ((hi & 0x01) == 0) {
      return AddressMode::absolute;
    } else {
      return AddressMode::absoluteX;
    }
  case 0x0E:
    if (hi == 0x09) {
      // NOTE: illegal
      return AddressMode::implicit;
    } else if (hi == 0x0B) {
      return AddressMode::absoluteY;
    } else if ((hi & 0x01) == 0) {
      return AddressMode::absolute;
    } else {
      return AddressMode::absoluteX;
    }
  case 0x0F:
    // NOTE: all illegal
    return AddressMode::implicit;
  default:
    return AddressMode::implicit;
  }
}

Operation Instruction::decodeOperation() {
  switch (opcode) {
  case 0xA1:
  case 0xA5:
  case 0xA9:
  case 0xAD:
  case 0xB1:
  case 0xB5:
  case 0xB9:
  case 0xBD:
    return Operation::loadA;
  case 0xA2:
  case 0xA6:
  case 0xAE:
  case 0xB6:
  case 0xBE:
    return Operation::loadX;
  case 0xA0:
  case 0xA4:
  case 0xAC:
  case 0xB4:
  case 0xBC:
    return Operation::loadY;
  case 0xA7:
  case 0xB7:
  case 0xAF:
  case 0xBF:
  case 0xA3:
  case 0xB3:
    return Operation::loadAX;
  case 0x81:
  case 0x85:
  case 0x8D:
  case 0x91:
  case 0x95:
  case 0x99:
  case 0x9D:
    return Operation::storeA;
  case 0x86:
  case 0x8E:
  case 0x96:
    return Operation::storeX;
  case 0x84:
  case 0x8C:
  case 0x94:
    return Operation::storeY;
  case 0x61:
  case 0x65:
  case 0x69:
  case 0x6D:
  case 0x71:
  case 0x75:
  case 0x79:
  case 0x7D:
    return Operation::add;
  case 0xE1:
  case 0xE5:
  case 0xE9:
  case 0xEB: // unofficial
  case 0xED:
  case 0xF1:
  case 0xF5:
  case 0xF9:
  case 0xFD:
    return Operation::subtract;
  case 0xE6:
  case 0xEE:
  case 0xF6:
  case 0xFE:
    return Operation::increment;
  case 0xE8:
    return Operation::incrementX;
  case 0xC8:
    return Operation::incrementY;
  case 0xC6:
  case 0xCE:
  case 0xD6:
  case 0xDE:
    return Operation::decrement;
  case 0xCA:
    return Operation::decrementX;
  case 0x88:
    return Operation::decrementY;
  case 0x06:
  case 0x0E:
  case 0x16:
  case 0x1E:
    return Operation::shiftL;
  case 0x0A:
    return Operation::shiftLA;
  case 0x46:
  case 0x4E:
  case 0x56:
  case 0x5E:
    return Operation::shiftR;
  case 0x4A:
    return Operation::shiftRA;
  case 0x26:
  case 0x2E:
  case 0x36:
  case 0x3E:
    return Operation::rotateL;
  case 0x2A:
    return Operation::rotateLA;
  case 0x66:
  case 0x6E:
  case 0x76:
  case 0x7E:
    return Operation::rotateR;
  case 0x6A:
    return Operation::rotateRA;
  case 0x21:
  case 0x25:
  case 0x29:
  case 0x2D:
  case 0x31:
  case 0x35:
  case 0x39:
  case 0x3D:
    return Operation::bwAND;
  case 0x01:
  case 0x05:
  case 0x09:
  case 0x0D:
  case 0x11:
  case 0x15:
  case 0x19:
  case 0x1D:
    return Operation::bwOR;
  case 0x41:
  case 0x45:
  case 0x49:
  case 0x4D:
  case 0x51:
  case 0x55:
  case 0x59:
  case 0x5D:
    return Operation::bwXOR;
  case 0xC1:
  case 0xC5:
  case 0xC9:
  case 0xCD:
  case 0xD1:
  case 0xD5:
  case 0xD9:
  case 0xDD:
    return Operation::compare;
  case 0xE0:
  case 0xE4:
  case 0xEC:
    return Operation::compareX;
  case 0xC0:
  case 0xC4:
  case 0xCC:
    return Operation::compareY;
  case 0x24:
  case 0x2C:
    return Operation::bitTest;
  case 0x10:
    return Operation::branchPos;
  case 0x50:
    return Operation::branchVC;
  case 0x90:
    return Operation::branchCC;
  case 0xD0:
    return Operation::branchNE;
  case 0x30:
    return Operation::branchNeg;
  case 0x70:
    return Operation::branchVS;
  case 0xB0:
    return Operation::branchCS;
  case 0xF0:
    return Operation::branchEQ;
  case 0xAA:
    return Operation::transferAX;
  case 0xA8:
    return Operation::transferAY;
  case 0xBA:
    return Operation::transferSX;
  case 0x8A:
    return Operation::transferXA;
  case 0x9A:
    return Operation::transferXS;
  case 0x98:
    return Operation::transferYA;
  case 0x4C:
  case 0x6C:
    return Operation::jump;
  case 0x20:
    return Operation::jumpSR;
  case 0x60:
    return Operation::returnSR;
  case 0x40:
    return Operation::returnINT;
  case 0x48:
    return Operation::pushA;
  case 0x68:
    return Operation::pullA;
  case 0x08:
    return Operation::pushS;
  case 0x28:
    return Operation::pullS;
  case 0x00:
    return Operation::forceBreak;
  case 0x18:
    return Operation::clearC;
  case 0x58:
    return Operation::clearI;
  case 0xB8:
    return Operation::clearV;
  case 0xD8:
    return Operation::clearD;
  case 0x38:
    return Operation::setC;
  case 0x78:
    return Operation::setI;
  case 0xF8:
    return Operation::setD;
  case 0x80:
  case 0x0C:
  case 0x1C:
  case 0x3C:
  case 0x5C:
  case 0x7C:
  case 0xDC:
  case 0xFC:
  case 0x04:
  case 0x14:
  case 0x34:
  case 0x44:
  case 0x54:
  case 0x64:
  case 0x74:
  case 0xD4:
  case 0xF4:
  case 0x1A:
  case 0x3A:
  case 0x5A:
  case 0x7A:
  case 0xDA:
  case 0xEA:
  case 0xFA:
    return Operation::nop;
  default:
    return Operation::illegal;
  }
}

ostream &operator<<(ostream &os, Instruction const &in) {
  stringstream ss;
  ss << "["
     << "0x" << hex << setfill('0') << setw(4) << +in.pc << "]\t"
     << opMnemonics[static_cast<int>(in.operation)] << " ";

  if (in.operation == Operation::illegal) {
    ss << "(0x" << setfill('0') << setw(2) << +in.opcode << ") ";
  }

  // TODO(oren): currently printing addresses in big endian
  switch (in.addressMode) {
  case AddressMode::implicit:
    break;
  case AddressMode::accumulator:
    ss << "A";
    break;
  default:
    ss << "$" << setw(4) << +in.address << " ("
       << aModeMnemonics[static_cast<int>(in.addressMode)] << ")";
    break;
  }
  ss << " C: " << std::dec << in.issueCycle + 7;
  os << ss.str();
  return os;
}

} // namespace instr
