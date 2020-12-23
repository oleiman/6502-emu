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
    "ILLEGAL", "LDA", "LDX", "LDY", "STA", "STX", "STY", "ADC", "SBC",
    "INC",     "INX", "INY", "DEC", "DEX", "DEY", "ASL", "ASL", "LSR",
    "LSR",     "ROL", "ROL", "ROR", "ROR", "AND", "ORA", "EOR", "CMP",
    "CPX",     "CPY", "BIT", "BPL", "BVC", "BCC", "BNE", "BMI", "BVS",
    "BCS",     "BEQ", "TAX", "TAY", "TSX", "TXA", "TXS", "TYA", "JMP",
    "JSR",     "RTS", "RTI", "PHA", "PLA", "PHP", "PLP", "BRK", "CLC",
    "CLI",     "CLV", "CLD", "SEC", "SEI", "SED", "NOP"};

array<string, static_cast<int>(AddressMode::nAddressModes)> aModeMnemonics = {
    "Imp",   "Accum", "Imm", "Abs",   "AbsX",     "AbsY",     "Zero",
    "ZeroX", "ZeroY", "Rel", "Indir", "IdxIndir", "IndirIdx",
};

Instruction::Instruction(DataT opcode, AddressT pc,
                         function<AddressT(AddressMode)> calc_addr)
    : opcode_(opcode), size_(1), address_mode_(AddressMode::implicit),
      operation_(Operation::illegal), pc_(pc) {
  decodeAddressMode();
  decodeOperation();
  address_ = calc_addr(address_mode_);
}

void Instruction::decodeAddressMode() {
  DataT lo = opcode_ & 0x0F;
  DataT hi = (opcode_ & 0xF0) >> 4;

  switch (lo) {
  case 0x00:
    if (hi == 0x00 || hi == 0x04 || hi == 0x06 || hi == 0x08) {
      // NOTE: 0x80 is illegal
      address_mode_ = AddressMode::implicit;
    } else if (hi == 0x02) {
      address_mode_ = AddressMode::absolute;
      size_ = 3;
    } else if (hi % 2 == 0) {
      address_mode_ = AddressMode::immediate;
      size_ = 2;
    } else {
      address_mode_ = AddressMode::relative;
    }
    break;
  case 0x01:
    if (hi % 2 == 0) {
      address_mode_ = AddressMode::indexedIndirect;
    } else {
      address_mode_ = AddressMode::indirectIndexed;
    }
    break;
  case 0x02:
    if (hi == 0x0A) {
      address_mode_ = AddressMode::immediate;
    } else {
      // NOTE: rest are illegal
      address_mode_ = AddressMode::implicit;
    }
    break;
  case 0x03:
    // NOTE: all illegal
    address_mode_ = AddressMode::implicit;
    break;
  case 0x04:
    if (hi == 0x00 || hi == 0x01 || (hi >= 0x03 && hi <= 0x07) || hi == 0x0D ||
        hi == 0x0F) {
      address_mode_ = AddressMode::implicit;
    } else if (hi % 2 == 0) {
      address_mode_ = AddressMode::zeroPage;
    } else {
      address_mode_ = AddressMode::zeroPageX;
    }
    break;
  case 0x05:
    if (hi % 2 == 0) {
      address_mode_ = AddressMode::zeroPage;
    } else {
      address_mode_ = AddressMode::zeroPageX;
    }
    break;
  case 0x06:
    if (hi == 0x09 || hi == 0x0B) {
      address_mode_ = AddressMode::zeroPageY;
    } else if (hi % 2 == 0) {
      address_mode_ = AddressMode::zeroPage;
    } else {
      address_mode_ = AddressMode::zeroPageX;
    }
    break;
  case 0x07:
    // NOTE: all illegal
    address_mode_ = AddressMode::implicit;
    break;
  case 0x08:
    address_mode_ = AddressMode::implicit;
    break;
  case 0x09:
    if (hi == 0x08) {
      // NOTE: illegal
      address_mode_ = AddressMode::implicit;
    } else if (hi % 2 == 0) {
      address_mode_ = AddressMode::immediate;
    } else {
      address_mode_ = AddressMode::absoluteY;
    }
    break;
  case 0x0A:
    if (hi % 2 == 0 && hi < 0x08) {
      address_mode_ = AddressMode::accumulator;
    } else {
      // NOTE: 1, 3, 5, 7, D, F illegal
      address_mode_ = AddressMode::implicit;
    }
    break;
  case 0x0B:
    // NOTE: all either illegal or unofficial
    if (hi == 0x0E) { // unofficial SBC immediate
      address_mode_ = AddressMode::immediate;
    } else {
      address_mode_ = AddressMode::implicit;
    }

    break;
  case 0x0C:
    if (hi == 0x06) {
      address_mode_ = AddressMode::indirect;
    } else if (hi > 0x00 && hi % 2 == 0) {
      address_mode_ = AddressMode::absolute;
    } else if (hi == 0x0B) {
      address_mode_ = AddressMode::absoluteX;
    } else {
      // NOTE: rest illegal
      address_mode_ = AddressMode::implicit;
    }
    break;
  case 0x0D:
    if (hi % 2 == 0) {
      address_mode_ = AddressMode::absolute;
    } else {
      address_mode_ = AddressMode::absoluteX;
    }
    break;
  case 0x0E:
    if (hi == 0x09) {
      // NOTE: illegal
      address_mode_ = AddressMode::implicit;
    } else if (hi == 0x0B) {
      address_mode_ = AddressMode::absoluteY;
    } else if (hi % 2 == 0) {
      address_mode_ = AddressMode::absolute;
    } else {
      address_mode_ = AddressMode::absoluteX;
    }
    break;
  case 0x0F:
    // NOTE: all illegal
    address_mode_ = AddressMode::implicit;
  }

  size_ = aModeSizes[static_cast<int>(address_mode_)];
}

void Instruction::decodeOperation() {
  switch (opcode_) {
  case 0xA1:
  case 0xA5:
  case 0xA9:
  case 0xAD:
  case 0xB1:
  case 0xB5:
  case 0xB9:
  case 0xBD:
    operation_ = Operation::loadA;
    break;
  case 0xA2:
  case 0xA6:
  case 0xAE:
  case 0xB6:
  case 0xBE:
    operation_ = Operation::loadX;
    break;
  case 0xA0:
  case 0xA4:
  case 0xAC:
  case 0xB4:
  case 0xBC:
    operation_ = Operation::loadY;
    break;
  case 0x81:
  case 0x85:
  case 0x8D:
  case 0x91:
  case 0x95:
  case 0x99:
  case 0x9D:
    operation_ = Operation::storeA;
    break;
  case 0x86:
  case 0x8E:
  case 0x96:
    operation_ = Operation::storeX;
    break;
  case 0x84:
  case 0x8C:
  case 0x94:
    operation_ = Operation::storeY;
    break;
  case 0x61:
  case 0x65:
  case 0x69:
  case 0x6D:
  case 0x71:
  case 0x75:
  case 0x79:
  case 0x7D:
    operation_ = Operation::add;
    break;
  case 0xE1:
  case 0xE5:
  case 0xE9:
  case 0xEB: // unofficial
  case 0xED:
  case 0xF1:
  case 0xF5:
  case 0xF9:
  case 0xFD:
    operation_ = Operation::subtract;
    break;
  case 0xE6:
  case 0xEE:
  case 0xF6:
  case 0xFE:
    operation_ = Operation::increment;
    break;
  case 0xE8:
    operation_ = Operation::incrementX;
    break;
  case 0xC8:
    operation_ = Operation::incrementY;
    break;
  case 0xC6:
  case 0xCE:
  case 0xD6:
  case 0xDE:
    operation_ = Operation::decrement;
    break;
  case 0xCA:
    operation_ = Operation::decrementX;
    break;
  case 0x88:
    operation_ = Operation::decrementY;
    break;
  case 0x06:
  case 0x0E:
  case 0x16:
  case 0x1E:
    operation_ = Operation::shiftL;
    break;
  case 0x0A:
    operation_ = Operation::shiftLA;
    break;
  case 0x46:
  case 0x4E:
  case 0x56:
  case 0x5E:
    operation_ = Operation::shiftR;
    break;
  case 0x4A:
    operation_ = Operation::shiftRA;
    break;
  case 0x26:
  case 0x2E:
  case 0x36:
  case 0x3E:
    operation_ = Operation::rotateL;
    break;
  case 0x2A:
    operation_ = Operation::rotateLA;
    break;
  case 0x66:
  case 0x6E:
  case 0x76:
  case 0x7E:
    operation_ = Operation::rotateR;
    break;
  case 0x6A:
    operation_ = Operation::rotateRA;
    break;
  case 0x21:
  case 0x25:
  case 0x29:
  case 0x2D:
  case 0x31:
  case 0x35:
  case 0x39:
  case 0x3D:
    operation_ = Operation::bwAND;
    break;
  case 0x01:
  case 0x05:
  case 0x09:
  case 0x0D:
  case 0x11:
  case 0x15:
  case 0x19:
  case 0x1D:
    operation_ = Operation::bwOR;
    break;
  case 0x41:
  case 0x45:
  case 0x49:
  case 0x4D:
  case 0x51:
  case 0x55:
  case 0x59:
  case 0x5D:
    operation_ = Operation::bwXOR;
    break;
  case 0xC1:
  case 0xC5:
  case 0xC9:
  case 0xCD:
  case 0xD1:
  case 0xD5:
  case 0xD9:
  case 0xDD:
    operation_ = Operation::compare;
    break;
  case 0xE0:
  case 0xE4:
  case 0xEC:
    operation_ = Operation::compareX;
    break;
  case 0xC0:
  case 0xC4:
  case 0xCC:
    operation_ = Operation::compareY;
    break;
  case 0x24:
  case 0x2C:
    operation_ = Operation::bitTest;
    break;
  case 0x10:
    operation_ = Operation::branchPos;
    break;
  case 0x50:
    operation_ = Operation::branchVC;
    break;
  case 0x90:
    operation_ = Operation::branchCC;
    break;
  case 0xD0:
    operation_ = Operation::branchNE;
    break;
  case 0x30:
    operation_ = Operation::branchNeg;
    break;
  case 0x70:
    operation_ = Operation::branchVS;
    break;
  case 0xB0:
    operation_ = Operation::branchCS;
    break;
  case 0xF0:
    operation_ = Operation::branchEQ;
    break;
  case 0xAA:
    operation_ = Operation::transferAX;
    break;
  case 0xA8:
    operation_ = Operation::transferAY;
    break;
  case 0xBA:
    operation_ = Operation::transferSX;
    break;
  case 0x8A:
    operation_ = Operation::transferXA;
    break;
  case 0x9A:
    operation_ = Operation::transferXS;
    break;
  case 0x98:
    operation_ = Operation::transferYA;
    break;
  case 0x4C:
  case 0x6C:
    operation_ = Operation::jump;
    break;
  case 0x20:
    operation_ = Operation::jumpSR;
    break;
  case 0x60:
    operation_ = Operation::returnSR;
    break;
  case 0x40:
    operation_ = Operation::returnINT;
    break;
  case 0x48:
    operation_ = Operation::pushA;
    break;
  case 0x68:
    operation_ = Operation::pullA;
    break;
  case 0x08:
    operation_ = Operation::pushS;
    break;
  case 0x28:
    operation_ = Operation::pullS;
    break;
  case 0x00:
    operation_ = Operation::forceBreak;
    break;
  case 0x18:
    operation_ = Operation::clearC;
    break;
  case 0x58:
    operation_ = Operation::clearI;
    break;
  case 0xB8:
    operation_ = Operation::clearV;
    break;
  case 0xD8:
    operation_ = Operation::clearD;
    break;
  case 0x38:
    operation_ = Operation::setC;
    break;
  case 0x78:
    operation_ = Operation::setI;
    break;
  case 0xF8:
    operation_ = Operation::setD;
    break;
  case 0xEA:
    operation_ = Operation::nop;
    break;
  default:
    operation_ = Operation::illegal;
  }
}

ostream &operator<<(ostream &os, Instruction const &in) {
  stringstream ss;
  ss << "["
     << "0x" << hex << setfill('0') << setw(4) << +in.pc_ << "]\t"
     << opMnemonics[static_cast<int>(in.operation_)] << " ";

  // TODO(oren): currently printing addresses in big endian
  switch (in.address_mode_) {
  case AddressMode::implicit:
    break;
  case AddressMode::accumulator:
    ss << "A";
    break;
  default:
    ss << "$" << setw(4) << +in.address_ << " ("
       << aModeMnemonics[static_cast<int>(in.address_mode_)] << ")";
    break;
  }
  os << ss.str();
  return os;
}

} // namespace instr
