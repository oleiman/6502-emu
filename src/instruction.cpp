#include "instruction.hpp"

#include <array>
#include <iomanip>
#include <string>
#include <sstream>

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
    "ILLEGAL",
    "LDA",
    "LDX",
    "LDY",
    "STA",
    "STX",
    "STY",
    "ADC",
    "SBC",
    "INC",
    "INX",
    "INY",
    "DEC",
    "DEX",
    "DEY",
    "ASL",
    "ASL",
    "LSR",
    "LSR",
    "ROL",
    "ROL",
    "ROR",
    "ROR",
    "AND",
    "ORA",
    "EOR",
    "CMP",
    "CPX",
    "CPY",
    "BIT",
    "BPL",
    "BVC",
    "BCC",
    "BNE",
    "BMI",
    "BVS",
    "BCS",
    "BEQ",
    "TAX",
    "TAY",
    "TSX",
    "TXA",
    "TXS",
    "TYA",
    "JMP",
    "JSR",
    "RTS",
    "RTI",
    "PHA",
    "PLA",
    "PHP",
    "PLP",
    "BRK",
    "CLC",
    "CLI",
    "CLV",
    "CLD",
    "SEC",
    "SEI",
    "SED",
    "NOP"
};
    
Instruction::Instruction(
    uint8_t opcode,
    uint16_t pc,
    function<uint16_t (AddressMode)> calc_addr)
    : _opcode(opcode)
    , _size(1)
    , _addr_mode(AddressMode::implicit)
    , _op(Operation::illegal)
    , _pc(pc)
{
    decodeAMode();
    decodeOp();
    _address = calc_addr(_addr_mode);        
}

void Instruction::decodeAMode()
{
    uint8_t lo = _opcode & 0x0F;
    uint8_t hi = (_opcode & 0xF0) >> 4;

    switch(lo) {
    case 0x00:
        if (hi == 0x00 || hi == 0x04 ||
            hi == 0x06 || hi == 0x08) {
            // NOTE: 0x80 is illegal
            _addr_mode = AddressMode::implicit;
        } else if (hi == 0x02) {
            _addr_mode = AddressMode::absolute;
            _size = 3;
        } else if (hi % 2 == 0) {
            _addr_mode = AddressMode::immediate;
            _size = 2;
        } else {
            _addr_mode = AddressMode::relative;
                
        }
        break;
    case 0x01:
        if (hi % 2 == 0) {
            _addr_mode = AddressMode::indexedIndirect;
        } else{
            _addr_mode = AddressMode::indirectIndexed;
        }
        break;
    case 0x02:
        if (hi == 0x0A) {
            _addr_mode = AddressMode::immediate;
        } else {
            // NOTE: rest are illegal
            _addr_mode = AddressMode::implicit;
        }
        break;
    case 0x03:
        // NOTE: all illegal
        _addr_mode = AddressMode::implicit;
        break;
    case 0x04:
        if (hi == 0x00 || hi == 0x01 ||
            (hi >= 0x03 && hi <= 0x07) ||
            hi == 0x0D || hi == 0x0F) {
            _addr_mode = AddressMode::implicit;
        } else if (hi % 2 == 0) {
            _addr_mode = AddressMode::zeroPage;
        } else {
            _addr_mode = AddressMode::zeroPageX;
        }
        break;
    case 0x05:
        if (hi % 2 == 0) {
            _addr_mode = AddressMode::zeroPage;
        } else {
            _addr_mode = AddressMode::zeroPageX;
        }
        break;
    case 0x06:
        if (hi == 0x09 || hi == 0x0B) {
            _addr_mode = AddressMode::zeroPageY;
        } else if (hi % 2 == 0) {
            _addr_mode = AddressMode::zeroPage;
        } else {
            _addr_mode = AddressMode::zeroPageX;
        }
        break;
    case 0x07:
        // NOTE: all illegal
        _addr_mode = AddressMode::implicit;
        break;
    case 0x08:
        _addr_mode = AddressMode::implicit;
        break;
    case 0x09:
        if (hi == 0x08) {
            //NOTE: illegal
            _addr_mode = AddressMode::implicit;
        } else if (hi % 2 == 0) {
            _addr_mode = AddressMode::immediate;
        } else {
            _addr_mode = AddressMode::absoluteY;
        }
        break;
    case 0x0A:
        if (hi % 2 == 0 && hi < 0x08) {
            _addr_mode = AddressMode::accumulator;
        } else {
            // NOTE: 1, 3, 5, 7, D, F illegal
            _addr_mode = AddressMode::implicit;
        }
        break;
    case 0x0B:
        // NOTE: all illegal
        _addr_mode = AddressMode::implicit;
        break;
    case 0x0C:
        if (hi == 0x06) {
            _addr_mode = AddressMode::indirect;
        } else if (hi > 0x00 && hi % 2 == 0) {
            _addr_mode = AddressMode::absolute;
        } else if (hi == 0x0B) {
            _addr_mode = AddressMode::absoluteX;
        } else {
            // NOTE: rest illegal
            _addr_mode = AddressMode::implicit;
        }
        break;
    case 0x0D:
        if (hi % 2 == 0) {
            _addr_mode = AddressMode::absolute;
        } else {
            _addr_mode = AddressMode::absoluteX;
        }
        break;
    case 0x0E:
        if (hi == 0x09) {
            // NOTE: illegal
            _addr_mode = AddressMode::implicit;
        } else if (hi == 0x0B) {
            _addr_mode = AddressMode::absoluteY;
        } else if (hi % 2 == 0) {
            _addr_mode = AddressMode::absolute;
        } else {
            _addr_mode = AddressMode::absoluteX;
        }
        break;
    case 0x0F:
        // NOTE: all illegal
        _addr_mode = AddressMode::implicit;
    }

    _size = aModeSizes[static_cast<int>(_addr_mode)];
}

void Instruction::decodeOp()
{
    switch(_opcode) {
    case 0xA1:
    case 0xA5:
    case 0xA9:
    case 0xAD:
    case 0xB1:
    case 0xB5:
    case 0xB9:
    case 0xBD:
        _op = Operation::loadA;
        break;
    case 0xA2:
    case 0xA6:
    case 0xAE:
    case 0xB6:
    case 0xBE:
        _op = Operation::loadX;
        break;
    case 0xA0:
    case 0xA4:
    case 0xAC:
    case 0xB4:
    case 0xBC:
        _op = Operation::loadY;
        break;
    case 0x81:
    case 0x85:
    case 0x8D:
    case 0x91:
    case 0x95:
    case 0x99:
    case 0x9D:
        _op = Operation::storeA;
        break;
    case 0x86:
    case 0x8E:
    case 0x96:
        _op = Operation::storeX;
        break;
    case 0x84:
    case 0x8C:
    case 0x94:
        _op = Operation::storeY;
        break;
    case 0x61:
    case 0x65:
    case 0x69:
    case 0x6D:
    case 0x71:
    case 0x75:
    case 0x79:
    case 0x7D:
        _op = Operation::add;
        break;
    case 0xE1:
    case 0xE5:
    case 0xE9:
    case 0xED:
    case 0xF1:
    case 0xF5:
    case 0xF9:
    case 0xFD:
        _op = Operation::subtract;
        break;
    case 0xE6:
    case 0xEE:
    case 0xF6:
    case 0xFE:
        _op = Operation::increment;
        break;
    case 0xE8:
        _op = Operation::incrementX;
        break;
    case 0xC8:
        _op = Operation::incrementY;
        break;
    case 0xC6:
    case 0xCE:
    case 0xD6:
    case 0xDE:
        _op = Operation::decrement;
        break;
    case 0xCA:
        _op = Operation::decrementX;
        break;
    case 0x88:
        _op = Operation::decrementY;
        break;
    case 0x06:
    case 0x0E:
    case 0x16:
    case 0x1E:
        _op = Operation::shiftL;
        break;
    case 0x0A:
        _op = Operation::shiftLA;
        break;
    case 0x46:
    case 0x4E:
    case 0x56:
    case 0x5E:
        _op = Operation::shiftR;
        break;
    case 0x4A:
        _op = Operation::shiftRA;
        break;
    case 0x26:
    case 0x2E:
    case 0x36:
    case 0x3E:
        _op = Operation::rotateL;
        break;
    case 0x2A:
        _op = Operation::rotateLA;
        break;
    case 0x66:
    case 0x6E:
    case 0x76:
    case 0x7E:
        _op = Operation::rotateR;
        break;
    case 0x6A:
        _op = Operation::rotateRA;
        break;
    case 0x21:
    case 0x25:
    case 0x29:
    case 0x2D:
    case 0x31:
    case 0x35:
    case 0x39:
    case 0x3D:
        _op = Operation::bwAND;
        break;
    case 0x01:
    case 0x05:
    case 0x09:
    case 0x0D:
    case 0x11:
    case 0x15:
    case 0x19:
    case 0x1D:
        _op = Operation::bwOR;
        break;
    case 0x41:
    case 0x45:
    case 0x49:
    case 0x4D:
    case 0x51:
    case 0x55:
    case 0x59:
    case 0x5D:
        _op = Operation::bwXOR;
        break;
    case 0xC1:
    case 0xC5:
    case 0xC9:
    case 0xCD:
    case 0xD1:
    case 0xD5:
    case 0xD9:
    case 0xDD:
        _op = Operation::compare;
        break;
    case 0xE0:
    case 0xE4:
    case 0xEC:
        _op = Operation::compareX;
        break;
    case 0xC0:
    case 0xC4:
    case 0xCC:
        _op = Operation::compareY;
        break;
    case 0x24:
    case 0x2C:
        _op = Operation::bitTest;
        break;
    case 0x10:
        _op = Operation::branchPos;
        break;
    case 0x50:
        _op = Operation::branchVC;
        break;
    case 0x90:
        _op = Operation::branchCC;
        break;
    case 0xD0:
        _op = Operation::branchNE;
        break;
    case 0x30:
        _op = Operation::branchNeg;
        break;
    case 0x70:
        _op = Operation::branchVS;
        break;
    case 0xB0:
        _op = Operation::branchCS;
        break;
    case 0xF0:
        _op = Operation::branchEQ;
        break;
    case 0xAA:
        _op = Operation::transferAX;
        break;
    case 0xA8:
        _op = Operation::transferAY;
        break;
    case 0xBA:
        _op = Operation::transferSX;
        break;
    case 0x8A:
        _op = Operation::transferXA;
        break;
    case 0x9A:
        _op = Operation::transferXS;
        break;
    case 0x98:
        _op = Operation::transferYA;
        break;
    case 0x4C:
    case 0x6C:
        _op = Operation::jump;
        break;
    case 0x20:
        _op = Operation::jumpSR;
        break;
    case 0x60:
        _op = Operation::returnSR;
        break;
    case 0x40:
        _op = Operation::returnINT;
        break;
    case 0x48:
        _op = Operation::pushA;
        break;
    case 0x68:
        _op = Operation::pullA;
        break;
    case 0x08:
        _op = Operation::pushS;
        break;
    case 0x28:
        _op = Operation::pullS;
        break;
    case 0x00:
        _op = Operation::forceBreak;
        break;
    case 0x18:
        _op = Operation::clearC;
        break;
    case 0x58:
        _op = Operation::clearI;
        break;
    case 0xB8:
        _op = Operation::clearV;
        break;
    case 0xD8:
        _op = Operation::clearD;
        break;
    case 0x38:
        _op = Operation::setC;
        break;
    case 0x78:
        _op = Operation::setI;
        break;
    case 0xF8:
        _op = Operation::setD;
        break;
    case 0xEA:
        _op = Operation::nop;
        break;
    default:
        _op = Operation::illegal;
    }
}

ostream& operator<<(ostream& os, Instruction const& in)
{
    stringstream ss;
    ss << "[" << "0x" <<
        hex << setfill('0') << setw(4) <<+in._pc << "]\t" <<
        opMnemonics[static_cast<int>(in._op)] << " ";

    // TODO(oren): currently printing addresses in big endian
    switch(in._addr_mode) {
    case AddressMode::implicit:
        break;
    case AddressMode::accumulator:
        ss << "A";
        break;
    case AddressMode::immediate:
        ss << "#$" << setw(2) << +static_cast<uint8_t>(in._address & 0xFF);
        break;
    case AddressMode::absolute:
        ss << "$" << setw(4) << +in._address;
        break;
    case AddressMode::absoluteX:
        ss << "$" << setw(4) << +in._address << ",X";
        break;
    case AddressMode::absoluteY:
        ss << "$" << setw(4) << +in._address << ",Y";
        break;
    case AddressMode::zeroPage:
        ss << "$" << setw(2) << +static_cast<uint8_t>(in._address & 0xFF);
        break;
    case AddressMode::zeroPageX:
        ss << "$" << setw(2) << +static_cast<uint8_t>(in._address & 0xFF) << ",X";
        break;
    case AddressMode::zeroPageY:
        ss << "$" << setw(2) << +static_cast<uint8_t>(in._address & 0xFF) << ",Y";
        break;
    case AddressMode::relative:
        // always interpreted as a signed byte
        ss << "$" << std::dec << +static_cast<int8_t>(in._address & 0xFF);
        break;
    case AddressMode::indirect:
        ss << "($" << setw(4) << +in._address << ")";
        break;
    case AddressMode::indexedIndirect:
        ss << "($" << +static_cast<uint8_t>(in._address & 0xFF) << ",X)";
        break;
    case AddressMode::indirectIndexed:
        ss << "($" << +static_cast<uint8_t>(in._address & 0xFF) << "),Y";
        break;
    default:
        break;
    }
    os << ss.str();
    return os;
}

} // namespace instr
