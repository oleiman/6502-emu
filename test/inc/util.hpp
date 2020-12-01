#pragma once

#include <sstream>
#include <string>

const std::string AllSuiteA = "rom/AllSuiteA.bin";
const std::string KlausFunctional = "rom/6502_functional_test.bin";
const std::string BruceClarkDecimal = "rom/6502_decimal_test.bin";
const std::string KlausInterrupt = "rom/6502_interrupt_test.bin";

std::stringstream report(std::string const &rom, int cycles, int instructions);
