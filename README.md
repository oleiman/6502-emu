# 6502-emu

A cycle accurate 6502 emulator written in C++. Includes all official and unofficial opcodes. Passes nestest with correct cycle count, though it's not included in the tests here since the cartridge and memory mapping machinery both live in a WIP not-yet-public repo.

The goal here is to provide a flexible enough library frontent that `6502-emu` could be reused in future emulations of complete 6502-based systems. I have an NES emulator in progress and hope to tackle C64 next.

## Build it (with tests)

```[bash]
$ cd /path/to/6502-emu
$ ./build_dev.sh
```

## Run the tests

Passes a number of test suites pulled from various forums including 6502.org user Klaus2m5's functional test suite.

```[bash]
$ cd build
$ ./test/catch
```

## EhBASIC

NOTE: This is broken on account of some recent refactoring efforts in service of my NES project. Don't know when if ever I'll get around to fixing it up.

## Implementation notes

This implementation is complete enough to be useful, I think, and performs reasonably well, executing the "Klaus Functional" test suite in ~0.6s (~160MHz, give or take).

TODO:

- Assemble and add interrupt test to catch suite.
- Memory mapping functionality (required for full-system emulation) is injected into the CPU at runtime and consumed via virtual function interface. I don't like the design of the mapper class very much.
- The instruction address mode decoding logic would probably be better implemented as straight lookup tables, both for performance reasons and for easier verification.

