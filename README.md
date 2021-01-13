# 6502-emu

A cycle accurate 6502 emulator written in C++.

The goal here is to provide a flexible enough library frontent that `6502-emu` could be reused in future emulations of complete 6502-based systems. I have an NES emulator in progress and hope to tackle C64 next.

## Build it (with tests)

```[bash]
$ cd /path/to/6502-emu
$ ./build_dev.sh
```

## Run the tests (Allsuite)

Passes a number of test suites pulled from various forums including 6502.org user Klaus2m5's functional test suite.

```[bash]
$ cd build
$ ./test/catch
```

## EhBASIC

NOTE: This is broken on account of some recent refactoring efforts in service of my NES project. Don't know when if ever I'll get around to fixing it up.

## Implementation notes

This implementation is complete enough to be useful, I think, and performs reasonably well, executing the "Klaus Functional" test suite in around 1.8s (~60MHz, give or take).

In service of getting past nestest, I've implemented a handful of the less PITA unofficial opcodes. WIP on that front.

TODO:

- Assemble and add interrupt test to catch suite.
- Add remaining unofficial opcodes (full list forthcoming).
- Memory mapping functionality (required for full-system emulation) is injected into the CPU at runtime and consumed via virtual function interface. I don't like the design of the mapper class very much.
- The instruction address mode decoding logic would probably be better implemented as straight lookup tables, both for performance reasons and for easier verification.
- Mantra: this is not a simulator ;)

