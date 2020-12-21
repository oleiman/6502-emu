# 6502-emu

A simple (enough) 6502 emulator with an extremely dumb debugging interface.

## Build it

```[bash]
$ cd /path/to/6502-emu
$ ./build.sh
```

## Run the tests (Allsuite)

```[bash]
$ cd build
$ ./test/catch
```

## Run EhBASIC

```[bash]
$ cd build
$ ./EhBASIC
```

Pretty janky as a programming environment: 
- user I/O is not built out (just a thread with a getchar loop).
- As a result, inputs shows up on the terminal right away but isn't processed until the next newline.
- This results in double printing, when the EhBASIC prints the user input before evaluating its expressions.

Even still, it's possible to execute simple BASIC commands (e.g. LET a = 23, PRINT a) in a terminal buffer. Pretty neat.

## Implementation notes

This implementation is extremely a work in progress. There are some easy wins coming up as well as some not so easy wins.

TODO:

- Very close to cycle accuracy. Certain instructions require some digging.
- Cycle stepping is done via callback. While not very performant, this should provide enough flexibility to integrate this into a full-system emulation of, say, the NES. The machine itself is still funcamentally stepped by instruction.
- Add interrupt test.
- Start thinking about undocumented opcodes.
- Address calculation should live closer to execution. Currently it happens before "dispatch", which doesn't make sense.
- Address performance issues. Runs at a dozen or two MHz, but it's pretty bad, considering. Memory bus architecture is a likely culprit.
- Remember: this is not a simulator ;)

