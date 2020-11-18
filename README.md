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

Easy:
- cpu::M6502 is no longer a template class, so its implementation should be separated from its interface.

Harder:
- cpu:M6502 is not cycle accurate. In fact, it doesn't account for cycle counts at all.
- Will need to decide on a granularity for stepping AND do some refactoring to accomodate.
