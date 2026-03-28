## NEC uPD78C1x Disassembler

### About

This is a disassembler for the vintage NEC uPD78C1x microprocessor (7800 series).  
**Note:** This is *not* the later 78K family.

It uses a multi-pass architecture that tracks branches and does its best to distinguish real code from data. The multi-pass design is necessary because of the `TABLE` instruction: a jump table can contain another `TABLE`, which is impossible to resolve correctly in a single pass.

The code is derived from a custom emulator, so some parts could be further optimized, but the current structure makes it very easy to add patches when an instruction behaves in a non-standard way (something that has come up with certain Intel processors the original emulator was based on).

The disassembler has been tested on as many binaries as I could find — mostly firmware from synthesizers. It works very well with KORG firmware. One Technics binary behaves strangely because the upper two bits of the address space are wired to pins on PORT-B (the chip is using a 128 KB address space in that device). Casio CZ-series binaries also look unusual because of heavy use of jumps; I have not yet investigated the exact schematics of those instruments.

If you run into any issues or have binaries that don’t disassemble correctly, please open an issue!

### Usage
* Compile
``gcc upd7810_disassem.c -o upd7810_disassem``
* Run
``./disassembler -i <binary_name>.bin``

### License
This project is released under the Apache 2.0 license with the Commons Clause.
You are free to use it privately, modify it, and commercially use any data or outputs it generates.
You may not sell or distribute this project (or any product/service whose main value comes from it) as a standalone product or lightly bundled package.
