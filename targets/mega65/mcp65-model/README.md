# The M65-MCP in Xemu

The M65-MCP is a mathematics coprocessor for the MEGA65 (FP32 registers,
vectors, a matrix bank, programs in its own memory, a vertex path), one
page of I/O at `$FFDF000-$FFDFFFF`. It is not part of the stock MEGA65
core. This branch of Xemu emulates it, so that software written for it can
be developed and tested without the board. What a program sees of the unit
is described in its hardware reference, `src/mcp/REFERENCE.md` in the
M65-MCP's repository (TiwazM/m65-mcp).

It is on by default; `-nomcp` leaves `$FFDF000` undecoded, as on a stock
MEGA65 (MID at `$FFDF101` then does not read `$4D`).

## How it is emulated

| | |
|---|---|
| `../mcp65.c` | Xemu's side: the unit in the memory map, and the unit's clock told from the cycles the CPU and the DMA have run |
| `../mcp65_unit.c` | the unit: its bus, its queue, and when each command runs |
| `../mcp65_model.c` | the M65-MCP's own reference model, as it is (`mcp_model.c`, `mcp_model.h` here) |
| `../mcp65_softfloat.c` | the part of Berkeley SoftFloat 3e the model is built on (the files here) |

**What a command does** is the reference model's business: the C model the
hardware is verified against, bit for bit. Every command is executed by
the model's own code, so results, flags, MCOND, MINT and the result buffer
are the hardware's by construction. Nothing uses the host's floating
point.

**When it does it** is `mcp65_unit.c`'s. The unit is asynchronous, and a
program that ignores that fails on a MEGA65: it has to fail here too. So:

- a command goes into a queue of 16 and is taken from it in the clock the
  hardware would take it: in order, one a clock, once nothing it reads or
  writes has a result still to come, the divider is free and no sequence
  is running
- its result is in after the hardware's clocks (an add 5, a divide 20, or
  5 when an operand is a zero, an infinity or a NaN; a matrix product 71)
- until then MSTATUS says busy and a read gives the value from before
- a program (PRUN) runs a command at a time as the clock goes on: MPABORT
  stops it, a read meanwhile sees it half done
- a command that finds the queue full is dropped and sets MSTATUS bit 2.
  Xemu also says so in its log, with the PC

The unit's clock is the MEGA65's 40.5 MHz whatever speed the CPU is set
to, counted from the CPU's and the DMA's cycles.

## How close it is

`vs_hardware.py` runs the unit alone, without Xemu, beside the hardware's
VHDL in simulation, on the same bus scripts:

- 181 programs (every command alone and four in a row, and mixes that wait
  for each other, for the divider, for MCOND, for stores): the same clocks
  a pass as the hardware, all 181
- the clock in which MSTATUS first says idle, after each of 85 kinds of
  command, probed at 24 delays: the hardware's, all of them
- random scripts of 3,000 commands (the M65-MCP's own test generator):
  every read identical, and every wait ending in the hardware's clock (46
  scripts, about 34,000 waits)

In Xemu itself the M65-MCP's own tests pass as they do on a MEGA65 R6: its
board test (89,766 reads over three scripts, identical to the model's),
the six SDK examples, the vertex program test and both display demos
(pixel for pixel). The SDK's `async` example, which reads a register too
early and overruns the queue through a DMA job on purpose, prints the same
on the board and here, down to how many of 64 divides survive (29).

What is not the hardware's:

- **The CPU's and the DMA's cycle counts are Xemu's**, which this branch
  has brought in line with a MEGA65 R6 (`build/tests/timing-m65`: every
  opcode, the wait states of each kind of memory, DMA jobs, the CIAs'
  clock). With that, times measured in Xemu are the board's to the last
  digit it prints: the Mandelbrot demo 1.78 s on both, the standard vertex
  program 89.7 clocks a vertex on both, the torus demo 19.51 ms a frame
  here and 19.5 there. The other CPU speeds are as Xemu had them.
- **An access is placed at the end of its instruction**, a few cycles
  late for the first byte of an `LDQ`.
- **A value on its way is not shown piece by piece.** The hardware's
  pipelines are not simulated: a register reads old until its result is
  in, then new; a sequence's results (a vector or matrix command) appear
  together at its end, as in the hardware.
- A machine reset does not reset the unit, as on a MEGA65 (the unit has
  no reset line: MRESET does it, and nothing but loading the core again
  clears its three memories). The monitor's `~mcpinit` does the latter.

## Tests

```
python3 targets/mega65/mcp65-model/vs_hardware.py /path/to/m65-mcp          # the unit against the model
python3 targets/mega65/mcp65-model/vs_hardware.py /path/to/m65-mcp --sim    # and against the VHDL (needs nvc)
```

and, in the M65-MCP's repository, with this Xemu built (`make` in
`targets/mega65`; `$XEMU` if it is not in `~/Documents/GitHub/xemu-mcp`):

```
python3 -I src/mcp/sdk/run.py --xemu                 # the SDK examples
src/mcp/demo/build.sh && src/mcp/demo/hwtest.sh xemu # the board test
python3 -I src/mcp/demo/geo/geo.py --xemu            # the standard vertex program
python3 -I src/mcp/demo/show/show.py mandel --xemu   # the display demos
python3 -I src/mcp/demo/show/show.py scene --xemu
```

Two additions to the UART monitor serve them: `~load ADDR PATH` (a host
file into memory at once; the monitor's own way takes five minutes for
240 KB) and `~mcpinit` (above).

## Keeping it in step

`mcp_model.c` and `mcp_model.h` are copies; the M65-MCP's repository has
the originals. When the model changes there:

```
targets/mega65/mcp65-model/sync.sh /path/to/m65-mcp
```

copies them and the SoftFloat sources again and notes the commits in
`VERSIONS`. If the timing of the hardware changes, the constants at the top
of `mcp65_unit.c` are where it is told, and `vs_hardware.py --sim` says
whether it is right.

## Licences

Xemu and the emulation's own files are GPL v2 or later. The SoftFloat files
(`f32_*.c`, `f64_*.c`, `i32_to_f32.c`, `s_*.c`, `softfloat*.c`,
`softfloat*.h`, `internals.h`, `primitives.h`, `primitiveTypes.h`,
`opts-GCC.h`, `specialize.h`) are John R. Hauser's Berkeley SoftFloat 3e,
under its BSD licence: `SoftFloat-COPYING.txt`. `platform.h` is this
directory's own.
