# timing-m65: what things take on a MEGA65, and in Xemu

A test program that measures, on a real MEGA65 and in Xemu alike, how many
cycles things take at 40.5 MHz, and a script that lays the two side by side.
It is how Xemu's MEGA65 timing was brought in line with the machine, and how
to check that it still is.

```
python3 build/tests/timing-m65/timing.py board      # a MEGA65 on its USB JTAG/serial link -> results/board.json
python3 build/tests/timing-m65/timing.py xemu       # build/bin/xmega65.native, headless    -> results/xemu.json
python3 build/tests/timing-m65/timing.py compare    # every test where the two differ by more than 0.1 cycles
python3 build/tests/timing-m65/timing.py show       # all of one result file
```

Needs [ACME](https://sourceforge.net/projects/acme-crossass/) 0.97 or
later and `m65rs` (MEGA65 Studio). `results/board.json` and
`results/board-pal.json` are a MEGA65 R6 (core 0.97 with the M65-MCP in it)
in NTSC and in PAL, 2026-11-13; `results/xemu.json` is this Xemu.

## What is measured

637 tests:

- **every 45GS02 opcode** in every addressing mode: branches taken and not,
  indexing with and without a page crossed, decimal mode, the Q
  instructions, the 32-bit indirect ones (BRK and MAP are left out)
- **the same instructions against each kind of memory**: chip RAM, ROM,
  colour RAM, the I/O area (VIC, CIA, SID, palette, floppy, SD, DMA), the
  character RAM, attic RAM, the SD and ethernet buffers, the I2C devices,
  the M65-MCP
- **the hardware divider and multiplier**: how long after an operand is
  written they say busy
- **DMA jobs**: fills and copies between those kinds of memory at two
  lengths (what a byte takes, and what a job takes besides), with steps,
  with the destination held, chained, with more options, old-style lists
- **the CIAs' clock** over 11 and 40 video frames

## How

The instruction stands R times in a block, the block runs N times, and CIA
2's two timers count meanwhile as one 32-bit counter (`timing.asm`). That is
done for two values of R: the difference of the two times is the
instruction alone, 16 x N times; the loop, the timers' start and stop and
whatever sets the registers up are in both. Where an instruction cannot
stand alone (RTS, PLP, a prefix), what stands with it is measured by itself
and taken off. The CIA counts at 984,375 Hz in PAL and 1,022,727 Hz in
NTSC, so a difference of one cycle is about a hundred ticks.

On the MEGA65 every opcode comes out as a whole number of cycles to two
decimal places, which is the check on the method.

Each measurement is kept well inside one video frame and taken five times,
the longest kept, because of the last point below.

## What the MEGA65 does (R6, core 0.97)

- **The opcodes** take what Xemu's table (from LGB's megacyc) says, with
  three exceptions: `LDA [$nn],Z` and `ADC [$nn],Z` take 10 cycles like the
  other six of their kind (the table had 13 and 14), and a taken 8-bit
  branch takes 4 cycles whether or not it crosses a page.
- **Reads have wait states** by the kind of memory; writes have none, but
  to the attic RAM:

  | A read of | cycles more |
  |---|---|
  | chip RAM, ROM | 0 |
  | the I/O area (`$D000-$DFFF`: VIC, SID, CIAs, DMA, floppy, SD, ...), the SD buffer, the I2C devices, the ethernet buffer | 1 |
  | colour RAM (at `$FF80000`, at `$1F800-$1FFFF`, in the I/O area), the character RAM, the M65-MCP | 2 |
  | the palette in the I/O area (`$D100-$D3FF`) | 3 |
  | attic RAM | 13 (13.6 for the further bytes of an LDQ); a write 3 |

  So `LDA $D020` is 6 cycles, not 5, and an `LDQ` from colour RAM 24, not
  16.
- **The divider** says busy for some cycles after an operand is written: a
  `BIT $D70F` up to three one-cycle instructions after the write still sees
  DIVBUSY for most divisors, only right after it for a divisor like
  `$FFFFFFF0` (it stops as soon as it has all its bits). The multiplier is
  never busy.
- **DMA**: 1 cycle a byte for a fill, 2 for a copy, plus the source's wait
  states; a cycle for each byte of the list; 3 more for a session's first
  entry and 2 for each further one; and a copy reads its first byte twice
  (2 cycles and the source's wait states again). A fill of one byte of chip
  RAM is 18 cycles after the write that starts it. The attic RAM is slower
  and uneven: 4.4 cycles a byte for a fill, 16.5 for a copy from it, 9.7
  for a copy to it after its first 64 bytes, 26.7 within it.
- **The CIAs' clock loses one raster line's ticks every video frame.** The
  machine hands the clock out as 63 (PAL) or 65 (NTSC) ticks for every two
  raster lines, and its frame has an odd number of lines. Measured: 31.2
  ticks a frame less than the nominal 19,656 in PAL, 32.1 less than 17,095
  in NTSC. Within a frame the clock is exact; a time taken over many frames
  comes out 0.16% (PAL) or 0.19% (NTSC) short.

## Where Xemu stands

After the changes that came with this test, 617 of the 637 tests are within
a tenth of a cycle of the MEGA65 in NTSC, and 622 of its run in PAL. The
machine's own two runs agree with each other on 618: what is left are DMA
jobs of 1,088 bytes, which the timer resolves to about a cycle, and the
attic RAM, which is uneven on the machine itself. All of them are within
0.4%.

Before them (the same Xemu a day earlier): the CIAs counted once a raster
line, at 0.17% (PAL) and 3.1% (NTSC) less than the machine's rate; reads of
anything but chip RAM were a cycle or more too fast; the divider was never
busy; and a DMA job was 3 to 5 cycles short. A real program shows what that
adds up to: the M65-MCP's torus demo, which draws with many small DMA fills,
took 18.72 ms a frame in Xemu and 19.5 ms on the MEGA65; it now takes 19.51.

Not measured, and so as Xemu had them: the other CPU speeds (1, 2 and
3.5 MHz), interrupts, BRK and MAP, the DMA's mix and swap, the hypervisor.
