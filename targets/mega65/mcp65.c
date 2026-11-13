/* A work-in-progess MEGA65 (Commodore 65 clone origins) emulator
   Part of the Xemu project, please visit: https://github.com/lgblgblgb/xemu
   M65-MCP emulation: Copyright (C)2026 Tiwaz

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA */

// The M65-MCP, the MEGA65 mathematics coprocessor (not part of the stock MEGA65 core),
// Xemu's side of it. The unit itself is mcp65_unit.c: it wants to know, with every
// access, in which of its clocks the access happens, since it is asynchronous (a
// result is there some clocks after its command, and a program that reads too early
// must get what it would get on a MEGA65). That clock is told here, from the cycles
// the CPU and the DMA have run.

#include "xemu/emutools.h"
#include "mcp65.h"
#include "mcp65_unit.h"
#include "xemu/cpu65.h"
#include "dma65.h"
#include "mega65.h"

// The unit's clock is the MEGA65's 40.5MHz, whatever speed the CPU is set to
#define MCP65_CLOCK_MHZ	MEGA65_DEFAULT_FAST_CLOCK

static Uint64 clock_fp;				// the unit's clock now, in 1/65536 clocks
static Uint64 cycles_seen;			// the CPU's and the DMA's cycles it has been counted up to
static Uint64 clocks_per_cycle_fp = 0x10000;	// the unit's clocks in a CPU cycle, in 1/65536 clocks
static unsigned int overruns_told;


// The unit's clock in which an access made now happens: at the end of the opcode in
// progress (or of the DMA's cycles so far).
static inline Uint64 clock_now ( void )
{
	const Uint64 cycles = cpu65.cycles_done + (Uint64)cpu65.op_cycles + dma_cycles_done;
	if (XEMU_LIKELY(cycles > cycles_seen)) {	// (it never goes back: an access from the monitor comes between opcodes)
		clock_fp += (cycles - cycles_seen) * clocks_per_cycle_fp;
		cycles_seen = cycles;
	}
	return clock_fp >> 16;
}


// The CPU's speed has changed (it is told in MHz): from now on a CPU cycle is this many of the unit's clocks.
void mcp65_set_cpu_speed ( const double mhz )
{
	(void)clock_now();	// the cycles so far were at the speed before
	clocks_per_cycle_fp = (Uint64)(MCP65_CLOCK_MHZ / mhz * 65536.0 + 0.5);
	if (!clocks_per_cycle_fp)
		clocks_per_cycle_fp = 1;
}


Uint8 mcp65_read ( const unsigned int addr )
{
	return mcp65_unit_read(clock_now(), addr & 0xFFF);
}


void mcp65_write ( const unsigned int addr, const Uint8 data )
{
	mcp65_unit_write(clock_now(), addr & 0xFFF, data);
	const unsigned int overruns = mcp65_unit_overruns();
	if (XEMU_UNLIKELY(overruns != overruns_told)) {
		// This is not an emulation problem, but the program's: it would lose the command on a MEGA65 too.
		if (overruns <= 8 || !(overruns & 0x3FF))
			DEBUGPRINT("MCP: the queue was full, a command is dropped (MSTATUS bit 2), write of $%02X to $%X, PC=$%04X (%u so far)" NL,
				data, MCP65_BASE_ADDR + addr, cpu65.old_pc, overruns
			);
		overruns_told = overruns;
	}
}


// The unit as it is when the machine is switched on (also later, for the monitor's ~mcpinit)
void mcp65_init ( void )
{
	mcp65_unit_init();
	overruns_told = 0;
	DEBUGPRINT("MCP: M65-MCP mathematics coprocessor at $%X (MVER $03), its clock %.2fMHz" NL, MCP65_BASE_ADDR, (double)MCP65_CLOCK_MHZ);
}
