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

#ifndef XEMU_MEGA65_MCP65_UNIT_H_INCLUDED
#define XEMU_MEGA65_MCP65_UNIT_H_INCLUDED

// The M65-MCP as a unit on its own, with no Xemu in it (mcp65.c is Xemu's side, and
// mcp65-model/unit_script.c runs the unit alone, against the hardware's simulation).
//
// "now" is the number of the unit's clock (40.5 MHz on a MEGA65) in which the bus
// access happens. It must never go back from one call to the next.
// "addr" is the offset in the unit's page, $000-$FFF.

#include <stdint.h>

extern void    mcp65_unit_init  ( void );
extern uint8_t mcp65_unit_read  ( uint64_t now, unsigned int addr );
extern void    mcp65_unit_write ( uint64_t now, unsigned int addr, uint8_t data );
// How many commands found the queue full and were dropped, since the unit's init (for a warning: on
// a MEGA65 the program would have lost them just the same, and only MSTATUS bit 2 says so)
extern unsigned int mcp65_unit_overruns ( void );

#endif
