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

#ifndef XEMU_MEGA65_MCP65_H_INCLUDED
#define XEMU_MEGA65_MCP65_H_INCLUDED

// The M65-MCP, the MEGA65 mathematics coprocessor: one 4K page of I/O at $FFDF000-$FFDFFFF.
// The address given to the read/write functions is the offset within that page.
#define MCP65_BASE_ADDR	0xFFDF000U

extern void  mcp65_init  ( void );
extern void  mcp65_set_cpu_speed ( const double mhz );
extern Uint8 mcp65_read  ( const unsigned int addr );
extern void  mcp65_write ( const unsigned int addr, const Uint8 data );

#endif
