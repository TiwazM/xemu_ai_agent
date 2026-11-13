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

#ifndef XEMU_MEGA65_MCP65_MODEL_H_INCLUDED
#define XEMU_MEGA65_MCP65_MODEL_H_INCLUDED

// The M65-MCP's reference model (mcp65-model/mcp_model.c), and the three functions
// of it that are not in its own header but the emulation needs: the model can run a
// whole program by itself (PRUN), the emulation steps it a command at a time.
// The model is compiled in a file of its own (mcp65_model.c): SoftFloat's types
// (float32_t, ...) have the names of some hosts' own, which SDL's headers bring in.

#include "mcp65-model/mcp_model.h"

// what a command did (the model's exec_cmd)
enum {
	MCP65_X_OK,		// go on
	MCP65_X_EXIT,		// in a program: PEXIT taken
	MCP65_X_END,		// in a program: PEND
	MCP65_X_BAD,		// in a program: a reserved command (outside: ignored, MSTATUS bit 3)
	MCP65_X_SKIP = 16	// + n, in a program: PSKIP taken, the next n commands are skipped
};

extern int     mcp65_model_exec    ( mcp_state *s, const mcp_cmd *c, int in_prog );
extern int     mcp65_model_cmd_len ( const mcp_state *s, uint8_t first );
extern mcp_cmd mcp65_model_cmd_of  ( const uint8_t *b, int n );

#endif
