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

// The M65-MCP's reference model as it is in its own repository (mcp65-model/ holds
// the copy, VERSIONS there says of what), and the way in for mcp65.c to the three
// of its functions that are static there. See mcp65_model.h.

#include "mcp65-model/mcp_model.c"
#include "mcp65_model.h"

_Static_assert(MCP65_X_OK == X_OK && MCP65_X_EXIT == X_EXIT && MCP65_X_END == X_END
	&& MCP65_X_BAD == X_BAD && MCP65_X_SKIP == X_SKIP, "the model's result codes have changed");

int mcp65_model_exec ( mcp_state *s, const mcp_cmd *c, int in_prog )
{
	return exec_cmd(s, c, in_prog);
}

int mcp65_model_cmd_len ( const mcp_state *s, uint8_t first )
{
	return cmd_len(s, first);
}

mcp_cmd mcp65_model_cmd_of ( const uint8_t *b, int n )
{
	return cmd_of(b, n);
}
