/* The m65tr3d 3D renderer (tile-based, with its float converter and vertex unit)
   at $FFDF000 - not part of the stock MEGA65 core.
   Part of the Xemu project.  https://github.com/lgblgblgb/xemu

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

#ifndef XEMU_MEGA65_TBDR65_H_INCLUDED
#define XEMU_MEGA65_TBDR65_H_INCLUDED

#define TBDR65_BASE_ADDR	0xFFDF000U

extern void  tbdr65_init          ( void );
extern void  tbdr65_reset         ( void );
extern Uint8 tbdr65_read          ( const Uint32 offset );
extern void  tbdr65_write         ( const Uint32 offset, const Uint8 data );
extern void  tbdr65_set_cpu_speed ( const double mhz );
extern int   tbdr65_dump_frame    ( const char *path );

// The VIC-IV's view of chip RAM $40000-$5FFFF: with FBMODE bit 2 set, the frame store that is
// not the render target shows there (vic4.c asks for each character row it fetches).
extern int tbdr65_vic_on;
extern const Uint8 *tbdr65_vic_char ( const Uint32 addr );

#endif
