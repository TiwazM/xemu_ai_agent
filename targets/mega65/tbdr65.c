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

/* The renderer of TiwazM/m65tr3d (branch tbdr-vu, core YV11), as Slide - its
 * Glide-compatible library - sees it: the registers of src/tbdr/README.md,
 * the float port (FLTDATA), the indexed vertices (VTXDATA, STDATA, IDXDATA, the
 * pointer table and its fetch list), the vertex unit (VREGDATA, VOBJDATA,
 * VTXIN, VTXINL, near clipping into CLIPNEXT), the two frame stores and the
 * VIC-IV showing them (FBMODE bit 2), FBWIN.
 *
 * What a frame looks like is the renderer's own C model (tbdr-model/: a copy of
 * src/tbdr/model, the contract the VHDL is verified against byte for byte), so
 * the pictures are the hardware's. What is not the hardware's: everything the
 * converter, the hasher and the vertex unit do happens at once (their busy bits
 * never show, STATUS bit 2 only while a triangle waits for its vertices); a
 * render takes the model's estimate of the hardware's 81MHz clocks (CYCLES),
 * counted from the CPU's and the DMA's cycles, during which STATUS says busy. */

#include "xemu/emutools.h"
#include "xemu/cpu65.h"
#include "tbdr65.h"
#include "dma65.h"
#include "tbdr-model/tbdr_model.h"
#include <string.h>

#define CAPS		0x7B	// the frame stores for the VIC-IV, indexed vertices, the vertex unit, clipping, BC1, mip levels
#define ID_VALUE	0x3D
#define POOL		1024
#define SLOTS		128

int tbdr65_vic_on = 0;

static double cpu_mhz = 40.5;

// registers
static Uint8  flags, bg, cols, rows, fbmode, fltmode, oowexp, lc_lo, vn_lo;
static Uint16 count;
static Uint32 zclear, cycles, pairs, fbbase;
static Uint8  fogrgb[3];
// pointers
static Uint32 tri_ptr, cm_ptr, fb_ptr, tex_ptr, inv_ptr, pal_ptr, desc_ptr, lc_ptr, fog_ptr;
static Uint32 fbwin_last = 0xFFFFFFFFU;
// memories
static Uint8  triram[TBDR_MAXTRI * TBDR_REC64];
static Uint8  cmap[TBDR_CMAP_SIZE];
static Uint8  fb[2][65536];
static Uint8  texmem[TBDR_TEXMEM];
static Uint8  inv[32768];
static Uint8  pal[1024];		// 256 R, 256 G, 256 B, 256 alpha
static Uint8  state[512];		// M7: the state table
static Uint8  desc[64];
static Uint16 lcurve[1024];
static Uint8  fog[64];
// the float port (M8)
static Uint32 flt_idx;
static Uint8  flt_buf[TBDR_FLTTRI];
static int    flt_pos;
// the indexed vertices (M9c)
static Uint32 vx_idx;
static Uint8  vx_buf[48];
static int    vx_pos;
static Uint8  slots[SLOTS][16];
static tbdr_tex slot_tex[SLOTS];	// a slot's texture, from the descriptors when its byte 2 came
static Uint32 st_slot;
static int    st_pos;
static Uint8  ix_buf[8];
static int    ix_pos;
static Uint32 alloc;			// VTXNEW: the next new entry (1024: the pool is full)
static int    ix_full;
static Uint32 ix_done;
static Sint16 ent[65536];		// the pointer table: pointer -> pool entry, -1 = not seen since VTXNEW
static Uint16 fetch[POOL];		// the fetch list: the new pointers in order of first use
static int    fe_n, fe_rptr, fe_hi;
static Uint8  pool[POOL][TBDR_PVTX];
static int    pflags[POOL];		// the vertex unit's: bit 0 behind the near plane, bit 1 off the guard band
static Sint32 pcs[POOL][3];		// the vertex unit's camera-space x, y, w (near clipping)
static Uint8  pvalid[POOL];		// written since VTXNEW
static struct { Uint16 e[3]; Uint8 slot; Uint16 rec; } rq[TBDR_MAXTRI * 2];	// resolved triangles waiting for their entries
static int    rq_head, rq_n;
// the vertex unit (M10b)
static Uint8  vr_buf[TBDR_VREGS], vo_buf[TBDR_VOBJ], vi_buf[TBDR_VINL];
static int    vr_pos, vo_pos, vi_pos;
static tbdr_vunit cam, unit;
static Uint32 cl_next;
// the render
static int    busy, go_wait, target, shown_target;
static Uint64 busy_until;


static Uint64 now ( void )
{
	return cpu65.cycles_done + (Uint64)cpu65.op_cycles + dma_cycles_done;
}


static int is_busy ( void )
{
	if (busy && !go_wait && now() >= busy_until)
		busy = 0;
	return busy;
}


static void descriptors ( tbdr_tex *t )
{
	for (int s = 0; s < 16; s++) {
		t[s].base = desc[4 * s] | (desc[4 * s + 1] << 8);
		t[s].lw = desc[4 * s + 2] & 15;
		t[s].lh = desc[4 * s + 2] >> 4;
		t[s].fmt = desc[4 * s + 3] & 0x71;	// (BC1, the level count)
	}
}


static void render ( void )
{
	static tbdr_tex tex[16];
	static Uint8 palrgb[256 * 3];
	tbdr_regs r;
	tbdr_stats st;
	descriptors(tex);
	for (int i = 0; i < 256; i++)
		for (int k = 0; k < 3; k++)
			palrgb[3 * i + k] = pal[256 * k + i];
	memset(&r, 0, sizeof r);
	r.cols = cols;
	r.rows = rows;
	r.count = count;
	r.bg = bg;
	r.zclear = zclear;
	r.dither = flags & 1;
	r.rec64 = (flags >> 1) & 1;
	r.texmem = texmem;
	r.tex = tex;
	r.palrgb = palrgb;
	r.inv = inv;
	r.lcurve = lcurve;
	r.fogtab = fog;
	memcpy(r.fogrgb, fogrgb, 3);
	r.palalpha = pal + 768;
	r.state = state;
	target = (fbmode >> 3) & 1;
	memset(&st, 0, sizeof st);
	if (cols * rows * 64 > (int)sizeof fb[0] || tbdr_render(triram, cmap, &r, fb[target], &st)) {
		DEBUGPRINT("TBDR: the frame's registers are out of range (%d x %d characters, %d triangles)" NL, cols, rows, count);
		st.fast = 0;
	}
	cycles = (Uint32)st.fast;
	pairs = (Uint32)st.pairs;
	shown_target = target;
	go_wait = 0;
	busy = 1;
	busy_until = now() + (Uint64)((double)st.fast * cpu_mhz / 81.0 + 0.999);
}


// the resolved triangles in order, each once its three pool entries are written (the assembler)
static void assemble ( void )
{
	while (rq_n) {
		const int h = rq_head;
		if (!pvalid[rq[h].e[0]] || !pvalid[rq[h].e[1]] || !pvalid[rq[h].e[2]])
			break;
		const Uint8 *st = slots[rq[h].slot];
		tbdr_tex tex[16];
		descriptors(tex);
		tex[st[2] & 15] = slot_tex[rq[h].slot];
		const uint8_t *const p[3] = { pool[rq[h].e[0]], pool[rq[h].e[1]], pool[rq[h].e[2]] };
		const int32_t *const c[3] = { pcs[rq[h].e[0]], pcs[rq[h].e[1]], pcs[rq[h].e[2]] };
		const int f[3] = { pflags[rq[h].e[0]], pflags[rq[h].e[1]], pflags[rq[h].e[2]] };
		Uint8 rec1[TBDR_REC64];
		if (tbdr_vtx_tri_clip(&unit, st, p, c, f, triram + TBDR_REC64 * rq[h].rec, rec1, tex) == 2 && cl_next < TBDR_MAXTRI) {
			memcpy(triram + TBDR_REC64 * cl_next, rec1, TBDR_REC64);	// (a clipped quad's second record)
			cl_next++;
		}
		rq_head = (rq_head + 1) % (int)(sizeof rq / sizeof rq[0]);
		rq_n--;
	}
	if (go_wait && !rq_n)
		render();
}


// an IDXDATA triangle: slot, copy bits, v0 v1 v2 - pointers, or (copy bit k) pool entries (the hasher)
static void idx_triangle ( void )
{
	const Uint32 rec = flt_idx;
	flt_idx = (flt_idx + 1) & (TBDR_MAXTRI - 1);
	if (ix_full)
		return;			// (dropped: the batch ended)
	Uint16 e[3];
	for (int k = 0; k < 3; k++) {
		const Uint16 v = ix_buf[2 + 2 * k] | (ix_buf[3 + 2 * k] << 8);
		if (ix_buf[1] & (1 << k)) {
			e[k] = v & (POOL - 1);
		} else if (ent[v] >= 0) {
			e[k] = ent[v];
		} else if (alloc >= POOL) {
			ix_full = 1;
			ix_done = rec;
			return;
		} else {
			ent[v] = alloc;
			fetch[fe_n++] = v;
			e[k] = alloc++;
		}
	}
	if (rq_n == (int)(sizeof rq / sizeof rq[0])) {
		DEBUGPRINT("TBDR: the resolved queue overflowed, a triangle dropped" NL);
		return;
	}
	const int t = (rq_head + rq_n) % (int)(sizeof rq / sizeof rq[0]);
	memcpy(rq[t].e, e, sizeof e);
	rq[t].slot = ix_buf[0] & (SLOTS - 1);
	rq[t].rec = rec;
	rq_n++;
	assemble();
}


static void pool_written ( const Uint32 i )
{
	pvalid[i] = 1;
	assemble();
}


void tbdr65_set_cpu_speed ( const double mhz )
{
	if (mhz > 0)
		cpu_mhz = mhz;
}


void tbdr65_reset ( void )
{
	flags = 1;
	bg = 0;
	cols = 40;
	rows = 25;
	count = 0;
	zclear = 0xFFFFFFFFU;
	fbmode = 0;
	fbbase = 0;
	fltmode = 0;
	oowexp = 16;
	tri_ptr = cm_ptr = fb_ptr = tex_ptr = inv_ptr = pal_ptr = desc_ptr = lc_ptr = fog_ptr = 0;
	fbwin_last = 0xFFFFFFFFU;
	flt_idx = 0;
	flt_pos = vx_pos = st_pos = ix_pos = vr_pos = vo_pos = vi_pos = 0;
	vx_idx = st_slot = 0;
	alloc = 0;
	ix_full = 0;
	ix_done = 0;
	memset(ent, 0xFF, sizeof ent);
	fe_n = fe_rptr = fe_hi = 0;
	memset(pvalid, 0, sizeof pvalid);
	rq_head = rq_n = 0;
	cl_next = 0;
	busy = go_wait = 0;
	target = shown_target = 0;
	tbdr65_vic_on = 0;
	memset(vr_buf, 0, sizeof vr_buf);
	tbdr_vunit_unpack(vr_buf, &cam);
	unit = cam;
}


void tbdr65_init ( void )
{
	// (the memories as an FPGA starts: zero)
	memset(triram, 0, sizeof triram);
	memset(cmap, 0, sizeof cmap);
	memset(fb, 0, sizeof fb);
	memset(texmem, 0, sizeof texmem);
	memset(inv, 0, sizeof inv);
	memset(pal, 0, sizeof pal);
	memset(state, 0, sizeof state);
	memset(desc, 0, sizeof desc);
	memset(lcurve, 0, sizeof lcurve);
	memset(fog, 0, sizeof fog);
	memset(fogrgb, 0, sizeof fogrgb);
	tbdr65_reset();
	DEBUGPRINT("TBDR: the m65tr3d renderer at $%X (CAPS $%02X)" NL, TBDR65_BASE_ADDR, CAPS);
}


const Uint8 *tbdr65_vic_char ( const Uint32 addr )
{
	const int store = (addr >> 16) & 1;		// $4xxxx: store 0, $5xxxx: store 1
	if ((addr & 0xE0000U) != 0x40000U || store == ((fbmode >> 3) & 1))
		return NULL;				// (the render target's range stays chip RAM)
	return fb[store] + (addr & 0xFFFFU);
}


static int dump ( const char *path, const char *ext, const void *p, size_t n )
{
	char name[1024];
	snprintf(name, sizeof name, "%s%s", path, ext);
	FILE *f = fopen(name, "wb");
	if (!f)
		return -1;
	const int ok = fwrite(p, 1, n, f) == n;
	fclose(f);
	return ok ? 0 : -1;
}


// the last frame (its store, 64 bytes a character) to path, and what it was made of beside it:
// path.tri (the records), .pal (R, G, B, alpha), .state, .desc, .fog (64 + R, G, B), .tex
int tbdr65_dump_frame ( const char *path )
{
	Uint8 fogall[67];
	memcpy(fogall, fog, 64);
	memcpy(fogall + 64, fogrgb, 3);
	return dump(path, "", fb[shown_target], (size_t)cols * rows * 64) |
		dump(path, ".tri", triram, (size_t)TBDR_REC64 * count) |
		dump(path, ".pal", pal, sizeof pal) |
		dump(path, ".state", state, sizeof state) |
		dump(path, ".desc", desc, sizeof desc) |
		dump(path, ".fog", fogall, sizeof fogall) |
		dump(path, ".tex", texmem, sizeof texmem);
}


Uint8 tbdr65_read ( const Uint32 offset )
{
	if (offset >= 0x800) {			// FBWIN: a read at a new address is the next byte of the frame
		if (offset != fbwin_last) {
			fbwin_last = offset;
			return fb[(fbmode >> 3) & 1][fb_ptr++ & 0xFFFF];
		}
		return fb[(fbmode >> 3) & 1][(fb_ptr - 1) & 0xFFFF];
	}
	fbwin_last = 0xFFFFFFFFU;
	switch (offset) {
		case 0x00: return (rq_n || go_wait ? 4 : 0) | (is_busy() ? 1 : 0);
		case 0x01: return ID_VALUE;
		case 0x02: return flags;
		case 0x03: return bg;
		case 0x04: return cols;
		case 0x05: return rows;
		case 0x06: return count & 0xFF;
		case 0x07: return count >> 8;
		case 0x08: case 0x09: case 0x0A: case 0x0B: return zclear >> (8 * (offset - 0x08));
		case 0x0C: case 0x0D: case 0x0E: case 0x0F: return cycles >> (8 * (offset - 0x0C));
		case 0x10: return tri_ptr & 0xFF;
		case 0x11: return (tri_ptr >> 8) & 0xFF;
		case 0x12: { const Uint8 d = triram[tri_ptr & 0xFFFF]; tri_ptr = (tri_ptr + 1) & 0xFFFF; return d; }
		case 0x14: return cm_ptr & 0xFF;
		case 0x15: return (cm_ptr >> 8) & 0x3F;
		case 0x16: { const Uint8 d = cmap[cm_ptr]; cm_ptr = (cm_ptr + 1) & 0x3FFF; return d; }
		case 0x18: return fb_ptr & 0xFF;
		case 0x19: return (fb_ptr >> 8) & 0xFF;
		case 0x1A: return fb[(fbmode >> 3) & 1][fb_ptr++ & 0xFFFF];
		case 0x1C: case 0x1D: case 0x1E: case 0x1F: return pairs >> (8 * (offset - 0x1C));
		case 0x20: return fbmode;
		case 0x24: case 0x25: case 0x26: return fbbase >> (8 * (offset - 0x24));
		case 0x27: return (fbbase >> 24) & 7;
		case 0x28: return tex_ptr & 0xFF;
		case 0x29: return (tex_ptr >> 8) & 0xFF;
		case 0x2B: return (tex_ptr >> 16) & 3;
		case 0x2C: return inv_ptr & 0xFF;
		case 0x2D: return (inv_ptr >> 8) & 0x7F;
		case 0x30: return pal_ptr & 0xFF;
		case 0x31: return (pal_ptr >> 8) & 7;
		case 0x34: return desc_ptr;
		case 0x38: return lc_ptr & 0xFF;
		case 0x39: return (lc_ptr >> 8) & 7;
		case 0x44: return flt_idx & 0xFF;
		case 0x45: return flt_idx >> 8;
		case 0x47: return fltmode;
		case 0x48: return oowexp;
		case 0x49: return CAPS;
		case 0x4A: return vx_idx & 0xFF;
		case 0x4B: return vx_idx >> 8;
		case 0x4D: return st_slot;
		case 0x50: return alloc & 0xFF;
		case 0x51: return (alloc >> 8) & 7;
		case 0x52: {				// FETCHDATA: the fetch list, two bytes an entry
			const Uint16 v = fe_rptr < fe_n ? fetch[fe_rptr] : 0;
			const Uint8 d = fe_hi ? v >> 8 : v & 0xFF;
			if (fe_hi)
				fe_rptr++;
			fe_hi ^= 1;
			return d;
		}
		case 0x53: return ix_full ? 2 : 0;	// IDXSTAT (the pointer table is cleared at once)
		case 0x54: return ix_done & 0xFF;
		case 0x55: return ix_done >> 8;
		case 0x5B: return cl_next & 0xFF;
		case 0x5C: return (cl_next >> 8) & 7;
		default: return 0;
	}
}


void tbdr65_write ( const Uint32 offset, const Uint8 data )
{
	if (offset >= 0x800)
		return;
	fbwin_last = 0xFFFFFFFFU;
	switch (offset) {
		case 0x00:
			if ((data & 1) && !is_busy()) {		// GO: once the converter's triangles are in
				busy = 1;
				if (rq_n)
					go_wait = 1;
				else
					render();
			}
			return;
		case 0x02: flags = data; return;
		case 0x03: bg = data; return;
		case 0x04: cols = data; return;
		case 0x05: rows = data; return;
		case 0x06: count = (count & 0xFF00) | data; return;
		case 0x07: count = (count & 0x00FF) | (data << 8); return;
		case 0x08: case 0x09: case 0x0A: case 0x0B: {
			const int s = 8 * (offset - 0x08);
			zclear = (zclear & ~(0xFFU << s)) | ((Uint32)data << s);
			return;
		}
		case 0x10: tri_ptr = (tri_ptr & 0xFF00) | data; return;
		case 0x11: tri_ptr = (tri_ptr & 0x00FF) | (data << 8); return;
		case 0x12: triram[tri_ptr] = data; tri_ptr = (tri_ptr + 1) & 0xFFFF; return;
		case 0x14: cm_ptr = (cm_ptr & 0x3F00) | data; return;
		case 0x15: cm_ptr = (cm_ptr & 0x00FF) | ((data & 0x3F) << 8); return;
		case 0x16: cmap[cm_ptr] = data; cm_ptr = (cm_ptr + 1) & 0x3FFF; return;
		case 0x18: fb_ptr = (fb_ptr & 0xFF00) | data; return;
		case 0x19: fb_ptr = (fb_ptr & 0x00FF) | (data << 8); return;
		case 0x20: fbmode = data; tbdr65_vic_on = (CAPS & 1) && (data & 4); return;
		case 0x24: case 0x25: case 0x26: case 0x27: {
			const int s = 8 * (offset - 0x24);
			fbbase = (fbbase & ~(0xFFU << s)) | ((Uint32)data << s);
			fbbase &= 0x7FFFFFFU;
			return;
		}
		case 0x28: tex_ptr = (tex_ptr & 0x3FF00) | data; return;
		case 0x29: tex_ptr = (tex_ptr & 0x300FF) | (data << 8); return;
		case 0x2B: tex_ptr = (tex_ptr & 0x0FFFF) | ((data & 3) << 16); return;
		case 0x2C: inv_ptr = (inv_ptr & 0x7F00) | data; return;
		case 0x2D: inv_ptr = (inv_ptr & 0x00FF) | ((data & 0x7F) << 8); return;
		case 0x30: pal_ptr = (pal_ptr & 0x700) | data; return;
		case 0x31: pal_ptr = (pal_ptr & 0x0FF) | ((data & 7) << 8); return;
		case 0x34: desc_ptr = data & 63; return;
		case 0x38: lc_ptr = (lc_ptr & 0x700) | data; return;
		case 0x39: lc_ptr = (lc_ptr & 0x0FF) | ((data & 7) << 8); return;
		case 0x3C: fog_ptr = data & 63; return;
		case 0x3E: case 0x3F: case 0x40: fogrgb[offset - 0x3E] = data; return;
		case 0x44: flt_idx = (flt_idx & 0x300) | data; flt_pos = 0; ix_pos = 0; return;
		case 0x45: flt_idx = (flt_idx & 0x0FF) | ((data & 3) << 8); flt_pos = 0; ix_pos = 0; return;
		case 0x47: fltmode = data & 7; return;
		case 0x48: oowexp = data & 31; return;
		case 0x4A: vx_idx = (vx_idx & 0x300) | data; vx_pos = 0; vi_pos = 0; return;
		case 0x4B: vx_idx = (vx_idx & 0x0FF) | ((data & 3) << 8); vx_pos = 0; vi_pos = 0; return;
		case 0x4D: st_slot = data & (SLOTS - 1); st_pos = 0; return;
		case 0x50: vn_lo = data; return;
		case 0x51:					// VTXNEW: a new batch
			alloc = ((data & 7) << 8) | vn_lo;
			ix_full = 0;
			fe_n = fe_rptr = fe_hi = 0;
			memset(ent, 0xFF, sizeof ent);
			memset(pvalid, 0, sizeof pvalid);
			return;
		case 0x56: vr_pos = 0; vo_pos = 0; return;	// VREGIDX
		case 0x5B: cl_next = (cl_next & 0x700) | data; return;
		case 0x5C: cl_next = (cl_next & 0x0FF) | ((data & 7) << 8); return;
	}
	// the data ports: ignored while the renderer is busy
	if (is_busy())
		return;
	switch (offset) {
		case 0x2A: texmem[tex_ptr] = data; tex_ptr = (tex_ptr + 1) & (TBDR_TEXMEM - 1); return;
		case 0x2E: inv[inv_ptr] = data; inv_ptr = (inv_ptr + 1) & 0x7FFF; return;
		case 0x32:
			if (pal_ptr < 1024)
				pal[pal_ptr] = data;
			else if (pal_ptr < 1536)
				state[pal_ptr - 1024] = data;
			pal_ptr = (pal_ptr + 1) & 0x7FF;
			return;
		case 0x36: desc[desc_ptr] = data; desc_ptr = (desc_ptr + 1) & 63; return;
		case 0x3A:					// LCDATA: an entry with its high byte
			if (lc_ptr & 1)
				lcurve[lc_ptr >> 1] = (lc_lo | (data << 8)) & 0x7FF;
			else
				lc_lo = data;
			lc_ptr = (lc_ptr + 1) & 0x7FF;
			return;
		case 0x3D: fog[fog_ptr] = data; fog_ptr = (fog_ptr + 1) & 63; return;
		case 0x46:					// FLTDATA: 16 state bytes, three GrVertex
			flt_buf[flt_pos++] = data;
			if (flt_pos == TBDR_FLTTRI) {
				tbdr_tex tex[16];
				descriptors(tex);
				tbdr_float_tri(flt_buf, triram + TBDR_REC64 * flt_idx, tex, oowexp, fltmode);
				flt_idx = (flt_idx + 1) & (TBDR_MAXTRI - 1);
				flt_pos = 0;
			}
			return;
		case 0x4C:					// VTXDATA: a GrVertex into the pool
			vx_buf[vx_pos++] = data;
			if (vx_pos == 48) {
				tbdr_float_vtx(vx_buf, pool[vx_idx], oowexp, fltmode);
				pflags[vx_idx] = 0;
				memset(pcs[vx_idx], 0, sizeof pcs[vx_idx]);
				vx_pos = 0;
				const Uint32 i = vx_idx;
				vx_idx = (vx_idx + 1) & (POOL - 1);
				pool_written(i);
			}
			return;
		case 0x4E:					// STDATA: 16 bytes a slot
			slots[st_slot][st_pos] = data;
			if (st_pos == 2) {
				tbdr_tex tex[16];
				descriptors(tex);
				slot_tex[st_slot] = tex[data & 15];
			}
			if (++st_pos == 16) {
				st_pos = 0;
				st_slot = (st_slot + 1) & (SLOTS - 1);
			}
			return;
		case 0x4F:					// IDXDATA: 8 bytes a triangle
			ix_buf[ix_pos++] = data;
			if (ix_pos == 8) {
				ix_pos = 0;
				idx_triangle();
			}
			return;
		case 0x57:					// VREGDATA: the camera block
			vr_buf[vr_pos++] = data;
			if (vr_pos == TBDR_VREGS) {
				vr_pos = 0;
				tbdr_vunit_unpack(vr_buf, &cam);
				unit = cam;			// (an identity object, no light)
			}
			return;
		case 0x59: {					// VOBJDATA: an object block, camera x object
			vo_buf[vo_pos++] = data;
			if (vo_pos == TBDR_VOBJ) {
				tbdr_vobj o;
				vo_pos = 0;
				tbdr_vobj_unpack(vo_buf, &o);
				tbdr_vunit_compose(&cam, &o, &unit);
			}
			return;
		}
		case 0x58:					// VTXIN: a model-space vertex
		case 0x5A: {					// VTXINL: a lit one
			const int n = offset == 0x58 ? TBDR_VIN : TBDR_VINL;
			vi_buf[vi_pos++] = data;
			if (vi_pos >= n) {
				vi_pos = 0;
				pflags[vx_idx] = offset == 0x58 ?
					tbdr_vtx_xform_cs(&unit, vi_buf, pool[vx_idx], pcs[vx_idx]) :
					tbdr_vtx_xform_lit_cs(&unit, vi_buf, pool[vx_idx], pcs[vx_idx]);
				const Uint32 i = vx_idx;
				vx_idx = (vx_idx + 1) & (POOL - 1);
				pool_written(i);
			}
			return;
		}
	}
}
