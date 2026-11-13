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
// as a program sees it: the M65-MCP's hardware reference (src/mcp/REFERENCE.md in its
// repository) describes that, and mcp65-model/README.md what of it is emulated how.
//
// WHAT a command does is the business of the M65-MCP's own reference model, the C
// model its hardware is verified against bit for bit (mcp65-model/mcp_model.c, on
// Berkeley SoftFloat): every command here is executed by the model's exec_cmd().
//
// WHEN it does it is this file's business. The unit is asynchronous: a command goes
// into a queue of 16, is taken from it when the unit gets to it, and its result is
// there some clocks later; a program runs for as long as it runs. Meanwhile MSTATUS
// says busy, a register still reads its old value, and a command that finds the queue
// full is dropped. A program that forgets to wait, or overruns the queue, fails on a
// MEGA65: it has to fail here too. So nothing is executed before its time:
//
// - a bus access comes with the number of the clock it happens in (the unit's own
//   clock, 40.5 MHz), and first lets the unit catch up to that clock (catch_up)
// - catching up takes commands in order, one a clock, each in the first clock the
//   hardware's rules allow it (mcp.vhdl's dispatch: nothing it reads or writes still
//   has a result to come, the iterative unit is free, no sequence is running), and
//   notes when its result will be in
// - a read before that gives the value from before the command
//
// The clocks are the hardware's as measured (REFERENCE.md, "Timing"), not a simulation
// of its pipelines: mcp65-model/unit_script.c runs the same scripts as the hardware's
// test bench and its README has how close the two are.

#include <string.h>
#include "mcp65_model.h"
#include "mcp65_unit.h"

#define MID_VALUE	0x4D	// "M"
#define MVER_VALUE	0x03

// Clocks from the clock a command is dispatched to the first clock a command that
// needs its result can be (REFERENCE.md 11.2)
#define LAT_SHORT	1	// moves, min/max, abs, neg, constants, compare, class, exchange
#define LAT_FMA		5	// add, subtract, multiply, multiply-add
#define LAT_CONV	7	// to and from integers, rounding to an integer (the iterative unit)
#define LAT_DIV		20	// divide, square root, reciprocal (the iterative unit)
#define LAT_SPECIAL	5	// the iterative unit when there is nothing to iterate (a NaN, an infinity, a zero ...)
#define LAT_ILD_ZERO	6	// FILD and FXLD of 0
#define LAT_STORE	3	// a store's bytes are in the result buffer (busy until then)
// A sequence, from its dispatch to the next command's (REFERENCE.md 11.2)
#define SEQ_VMOVE	7
#define SEQ_VARITH	11	// VADD, VSUB, VMUL, VSCALE
#define SEQ_VDOT3	18
#define SEQ_VDOT4	23
#define SEQ_VCROSS	16
#define SEQ_VNORM3	66
#define SEQ_VNORM3_SPECIAL	36	// its square root and its divide with nothing to iterate (a zero vector ...)
#define SEQ_VNORM3_ITER_AT	17	// the clock of the sequence in which its square root wants the iterative unit
#define SEQ_MXV		26	// and MTXV
#define SEQ_MMUL	71
#define SEQ_VIN		7
#define SEQ_VIN_COLOUR	8
#define SEQ_VOUTC	6
#define SEQ_VPASS	2	// + its words
// A program: the clocks from a PRUN, a PEND or a taken PSKIP to the command after it
#define JUMP_CLOCKS	2

#define QN		16	// the queue
#define BIT(i)		(1U << ((i) & 15))
#define VMASK(i)	(0xFU << (4 * ((i) & 3)))	// V0-V3's F registers
#define ALL_F		0xFFFFU

static mcp_state mcp;		// the model: the unit's state as its commands left it

// What the bus holds between accesses (MRESET does not touch any of these)
static uint8_t  f_latch[4];	// bytes 1-3 of the F register whose byte 0 was read last
static uint8_t  mint_latch[4];	// the same for MINT
static uint8_t  mpcnt_latch;	// MPCNT's high byte, taken when its low byte is read
static uint8_t  mpread_latch[8];	// the program memory entry, taken when MPREAD's byte 0 is read
static uint8_t  mem_buf[3];	// bytes 0-2 of a memory form (one buffer for all of them)
static uint8_t  wide_buf;	// the first byte of a wide form (likewise)

// The queue: the commands the bus has brought, and the clock each came in
static mcp_cmd  q_cmd[QN];
static uint64_t q_at[QN];
static unsigned int q_head, q_count;
static unsigned int overruns;

// The unit's time. A clock number here is always "the first clock in which ...".
static uint64_t t_free;		// ... the next command can be dispatched (one a clock; later after a sequence or a jump)
static uint64_t rdy_f[16];	// ... a command that needs this F register can be (its result is in)
static uint64_t rdy_cond;	// the same for MCOND
static uint64_t rdy_mint;	// and MINT
static uint64_t rdy_flags;	// ... MFLAGS has the flags of everything dispatched so far
static uint64_t it_free;	// ... the iterative unit can take another operation
static uint64_t st_free;	// ... SETFLAGS can go: no store is in its first two clocks
static uint64_t busy_end;	// ... nothing dispatched so far is unfinished any more
// What a read gives until then: the value from before the command
static uint32_t old_f[16], old_mint;
static uint8_t  old_cond, old_flags;

// The program running (PRUN): the model's run_program(), a command at a time
static int      prog;
static unsigned int prog_pc, prog_start;
static uint32_t prog_total, prog_done, prog_executed, prog_limit;
static int      prog_abort;	// MPABORT asked


static inline uint64_t max64 ( const uint64_t a, const uint64_t b )
{
	return a > b ? a : b;
}


// What mcp.vhdl's dispatch needs to know of a command to say in which clock it can go,
// and when its results are in.
struct cost_st {
	uint16_t need;		// F registers that must have no result still to come
	uint16_t wr;		// F registers it writes
	uint8_t  settled;	// the registers it needs must have been in for a clock (it reads them as they stand)
	uint8_t  need_cond;	// MCOND must have no result still to come (it reads or writes it)
	uint8_t  need_mint;	// the same for MINT
	uint8_t  need_all;	// everything must be in, the stores too (SETFLAGS)
	uint8_t  wr_cond, wr_mint, wr_flags;
	uint8_t  iter;		// it takes the iterative unit: which of the core's operations it is, + 1
	uint8_t  lat;		// clocks until what it writes is in (0: it writes nothing that takes time)
	uint8_t  seq;		// a sequence: clocks to the next command's dispatch (0: not one)
	uint8_t  store;		// it puts bytes on their way to the result buffer
};


// One of the arithmetic core's operations (its numbers are the register forms' opcodes,
// $1C and $1D the memory forms FILD and FXLD): the unit that does it, and what it writes
static void core_cost ( struct cost_st *k, const int cop, const int d )
{
	switch (cop) {
		case MCP_FADD: case MCP_FSUB: case MCP_FSUBR: case MCP_FMUL:
			k->lat = LAT_FMA;
			break;
		case MCP_FDIV: case MCP_FDIVR: case MCP_FSQRT: case MCP_FRECIP:
			k->lat = LAT_DIV;
			k->iter = (uint8_t)(cop + 1);
			break;
		case MCP_FRINT: case MCP_FTRUNC: case MCP_ILD: case MCP_XLD:
		case MCP_FTOI: case MCP_FTOX: case MCP_FTOI16:
			k->lat = LAT_CONV;
			k->iter = (uint8_t)(cop + 1);
			break;
		default:
			k->lat = LAT_SHORT;
			break;
	}
	k->wr_flags = 1;
	if (cop == MCP_FCMP || cop == MCP_FCLASS)
		k->need_cond = k->wr_cond = 1;
	else if (cop == MCP_FTOI || cop == MCP_FTOX || cop == MCP_FTOI16)
		k->need_mint = k->wr_mint = 1;
	else
		k->wr = BIT(d);
}


// mcp.vhdl's dispatch, as far as time goes. (Whether a command is a reserved one is the
// model's to say when it executes it: a reserved command takes its clock and no more.)
static void cost_of ( struct cost_st *k, const mcp_cmd *c, const int in_prog )
{
	const int d = (c->regs >> 4) & 15, s = c->regs & 15, t = (c->v >> 4) & 15;
	memset(k, 0, sizeof *k);
	if (c->form == 0) {
		switch (c->op) {
			case MCP_SETCTRL:
			case MCP_PEND:
			case MCP_VNEXT:
			case MCP_VEND:
				break;
			case MCP_SETFLAGS:
				k->need_all = 1;
				break;
			case MCP_FXCHG:
				k->need = k->wr = BIT(d) | BIT(s);
				k->lat = LAT_SHORT;
				break;
			case MCP_PEXIT:
				k->need_cond = in_prog;
				break;
			case MCP_VCLIP:		// the clip codes of the settled registers, into MCOND at once
				k->need = VMASK(s);
				k->settled = 1;
				k->need_cond = k->wr_cond = 1;
				k->store = 1;
				break;
			case MCP_VPASS:		// the attribute words, a sequence of copies
				if ((mcp.vfmt >> 3) & 3) {
					k->seq = SEQ_VPASS + ((mcp.vfmt >> 3) & 3);
					k->store = 1;
				}
				break;
			case MCP_FCONST:
				k->need = BIT(d);
				core_cost(k, MCP_FCONST, d);
				break;
			default:
				if (c->op < MCP_SETCTRL) {
					k->need = BIT(d) | BIT(s);
					core_cost(k, c->op, d);
				}
				break;
		}
	} else if (c->form == 1) {
		static const uint8_t mem_core[14] = {
			MCP_FMOVE, MCP_FADD, MCP_FSUB, MCP_FSUBR, MCP_FMUL, MCP_FDIV, MCP_FDIVR, MCP_FCMP,
			MCP_FMIN, MCP_FMAX, MCP_FSQRT, MCP_FRECIP, MCP_ILD, MCP_XLD
		};
		const int m = c->op & 15;
		if (m < 14) {		// (14 is PRUN: here only in a program, where it is a reserved command)
			k->need = BIT(d);
			core_cost(k, mem_core[m], d);
		}
	} else if (c->form == 2) {
		switch (c->op) {
			case MCPW_FADD3: case MCPW_FSUB3: case MCPW_FMUL3: case MCPW_FMA: case MCPW_FMS:
				k->need = BIT(d) | BIT(s) | BIT(t);
				core_cost(k, MCP_FADD, d);
				break;
			case MCPW_VMOVE: case MCPW_VADD: case MCPW_VSUB: case MCPW_VMUL: case MCPW_VSCALE:
			case MCPW_VDOT3: case MCPW_VDOT4: case MCPW_VCROSS: case MCPW_VNORM3:
			case MCPW_MXV: case MCPW_MTXV: {
				static const uint8_t clocks[9] = {
					SEQ_VMOVE, SEQ_VARITH, SEQ_VARITH, SEQ_VARITH, SEQ_VARITH,
					SEQ_VDOT3, SEQ_VDOT4, SEQ_VCROSS, SEQ_VNORM3
				};
				k->need = VMASK(s) | VMASK(t) | VMASK(d) | BIT(t) | BIT(d);
				k->seq = c->op >= MCPW_MXV ? SEQ_MXV : clocks[c->op - MCPW_VMOVE];
				k->wr = (c->op == MCPW_VDOT3 || c->op == MCPW_VDOT4) ? BIT(d) : VMASK(d);
				k->wr_flags = c->op != MCPW_VMOVE;
				break;
			}
			case MCPW_MLOAD:
				k->need = ALL_F;
				k->settled = 1;
				break;
			case MCPW_MSTORE:	// F0-F15 written in the clock of the dispatch
				k->need = k->wr = ALL_F;
				k->settled = 1;
				break;
			case MCPW_MMUL:
				k->need = ALL_F;
				k->seq = SEQ_MMUL;
				k->wr_flags = 1;
				break;
			case MCPW_PSKIP:
				k->need_cond = in_prog;
				break;
			case MCPW_VIN:
				if (t <= 2 && mcp_vfmt_offset(mcp.vfmt, t) >= 0) {
					k->need = k->wr = VMASK(d);
					k->seq = t == 2 ? SEQ_VIN_COLOUR : SEQ_VIN;
				}
				break;
			case MCPW_VOUT:		// the converter: nothing waits for it but SETFLAGS
				if (d <= 5 && !((d == 0 || d == 5) && (t & 1))) {
					k->need = BIT(s);
					k->wr_flags = 1;
					k->store = 1;
				}
				break;
			case MCPW_VOUTC:
				if (!(t & 1)) {
					k->need = VMASK(s);
					k->seq = SEQ_VOUTC;
					k->wr_flags = 1;
					k->store = 1;
				}
				break;
			default:
				break;
		}
	}
}


static inline int is_nan ( const uint32_t v )
{
	return (v & 0x7F800000U) == 0x7F800000U && (v & 0x007FFFFFU);
}

static inline int is_zero ( const uint32_t v )
{
	return !(v & 0x7FFFFFFFU);
}

// a NaN, an infinity or a zero
static inline int is_special ( const uint32_t v )
{
	return (v & 0x7F800000U) == 0x7F800000U || is_zero(v);
}


// The iterative unit's clocks for the operation cop on a (Fd) and b (Fs, or a memory
// form's operand): it has nothing to iterate when the result follows from what the
// operands are (mcp_fpu.vhdl, S_SPECIAL)
static int iter_clocks ( const int cop, const uint32_t a, const uint32_t b )
{
	switch (cop) {
		case MCP_FDIV: case MCP_FDIVR:
			return is_special(a) || is_special(b) ? LAT_SPECIAL : LAT_DIV;
		case MCP_FRECIP:
			return is_special(b) ? LAT_SPECIAL : LAT_DIV;
		case MCP_FSQRT:			// (anything negative is an invalid operation)
			return is_special(b) || (b >> 31) ? LAT_SPECIAL : LAT_DIV;
		case MCP_FRINT: case MCP_FTRUNC:	// (from 2^23 on it is an integer already)
			return is_special(b) || ((b >> 23) & 0xFF) >= 150 ? LAT_SPECIAL : LAT_CONV;
		case MCP_FTOI: case MCP_FTOX: case MCP_FTOI16:
			return is_nan(b) || is_zero(b) ? LAT_SPECIAL : LAT_CONV;
		default:			// FILD, FXLD
			return b ? LAT_CONV : LAT_ILD_ZERO;
	}
}


// VNORM3's clocks: its square root and its divide iterate unless the length's square
// is a zero, an infinity or a NaN (then neither does)
static int vnorm3_clocks ( const uint32_t *v )
{
	const int rm = mcp.mctrl & 3;
	uint8_t fl = 0;
	uint32_t n = mcp_core(MCP_FMUL, v[0], v[0], rm).r;
	n = mcp_fma(v[1], v[1], n, rm, &fl);
	n = mcp_fma(v[2], v[2], n, rm, &fl);
	return is_special(n) ? SEQ_VNORM3_SPECIAL : SEQ_VNORM3;
}


// The first clock, from t on, in which the hardware would dispatch a command of this cost
static uint64_t clock_for ( const struct cost_st *k, uint64_t t )
{
	uint16_t need = k->need_all ? ALL_F : k->need;
	for (int r = 0; need; r++, need >>= 1)
		if (need & 1)
			t = max64(t, rdy_f[r] + k->settled);
	if (k->need_cond || k->need_all)
		t = max64(t, rdy_cond);
	if (k->need_mint || k->need_all)
		t = max64(t, rdy_mint);
	if (k->need_all)
		t = max64(t, st_free);
	if (k->iter)
		t = max64(t, it_free);
	return t;
}


// The command is dispatched in clock t: the model executes it, and what it writes is
// noted as on its way. Returns what the model said of it (MCP65_X_...).
static int dispatch ( struct cost_st *k, const mcp_cmd *c, const int in_prog, const uint64_t t )
{
	// what the clocks depend on the operands for, while those are still what the command will read
	if (k->iter)
		k->lat = (uint8_t)iter_clocks(k->iter - 1, mcp.f[(c->regs >> 4) & 15], c->form == 1 ? c->v : mcp.f[c->regs & 15]);
	else if (c->form == 2 && c->op == MCPW_VNORM3) {
		k->seq = (uint8_t)vnorm3_clocks(&mcp.f[4 * (c->regs & 3)]);
		if (it_free > t + SEQ_VNORM3_ITER_AT)	// (a divide from before it still has the unit: the sequence waits there)
			k->seq += (uint8_t)(it_free - (t + SEQ_VNORM3_ITER_AT));
	}
	// (sequences put their results in place in their last clock, MSTORE in its own)
	const uint64_t in = t + (k->seq ? k->seq - 1 : k->lat);
	uint16_t wr = k->wr;
	for (int r = 0; wr; r++, wr >>= 1)
		if (wr & 1) {
			old_f[r] = mcp.f[r];
			rdy_f[r] = in;
		}
	if (k->wr_cond) {
		old_cond = mcp.mcond;
		rdy_cond = in;
	}
	if (k->wr_mint) {
		old_mint = mcp.mint;
		rdy_mint = in;
	}
	if (k->wr_flags) {
		if (rdy_flags <= t)		// (else flags from before are still on their way: the old ones stay)
			old_flags = mcp.mflags;
		rdy_flags = max64(rdy_flags, in);
	} else if (c->form == 0 && c->op == MCP_SETFLAGS) {
		old_flags = mcp.mflags;
		rdy_flags = t;
	}
	if (k->iter)
		it_free = t + k->lat;
	t_free = t + (k->seq ? k->seq : 1);
	busy_end = max64(busy_end, in + 1);
	if (k->store) {		// (a sequence issues its last store two clocks before the next command can go)
		const uint64_t last = k->seq ? t_free - 2 : t;
		st_free = max64(st_free, last + LAT_STORE);
		busy_end = max64(busy_end, last + LAT_STORE + 1);
	}
	return mcp65_model_exec(&mcp, c, in_prog);
}


// The program ends in clock t (code: MPERR's, 0 for an end as asked)
static void prog_end ( const int code, const uint64_t t )
{
	mcp.mpcnt = (uint16_t)prog_done;
	if (code) {
		mcp.mperr = (uint8_t)code;
		mcp.sticky |= 0x20;
	}
	prog = 0;
	t_free = max64(t_free, t + 1);		// (its last command may be a sequence still running)
	busy_end = max64(busy_end, t + 1);
}


// The unit does what it does up to and with clock "now": commands in order, one a clock,
// each in the first clock it can go. What cannot go yet stays for a later call.
static void catch_up ( const uint64_t now )
{
	struct cost_st k;
	for (;;) {
		uint64_t t = t_free;
		if (prog) {
			// the model's run_program(), one turn of its loop
			if (t > now)
				return;
			if (prog_abort) {
				prog_end(MCP_ERR_ABORT, t);
				continue;
			}
			if (prog_limit && prog_executed == prog_limit) {
				prog_end(MCP_ERR_LIMIT, t);
				continue;
			}
			const mcp_cmd *c = &mcp.prog[prog_pc];
			cost_of(&k, c, 1);
			t = clock_for(&k, t);
			if (t > now)
				return;
			prog_executed++;
			const int r = dispatch(&k, c, 1, t);
			if (r == MCP65_X_BAD)
				prog_end(MCP_ERR_BADOP, t);
			else if (r == MCP65_X_EXIT)
				prog_end(0, t);
			else if (r == MCP65_X_END) {
				if (++prog_done == prog_total)
					prog_end(0, t);
				else {
					prog_pc = prog_start;
					t_free = t + JUMP_CLOCKS;
				}
			} else if (r >= MCP65_X_SKIP) {
				prog_pc += r - MCP65_X_SKIP + 1;
				if (prog_pc > 511)		// past the last entry: it ends in this command's clock
					prog_end(MCP_ERR_END, t);
				else
					t_free = t + JUMP_CLOCKS;
			} else if (++prog_pc > 511)
				prog_end(MCP_ERR_END, t);
		} else if (q_count) {
			const mcp_cmd *c = &q_cmd[q_head];
			t = max64(t, q_at[q_head] + 1);		// (in the queue the clock after the write)
			if (c->form == 1 && (c->op & 15) == MCPM_PRUN) {
				if (t > now)
					return;
				prog = 1;
				prog_pc = prog_start = c->v & 511;
				prog_total = (c->v >> 16) ? (c->v >> 16) : 65536;
				prog_done = prog_executed = 0;
				prog_limit = mcp.mplimit & 0xFFFFFF;
				prog_abort = 0;
				mcp.mperr = 0;
				t_free = t + JUMP_CLOCKS;
			} else {
				cost_of(&k, c, 0);
				t = clock_for(&k, t);
				if (t > now)
					return;
				dispatch(&k, c, 0, t);
			}
			q_head = (q_head + 1) % QN;
			q_count--;
		} else
			return;
	}
}


// A command arrives from the bus in clock "now" (a register, memory or wide form, the
// stream, a write to MCTRL or MFLAGS): into the queue, or dropped if that is full
static void enqueue ( const uint64_t now, const mcp_cmd *cmd )
{
	if (q_count == QN) {
		mcp.sticky |= 0x04;		// MSTATUS: overrun
		overruns++;
		return;
	}
	const unsigned int at = (q_head + q_count++) % QN;
	q_cmd[at] = *cmd;
	q_at[at] = now;
}


static void enqueue_form ( const uint64_t now, const int form, const int op, const int regs, const uint32_t v )
{
	const mcp_cmd cmd = { .form = (uint8_t)form, .op = (uint8_t)op, .regs = (uint8_t)regs, .v = v };
	enqueue(now, &cmd);
}


// MRESET: everything back to its start but the three memories' contents; what was
// queued or on its way is gone
static void reset ( const uint64_t now )
{
	mcp_reset(&mcp);
	q_head = q_count = 0;
	prog = prog_abort = 0;
	memset(rdy_f, 0, sizeof rdy_f);
	rdy_cond = rdy_mint = rdy_flags = it_free = st_free = busy_end = 0;
	t_free = now + 1;
}


static inline int is_busy ( const uint64_t now )
{
	return q_count || prog || now < busy_end;
}


uint8_t mcp65_unit_read ( const uint64_t now, const unsigned int addr )
{
	catch_up(now);
	if (addr & 0x800)				// $800-$FFF: the window into the result buffer
		return mcp_window_read(&mcp, addr & 0x7FF);
	const unsigned int sa = addr & 0xFF;
	switch ((addr >> 8) & 7) {
		case 0:					// the vertex page (VINDATA reads 0)
			return mcp_vreg_read(&mcp, sa);
		case 1:					// the control page
			switch (sa) {
				case 0x00:
					return (uint8_t)(
						(((now <= rdy_flags ? old_flags : mcp.mflags) & 0x1F) ? 0x80 : 0) |
						(mcp.sticky & 0x2C) | (prog ? 0x10 : 0) |
						(q_count >= 12 ? 0x02 : 0) | (is_busy(now) ? 0x01 : 0)
					);
				case 0x01: return MID_VALUE;
				case 0x02: return MVER_VALUE;
				case 0x03: return (uint8_t)q_count;
				case 0x04: return mcp.mctrl & 3;
				case 0x05: return (now <= rdy_flags ? old_flags : mcp.mflags) & 0x1F;
				case 0x06: return now <= rdy_cond ? old_cond : mcp.mcond;
				case 0x08: {
					const uint32_t v = now <= rdy_mint ? old_mint : mcp.mint;
					for (int k = 0; k < 4; k++)
						mint_latch[k] = (uint8_t)(v >> (8 * k));
					return mint_latch[0];
				}
				case 0x09: case 0x0A: case 0x0B:
					return mint_latch[sa - 0x08];
				case 0x0C: return mcp.mpptr & 0xFF;
				case 0x0D: return (mcp.mpptr >> 8) & 1;
				case 0x0E:
					mpcnt_latch = (uint8_t)(mcp.mpcnt >> 8);
					return mcp.mpcnt & 0xFF;
				case 0x0F: return mpcnt_latch;
				case 0x10: return mcp.mperr & 7;
				case 0x14: return mcp.mplimit & 0xFF;
				case 0x15: return (mcp.mplimit >> 8) & 0xFF;
				case 0x16: return (mcp.mplimit >> 16) & 0xFF;
				case 0x30:		// MPREAD: the form is read as it is, and the entry latched
					mcp_mpread(&mcp, mpread_latch);
					return mpread_latch[0] & 3;
				case 0x31: case 0x32: case 0x33: case 0x34: case 0x35: case 0x36:
					return mpread_latch[sa - 0x30];
				case 0x37:		// reads 0, and MPPTR moves on (not while a program runs)
					if (!prog)
						mcp.mpptr = (mcp.mpptr + 1) & 511;
					return 0;
				default:
					return 0;
			}
		case 4:					// $400-$43F: F0-F15
			if (sa < 0x40) {
				if (!(sa & 3)) {
					const int r = sa >> 2;
					const uint32_t v = now <= rdy_f[r] ? old_f[r] : mcp.f[r];
					for (int k = 0; k < 4; k++)
						f_latch[k] = (uint8_t)(v >> (8 * k));
					return f_latch[0];
				}
				return f_latch[sa & 3];
			}
			return 0;
		default:
			return 0;
	}
}


void mcp65_unit_write ( const uint64_t now, const unsigned int addr, const uint8_t data )
{
	catch_up(now);
	const unsigned int sa = addr & 0xFF;
	switch ((addr >> 8) & 15) {
		case 0:					// the vertex page: written at once
			if (sa < 4)
				mcp_vin_write(&mcp, data);
			else
				mcp_vreg_write(&mcp, (int)sa, data);
			break;
		case 1:					// the control page
			if (sa >= 0x80) {		// register forms: $180 + opcode
				enqueue_form(now, 0, sa & 0x7F, data, 0);
				break;
			}
			switch (sa) {
				case 0x00:		// MSTATUS: the sticky bits cleared
					mcp.sticky = 0;
					break;
				case 0x04:		// MCTRL: queued, as the command SETCTRL
					enqueue_form(now, 0, MCP_SETCTRL, data, 0);
					break;
				case 0x05:		// MFLAGS: likewise, SETFLAGS
					enqueue_form(now, 0, MCP_SETFLAGS, data, 0);
					break;
				case 0x07:
					reset(now);
					break;
				case 0x0C:
					mcp.mpptr = (uint16_t)((mcp.mpptr & 0x100) | data);
					break;
				case 0x0D:
					mcp.mpptr = (uint16_t)((mcp.mpptr & 0x0FF) | ((data & 1) << 8));
					break;
				case 0x14:
					mcp.mplimit = (mcp.mplimit & 0xFFFF00U) | data;
					break;
				case 0x15:
					mcp.mplimit = (mcp.mplimit & 0xFF00FFU) | ((uint32_t)data << 8);
					break;
				case 0x16:
					mcp.mplimit = (mcp.mplimit & 0x00FFFFU) | ((uint32_t)data << 16);
					break;
				case 0x20: case 0x21: case 0x22: case 0x23: {
					// MSTREAM: a command a byte at a time (the model's mcp_stream() would execute it itself)
					mcp.sbuf[mcp.sn++] = data;
					const int n = mcp65_model_cmd_len(&mcp, mcp.sbuf[0]);
					if (mcp.sn >= n) {
						mcp.sn = 0;
						const mcp_cmd cmd = mcp65_model_cmd_of(mcp.sbuf, n);
						enqueue(now, &cmd);
					}
					break;
				}
				case 0x24:
					mcp_stream_restart(&mcp);
					break;
				case 0x28: case 0x29: case 0x2A: case 0x2B:
					mcp_mpload(&mcp, data);
					break;
				case 0x2C:
					mcp_mpload_restart(&mcp);
					break;
				case 0x2D:		// MPABORT: the running program stops at its next command
					if (prog)
						prog_abort = 1;
					break;
				default:
					break;
			}
			break;
		case 2:					// wide forms: $200 + op * 2, the second byte issues
			if (!(addr & 1))
				wide_buf = data;
			else
				enqueue_form(now, 2, sa >> 1, wide_buf, data);
			break;
		case 4: case 5: case 6: case 7:		// memory forms: $400 + m * 64 + d * 4, the fourth byte issues
			if ((addr & 3) == 3)
				enqueue_form(now, 1, 0x80 | ((addr >> 6) & 15), ((addr >> 2) & 15) << 4,
					((uint32_t)data << 24) | ((uint32_t)mem_buf[2] << 16) | ((uint32_t)mem_buf[1] << 8) | mem_buf[0]
				);
			else
				mem_buf[addr & 3] = data;
			break;
		default:				// $300-$3FF, and the window: nothing
			break;
	}
}


unsigned int mcp65_unit_overruns ( void )
{
	return overruns;
}


void mcp65_unit_init ( void )
{
	memset(&mcp, 0, sizeof mcp);
	mcp.stage = 5;		// the whole unit, MCP-3
	memset(f_latch, 0, sizeof f_latch);
	memset(mint_latch, 0, sizeof mint_latch);
	memset(mpread_latch, 0, sizeof mpread_latch);
	memset(mem_buf, 0, sizeof mem_buf);
	mpcnt_latch = wide_buf = 0;
	overruns = 0;
	reset(0);
}
