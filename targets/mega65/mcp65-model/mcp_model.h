/* mcp_model: the reference for the M65-MCP: MCP-1, the scalar FPU
 * (src/mcp/MCP1.md), and MCP-2, programs, multiply-add, vectors and
 * matrices (src/mcp/MCP2.md). The arithmetic is Berkeley SoftFloat 3e built
 * with the ARM-VFPv2-defaultNaN specialisation, tininess after rounding
 * (src/mcp/sim/fetch.sh); what SoftFloat has no function for (min/max,
 * fixed point, int16, constants, classes) is written here.
 *
 * Two levels:
 *   mcp_core   one operation of the arithmetic core: what src/vhdl/mcp_fpu.vhdl
 *              computes, for its bench (mcp_fma: the fused multiply-add)
 *   mcp_exec   one command against the whole unit's state (registers, the
 *              matrix bank, MINT, MCOND, MFLAGS, MCTRL, the status bits, the
 *              program memory); mcp_stream and mcp_mpload take the stream
 *              and MPLOAD ports a byte at a time
 * Timing (the queue, overrun, MPABORT) is not modelled: the model runs every
 * command it is given, in order.
 *
 * The state's stage says which unit it models (MCP2.md, "Build order"):
 *   0  MCP-1: $18-$7F reserved, stream bytes $90-$FF two-byte reserved
 *   1  + programs: PEXIT ($18), PEND ($19), PRUN (memory form 14), MPLOAD,
 *        MPLIMIT, MPERR, MSTATUS bit 5; the stream's $90 prefix (4 bytes)
 *        known, the wide forms still reserved
 *   2  + the wide forms FADD3, FSUB3, FMUL3, FMA, FMS
 *   3  + vectors and matrices (all of MCP-2)
 *   4  + MCP-3 stage 1 (src/mcp/MCP3.md): the vertex array, the result buffer
 *        and their registers ($000-$0FF, the window $800-$FFF), VIN, VNEXT,
 *        VOUT, VOUTC, VPASS, VEND; MVER $03
 *   5  + MCP-3 stage 2: VCLIP (register form $1C), PSKIP (wide form $28) */
#ifndef MCP_MODEL_H
#define MCP_MODEL_H

#include <stdint.h>

/* register-form opcodes (MCP1.md "Register forms", MCP2.md) */
enum {
    MCP_FMOVE = 0x00, MCP_FADD, MCP_FSUB, MCP_FSUBR, MCP_FMUL, MCP_FDIV, MCP_FDIVR,
    MCP_FCMP, MCP_FMIN, MCP_FMAX, MCP_FSQRT, MCP_FABS, MCP_FNEG, MCP_FRINT,
    MCP_FTRUNC, MCP_FCLASS, MCP_FTOI, MCP_FTOX, MCP_FTOI16, MCP_FRECIP,
    MCP_FXCHG, MCP_FCONST, MCP_SETCTRL, MCP_SETFLAGS,
    MCP_NREGOPS,                            /* MCP-1: $18-$7F reserved */
    MCP_PEXIT = 0x18, MCP_PEND = 0x19,      /* MCP-2 */
    MCP_NREGOPS2,                           /* MCP-2: $1A-$7F reserved */
    MCP_VNEXT = 0x1A, MCP_VPASS = 0x1B,     /* MCP-3 */
    MCP_VCLIP = 0x1C,                       /* (MCP-3 stage 2; $1D is reserved: PSKIP is a wide form) */
    MCP_VEND = 0x1E
};
/* core-only operations: the memory forms FILD and FXLD */
#define MCP_ILD 0x1C
#define MCP_XLD 0x1D
/* memory form 14 (MCP-2) */
#define MCPM_PRUN 14

/* wide forms (MCP2.md) */
enum {
    MCPW_FADD3 = 0x00, MCPW_FSUB3 = 0x01, MCPW_FMUL3 = 0x02, MCPW_FMA = 0x03, MCPW_FMS = 0x04,
    MCPW_VMOVE = 0x10, MCPW_VADD, MCPW_VSUB, MCPW_VMUL, MCPW_VSCALE, MCPW_VDOT3, MCPW_VDOT4,
    MCPW_VCROSS, MCPW_VNORM3,
    MCPW_MLOAD = 0x20, MCPW_MSTORE, MCPW_MXV, MCPW_MTXV, MCPW_MMUL,
    MCPW_VIN = 0x25, MCPW_VOUT, MCPW_VOUTC,                 /* MCP-3 */
    MCPW_PSKIP = 0x28                                       /* (stage 2) */
};

/* VOUT's formats (MCP3.md): FP32, its top half, s16 x 16 (12.4), u16 x 2^16,
 * s16, s32 x 2^16 (16.16); VOUTC's: u8 x 2^8 */
enum { MCPV_F32, MCPV_F32HI, MCPV_S12_4, MCPV_U16, MCPV_S16, MCPV_S16_16, MCPV_NFMT };
#define MCPV_U8 7

/* MFLAGS bits (MCP1.md) */
#define MCP_FL_INVALID   0x01
#define MCP_FL_DIVZERO   0x02
#define MCP_FL_OVERFLOW  0x04
#define MCP_FL_UNDERFLOW 0x08
#define MCP_FL_INEXACT   0x10

/* MCTRL rounding modes (x87 order) */
#define MCP_RM_NEAR  0
#define MCP_RM_DOWN  1
#define MCP_RM_UP    2
#define MCP_RM_ZERO  3

#define MCP_DEFAULT_NAN 0x7FC00000u

/* MPERR (MCP2.md) */
#define MCP_ERR_END     1                   /* ran past entry 511 */
#define MCP_ERR_LIMIT   2                   /* MPLIMIT */
#define MCP_ERR_ABORT   3                   /* MPABORT (not modelled: timing) */
#define MCP_ERR_BADOP   4                   /* a reserved command or a PRUN in a program */

/* One operation of the arithmetic core. a = Fd, b = Fs or the memory
 * operand (for FCONST: the constant's number). Results: r goes to Fd when
 * wr_f; mint to MINT when wr_mint; cond to MCOND when wr_cond; flags are
 * the operation's MFLAGS bits. */
typedef struct {
    uint32_t r, mint;
    uint8_t  cond, flags;
    uint8_t  wr_f, wr_mint, wr_cond;
} mcp_res;
mcp_res mcp_core(int op, uint32_t a, uint32_t b, int rmode);
/* a x b + c rounded once (IEEE fusedMultiplyAdd); flags as MFLAGS bits */
uint32_t mcp_fma(uint32_t a, uint32_t b, uint32_t c, int rmode, uint8_t *flags);

/* The constant table (FCONST) */
extern const uint32_t mcp_const[16];

/* a command as the queue and the program memory hold it */
typedef struct {
    uint8_t  form;                  /* 0 register, 1 memory, 2 wide, 3 reserved (two bytes) */
    uint8_t  op, regs;              /* wide: regs = (d << 4) | s */
    uint32_t v;                     /* memory: the operand; wide: t << 4 in bits 0-7 */
} mcp_cmd;

/* The unit's state as the CPU sees it once the queue is empty */
typedef struct {
    int      stage;                 /* 0-3, above (mcp_reset keeps it) */
    uint32_t f[16];                 /* F0-F15, FP32 */
    uint32_t m[16];                 /* M0-M15, the matrix by rows (stage 3) */
    uint32_t mint;
    uint8_t  mcond, mflags, mctrl;
    uint8_t  sticky;                /* MSTATUS bits 2 (overrun), 3 (bad opcode), 5 (program aborted) */
    uint8_t  sbuf[6];               /* the stream: the command being collected */
    int      sn;
    /* programs (stage 1 on) */
    mcp_cmd  prog[512];
    uint16_t mpptr, mpcnt;
    uint8_t  mperr;
    uint32_t mplimit;               /* 24 bits; 0: no limit */
    uint8_t  lbuf[6];               /* MPLOAD: the command being collected */
    int      ln;
    /* the vertex path (stage 4 on) */
    uint8_t  vin[16384];            /* the vertex array (MRESET keeps it) */
    uint8_t  vout[32768];           /* the result buffer: 1,024 entries of 32 bytes (kept too) */
    uint16_t vinptr;                /* VINPTR: VINDATA's byte address, 14 bits */
    uint16_t vinidx;                /* VINIDX: the vertex VIN reads, a byte address, bits 1-0 0 */
    uint8_t  vfmt, vposexp;         /* VFMT (5 bits), VPOSEXP (4 bits) */
    uint16_t vtxidx;                /* VTXIDX: the entry the stores go to, 10 bits */
    uint8_t  vbank;                 /* VBANK: the window's 2 KB, 4 bits */
} mcp_state;
/* everything back to its start (MRESET), but the stage and the program
 * memory's contents kept (MPPTR 0, MPLOAD restarted) */
void mcp_reset(mcp_state *s);
/* a command: op $00-$7F a register form (regs = (d << 4) | s, or the
 * data byte of SETCTRL / SETFLAGS / PEXIT), $80-$8F a memory form
 * (regs = d << 4, v the operand); anything else is a bad opcode */
void mcp_exec(mcp_state *s, int op, int regs, uint32_t v);
/* a wide form (stage 1 on: the stream's $90 prefix): op, (d << 4) | s, t << 4 */
void mcp_exec_wide(mcp_state *s, int op, int regs1, int regs2);
/* the stream port, a byte at a time; mcp_stream_restart is MSTREAMRST */
void mcp_stream(mcp_state *s, uint8_t byte);
void mcp_stream_restart(mcp_state *s);
/* MPLOAD, a byte at a time, into the program memory at MPPTR; MPLOADRST */
void mcp_mpload(mcp_state *s, uint8_t byte);
void mcp_mpload_restart(mcp_state *s);
/* MPREAD: the entry at MPPTR as its 8 bytes */
void mcp_mpread(const mcp_state *s, uint8_t out[8]);
/* MSTATUS once idle: bit 7 any flag, bits 2, 3, 5 sticky */
uint8_t mcp_status(const mcp_state *s);

/* the vertex path (stage 4): a byte written to VINDATA ($000-$003), the
 * other registers at $004-$00E written and read, a byte of the window
 * ($800-$FFF; off = the address - $800) */
void mcp_vin_write(mcp_state *s, uint8_t byte);
void mcp_vreg_write(mcp_state *s, int off, uint8_t v);
uint8_t mcp_vreg_read(const mcp_state *s, int off);
uint8_t mcp_window_read(const mcp_state *s, int off);
/* a vertex format's size in bytes, a field's offset in it (0 position, 1
 * normal, 2 colour, 3 attributes; -1 if the format has none) */
int mcp_vfmt_size(int fmt);
int mcp_vfmt_offset(int fmt, int field);
/* VOUT's and VOUTC's conversion: v as format fmt (MCPV_*), rounded by rm;
 * the bits written (sign- or zero-extended to 32), flags as MFLAGS bits */
uint32_t mcp_vout_conv(uint32_t v, int fmt, int rm, uint8_t *flags);
/* VCLIP's codes for (x, y, z, w): bit 0 x < -w, 1 x > w, 2 y < -w, 3 y > w,
 * 4 z < 0, 5 z > w, 6 w <= 0; a NaN anywhere: bit 7 alone */
uint8_t mcp_clip_codes(const uint32_t v[4]);

#endif
