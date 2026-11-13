"""timing: how many cycles things take on a MEGA65, measured, and the same
in Xemu: every 45GS02 opcode at 40.5 MHz, the same instructions against each
kind of memory, the hardware divider, and DMA jobs.

    python3 build/tests/timing-m65/timing.py board [--bit file] [--save name]
    python3 build/tests/timing-m65/timing.py xemu [--save name]
    python3 build/tests/timing-m65/timing.py compare [a] [b]

board: runs the tests on a MEGA65 over its USB JTAG/serial link (m65rs, of
MEGA65 Studio; --bit loads a bitstream first, volatile) and writes the
cycles to results/board.json (--save: another name).
xemu: the same in a headless Xemu (build/bin/xmega65.native here, or $XEMU),
to results/xemu.json.
compare: two result files side by side (default: board and xemu), every
test where they differ by more than a tenth of a cycle.

How a time is taken: the instruction stands R times in a block, the block
runs CNT times, CIA 2's timers count meanwhile (timing.asm). That is done
for two values of R; the difference of the two times is 16 x CNT executions
of the instruction and nothing else: the loop, the timers' start and stop
and whatever sets the registers up are in both. A cycle is one of the
40.5 MHz clock; the CIA's rate is taken from the video mode.

Needs ACME (0.97 or later) and m65rs.
"""
import json
import os
import struct
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
RES = os.path.join(HERE, "results")
M65RS = os.environ.get("M65RS", "/Applications/mega65studio/m65rs")
XEMU = os.environ.get("XEMU", os.path.abspath(os.path.join(HERE, "..", "..", "bin", "xmega65.native")))
XSOCK = "/tmp/xemu-timing.sock"
CLOCK = 40.5e6
REPS = 5
SPAN = 150000    # cycles: no run of a block longer than this, if it can be helped (see T)

# ---- the addresses the harness and the tests share
CODE, PRE, BACK, SUBS, DONE = 0x4000, 0x4800, 0x3F00, 0x3F10, 0x3F20
DATA, RESULTS, TESTS, CNT = 0x5000, 0x6000, 0x40000, 0xFB
BP_FIRST = 0x10
ZP, ZP0, ZPF = 0x10, 0x18, 0x19         # scratch; a byte of $00; a byte of $FF
P16, P16X, P16IO = 0x30, 0x32, 0x34     # pointers: DATA + $10, DATA + $F0, $D020
FAR = {                                 # 32-bit pointers, and what they point at
    "chip RAM": (0x40, 0x0005F000), "colour RAM": (0x44, 0x0FF80800), "attic RAM": (0x48, 0x08000000),
    "I/O (VIC)": (0x4C, 0x0FFD3020), "MCP registers": (0x50, 0x0FFDF400), "MCP window": (0x54, 0x0FFDF800),
    "MCP, nothing there": (0x58, 0x0FFDF300), "ROM": (0x5C, 0x0002A000), "character RAM": (0x60, 0x0FF7E000),
    "chip RAM bank 1": (0x64, 0x0001F000), "base page": (0x68, 0x00000010),
    "chip RAM where colour RAM's first 2K shows": (0x6C, 0x0001F800),
    "I2C (the clock)": (0x70, 0x0FFD7110), "SD card buffer": (0x74, 0x0FFD6E00),
    "ethernet buffer": (0x78, 0x0FFDE800),
}
READ_ONLY = ("MCP registers", "MCP window", "ROM", "I2C (the clock)", "ethernet buffer")
PCHIP = FAR["chip RAM"][0]

NAMES = """BRK ORA CLE SEE TSB ORA ASL RMB0 PHP ORA ASL TSY TSB ORA ASL BBR0
BPL ORA ORA BPL TRB ORA ASL RMB1 CLC ORA INC INZ TRB ORA ASL BBR1
JSR AND JSR JSR BIT AND ROL RMB2 PLP AND ROL TYS BIT AND ROL BBR2
BMI AND AND BMI BIT AND ROL RMB3 SEC AND DEC DEZ BIT AND ROL BBR3
RTI EOR NEG ASR ASR EOR LSR RMB4 PHA EOR LSR TAZ JMP EOR LSR BBR4
BVC EOR EOR BVC ASR EOR LSR RMB5 CLI EOR PHY TAB MAP EOR LSR BBR5
RTS ADC RTS BSR STZ ADC ROR RMB6 PLA ADC ROR TZA JMP ADC ROR BBR6
BVS ADC ADC BVS STZ ADC ROR RMB7 SEI ADC PLY TBA JMP ADC ROR BBR7
BRA STA STA BRA STY STA STX SMB0 DEY BIT TXA STY STY STA STX BBS0
BCC STA STA BCC STY STA STX SMB1 TYA STA TXS STX STZ STA STZ BBS1
LDY LDA LDX LDZ LDY LDA LDX SMB2 TAY LDA TAX LDZ LDY LDA LDX BBS2
BCS LDA LDA BCS LDY LDA LDX SMB3 CLV LDA TSX LDZ LDY LDA LDX BBS3
CPY CMP CPZ DEW CPY CMP DEC SMB4 INY CMP DEX ASW CPY CMP DEC BBS4
BNE CMP CMP BNE CPZ CMP DEC SMB5 CLD CMP PHX PHZ CPZ CMP DEC BBS5
CPX SBC LDA INW CPX SBC INC SMB6 INX SBC NOP ROW CPX SBC INC BBS6
BEQ SBC SBC BEQ PHW SBC INC SMB7 SED SBC PLX PLZ PHW SBC INC BBS7""".split()
ADMS = [int(x) for x in """
 0 15 0 0 3 3 3 3 0 1 18 0 7 7 7 4  10 12 13 11 3 5 5 3 0 9 0 0 7 8 8 4
 7 15 16 17 3 3 3 3 0 1 18 0 7 7 7 4  10 12 13 11 5 5 5 3 0 9 0 0 8 8 8 4
 0 15 0 0 3 3 3 3 0 1 18 0 7 7 7 4  10 12 13 11 5 5 5 3 0 9 0 0 0 8 8 4
 0 15 1 11 3 3 3 3 0 1 18 0 16 7 7 4  10 12 13 11 5 5 5 3 0 9 0 0 17 8 8 4
 10 15 14 11 3 3 3 3 0 1 0 8 7 7 7 4  10 12 13 11 5 5 6 3 0 9 0 9 7 8 8 4
 1 15 1 1 3 3 3 3 0 1 0 7 7 7 7 4  10 12 13 11 5 5 6 3 0 9 0 8 8 8 9 4
 1 15 1 3 3 3 3 3 0 1 0 7 7 7 7 4  10 12 13 11 3 5 5 3 0 9 0 0 7 8 8 4
 1 15 14 3 3 3 3 3 0 1 0 7 7 7 7 4  10 12 13 11 2 5 5 3 0 9 0 0 7 8 8 4""".split()]
ADM = ["", "#$nn", "#$nnnn", "$nn", "$nn,$rr", "$nn,X", "$nn,Y", "$nnnn", "$nnnn,X", "$nnnn,Y", "$rr", "$rrrr",
       "($nn),Y", "($nn),Z", "($nn,SP),Y", "($nn,X)", "($nnnn)", "($nnnn,X)", "A"]
QOPS = {0x05: "ORQ", 0x06: "ASLQ", 0x0A: "ASLQ", 0x0D: "ORQ", 0x0E: "ASLQ", 0x12: "ORQ", 0x16: "ASLQ", 0x1A: "INQ",
        0x1E: "ASLQ", 0x24: "BITQ", 0x25: "ANDQ", 0x26: "ROLQ", 0x2A: "ROLQ", 0x2C: "BITQ", 0x2D: "ANDQ",
        0x2E: "ROLQ", 0x32: "ANDQ", 0x36: "ROLQ", 0x3A: "DEQ", 0x3E: "ROLQ", 0x42: "NEGQ", 0x43: "ASRQ",
        0x44: "ASRQ", 0x45: "EORQ", 0x46: "LSRQ", 0x4A: "LSRQ", 0x4D: "EORQ", 0x4E: "LSRQ", 0x52: "EORQ",
        0x54: "ASRQ", 0x56: "LSRQ", 0x5E: "LSRQ", 0x65: "ADCQ", 0x66: "RORQ", 0x6A: "RORQ", 0x6D: "ADCQ",
        0x6E: "RORQ", 0x72: "ADCQ", 0x76: "RORQ", 0x7E: "RORQ", 0x85: "STQ", 0x8D: "STQ", 0x92: "STQ",
        0xA5: "LDQ", 0xAD: "LDQ", 0xB2: "LDQ", 0xC5: "CMPQ", 0xC6: "DEQ", 0xCD: "CMPQ", 0xCE: "DEQ",
        0xD2: "CMPQ", 0xD6: "DEQ", 0xDE: "DEQ", 0xE5: "SBCQ", 0xE6: "INQ", 0xED: "SBCQ", 0xEE: "INQ",
        0xF2: "SBCQ", 0xF6: "INQ", 0xFE: "INQ"}

# flags for the branches, set at the top of every run of the block (the loop's DEW changes N and Z):
# N V C clear, Z clear; N V C set (through PLP: I and E stay set); Z set
HEAD_CLEAR = "clc\nclv\nlda #1"
HEAD_SET = "lda #$e5\npha\nplp"
HEAD_ZERO = "lda #0"
TAKEN = {0x10: HEAD_CLEAR, 0x30: HEAD_SET, 0x50: HEAD_CLEAR, 0x70: HEAD_SET, 0x80: HEAD_CLEAR, 0x90: HEAD_CLEAR,
         0xB0: HEAD_SET, 0xD0: HEAD_CLEAR, 0xF0: HEAD_ZERO}
NOT_TAKEN = {0x10: HEAD_SET, 0x30: HEAD_CLEAR, 0x50: HEAD_SET, 0x70: HEAD_CLEAR, 0x90: HEAD_SET, 0xB0: HEAD_CLEAR,
             0xD0: HEAD_ZERO, 0xF0: HEAD_CLEAR}
LONG = {0x13: "lbpl", 0x33: "lbmi", 0x53: "lbvc", 0x73: "lbvs", 0x83: "lbra", 0x93: "lbcc", 0xB3: "lbcs",
        0xD3: "lbne", 0xF3: "lbeq"}

tests = []      # dicts: name, body, pre, head, post, data, m, r, sub


def T(name, body, pre="", head="", post="", data="", m=512, r=(4, 20), sub=(), about=None):
    """a test: body is ACME source for one instance ({i}: its number, for labels); sub: (name, times) of
    tests whose cycles are part of this body and are taken off; about: the cycles an instance is expected
    to take, when that is many: the block then runs fewer times, so that a run stays within SPAN.
    (The CIA's clock on a MEGA65 loses about 30 usec once a video frame: a run should be well inside a
    frame, so that of its REPS runs one at least has no such step in it; the longest is taken.)"""
    assert name not in [t["name"] for t in tests], name
    if about:
        m = max(1, min(m, int(SPAN / (about * r[1]))))
    tests.append(dict(name=name, body=body, pre=pre, head=head, post=post, data=data, m=m, r=r, sub=list(sub)))


def by(*b):
    return "!byte " + ", ".join(f"${x & 255:02x}" for x in b)


def w(v):
    return (v & 255, v >> 8)


def opname(op, suffix=""):
    return f"{op:02X} {NAMES[op]} {ADM[ADMS[op]]}".rstrip() + suffix


def base_opcodes():
    for op in range(256):
        a, nm = ADMS[op], NAMES[op]
        if op in (0x00, 0x5C):                      # BRK, MAP: not here
            continue
        if op == 0x40:                              # RTI: a frame pushed by hand first
            T("40 RTI", "lda #>.n{i}\npha\nlda #<.n{i}\npha\nphp\nrti\n.n{i}",
              sub=[("A9 LDA #$nn", 2), ("48 PHA", 2), ("08 PHP", 1)])
        elif op == 0x60:
            T("20 JSR $nnnn, 60 RTS", f"jsr ${SUBS:04x}")
            T("60 RTS", f"jsr ${SUBS:04x}", sub=[("20 JSR $nnnn", 1)])
        elif op == 0x62:                            # RTS #$nn (RTN)
            T("62 RTS #$nn", f"jsr ${SUBS + 1:04x}", sub=[("20 JSR $nnnn", 1)])
        elif op == 0x20:
            T("20 JSR $nnnn", "jsr .n{i}\n.n{i}\npla\npla", sub=[("68 PLA", 2)])
        elif op == 0x22:
            T("22 JSR ($nnnn)", f"jsr (${SUBS + 3:04x})", sub=[("60 RTS", 1)])
        elif op == 0x23:
            T("23 JSR ($nnnn,X)", f"jsr (${SUBS + 3:04x},x)", pre="ldx #0", sub=[("60 RTS", 1)])
        elif op == 0x63:
            T("63 BSR $rrrr", "bsr .n{i}\n.n{i}\npla\npla", sub=[("68 PLA", 2)])
        elif op == 0x4C:
            T("4C JMP $nnnn", "jmp .n{i}\n.n{i}")
        elif op == 0x6C:
            T("6C JMP ($nnnn)", "jmp (.v{i})\n.n{i}", data=".v{i} !word .n{i}")
        elif op == 0x7C:
            T("7C JMP ($nnnn,X)", "jmp (.v{i},x)\n.n{i}", pre="ldx #0", data=".v{i} !word .n{i}")
        elif op == 0x28:
            T("28 PLP", "php\nplp", sub=[("08 PHP", 1)])
        elif op == 0x42:                            # (two in a row are the Q prefix)
            T("42 NEG", by(0x42) + "\nclc", sub=[("18 CLC", 1)])
        elif op == 0xEA:                            # (a prefix too, before an indirect one)
            T("EA NOP", by(0xEA) + "\nclc", sub=[("18 CLC", 1)])
        elif a in (0, 18):
            pre = {0x5B: "lda #0", 0x9A: "ldx #$ff", 0x2B: "ldy #1"}.get(op, "")
            post = {0x58: "sei", 0xF8: "cld", 0x02: "see"}.get(op, "")
            T(opname(op), by(op), pre=pre, post=post)
        elif a == 1:
            T(opname(op), by(op, 1))
            if nm in ("ADC", "SBC"):
                T(opname(op, ", decimal mode"), by(op, 1), head="sed", post="cld")
        elif a == 2:
            T(opname(op), by(op, 0x34, 0x12))
        elif a in (3, 5, 6):
            T(opname(op), by(op, ZP))
        elif a == 4:                                # BBRn, BBSn: on a bit of a byte of $00 or of $FF
            reset = nm.startswith("BBR")
            T(opname(op, ", taken"), by(op, ZP0 if reset else ZPF, 0))
            T(opname(op, ", not taken"), by(op, ZPF if reset else ZP0, 0))
        elif a == 7:
            T(opname(op), by(op, *w(DATA)))
        elif a in (8, 9):
            reg = "x" if a == 8 else "y"
            T(opname(op), by(op, *w(DATA + 0x10)))
            T(opname(op, ", a page crossed"), by(op, *w(DATA + 0xF0)), pre=f"ld{reg} #$20")
        elif a == 10:
            T(opname(op, ", taken"), by(op, 0), head=TAKEN[op])
            if op in NOT_TAKEN:
                T(opname(op, ", not taken"), by(op, 0), head=NOT_TAKEN[op])
        elif a == 11:
            T(opname(op, ", taken"), LONG[op] + " .n{i}\n.n{i}", head=TAKEN[op - 3])
            if op - 3 in NOT_TAKEN:
                T(opname(op, ", not taken"), LONG[op] + " .n{i}\n.n{i}", head=NOT_TAKEN[op - 3])
        elif a in (12, 13):
            reg = "y" if a == 12 else "z"
            T(opname(op), by(op, P16))
            T(opname(op, ", a page crossed"), by(op, P16X), pre=f"ld{reg} #$20")
        elif a == 14:
            T(opname(op), by(op, 1), pre="ldx #$f0\ntxs\nldy #0")
        elif a == 15:
            T(opname(op), by(op, P16), pre="ldx #0")
        else:
            raise SystemExit(f"opcode ${op:02X}: no test")
    # a branch taken across a page boundary, 8 and 16 bits of offset (the JMP that leads to each is taken off)
    T("90 BCC $rr, taken, a page crossed", "jmp .b{i}\n!align 255, $f0\n.b{i} bcc .n{i}\n!fill $20, $ea\n.n{i}",
      head=HEAD_CLEAR, r=(1, 5), m=2048, sub=[("4C JMP $nnnn", 1)], about=10)
    T("93 BCC $rrrr, taken, a page crossed", "jmp .b{i}\n!align 255, $f0\n.b{i} lbcc .n{i}\n!fill $20, $ea\n.n{i}",
      head=HEAD_CLEAR, r=(1, 5), m=2048, sub=[("4C JMP $nnnn", 1)], about=10)


def q_opcodes():
    """the Q instructions (two NEG before) and the 32-bit indirect ones (a NOP before)"""
    for op, nm in sorted(QOPS.items()):
        a = ADMS[op]
        if op == 0x42:
            T("42 42 42 NEGQ", by(0x42, 0x42, 0x42))
        elif a in (0, 18) or op == 0x43:
            T(f"42 42 {op:02X} {nm} Q", by(0x42, 0x42, op))
        elif a in (3, 5):
            T(f"42 42 {op:02X} {nm} {ADM[a]}", by(0x42, 0x42, op, ZP))
        elif a in (7, 8):
            T(f"42 42 {op:02X} {nm} {ADM[a]}", by(0x42, 0x42, op, *w(DATA + 0x10)))
        elif a == 13:
            z = ",Z" if nm == "LDQ" else ""
            T(f"42 42 {op:02X} {nm} ($nn){z}", "ldz #0\n" + by(0x42, 0x42, op, P16), sub=[("A3 LDZ #$nn", 1)])
            T(f"42 42 EA {op:02X} {nm} [$nn]{z}", "ldz #0\n" + by(0x42, 0x42, 0xEA, op, PCHIP),
              sub=[("A3 LDZ #$nn", 1)])
        else:
            raise SystemExit(f"Q opcode ${op:02X}: no test")
    for op in (0x12, 0x32, 0x52, 0x72, 0x92, 0xB2, 0xD2, 0xF2):
        T(f"EA {op:02X} {NAMES[op]} [$nn],Z", by(0xEA, op, PCHIP))
        T(f"EA {op:02X} {NAMES[op]} [$nn],Z, a page crossed", by(0xEA, op, PCHIP), pre="ldz #$ff",
          head="lda #$f0\nsta $%02x" % PCHIP, post="lda #0\nsta $%02x" % PCHIP)


def memory_kinds(with_mcp):
    """the same instructions against each kind of memory"""
    for kind, (ptr, addr) in FAR.items():
        if kind.startswith("MCP") and not with_mcp:
            continue
        where = f"{kind}, ${addr:07X}"
        T(f"LDA [$nn],Z: {where}", "ldz #0\n" + by(0xEA, 0xB2, ptr), sub=[("A3 LDZ #$nn", 1)])
        T(f"LDQ [$nn],Z: {where}", "ldz #0\n" + by(0x42, 0x42, 0xEA, 0xB2, ptr), sub=[("A3 LDZ #$nn", 1)])
        if kind not in READ_ONLY:
            T(f"STA [$nn],Z: {where}", "ldz #0\n" + by(0xEA, 0x92, ptr), sub=[("A3 LDZ #$nn", 1)])
            T(f"STQ [$nn]: {where}", "ldz #0\n" + by(0x42, 0x42, 0xEA, 0x92, ptr), sub=[("A3 LDZ #$nn", 1)])
    for name, body, extra in (
        ("LDA $D020 (VIC)", "lda $d020", {}), ("STA $D020 (VIC)", "sta $d020", {}),
        ("INC $D020 (VIC)", "inc $d020", {}), ("LDA $D012 (VIC)", "lda $d012", {}),
        ("LDA $D800 (colour RAM)", "lda $d800", {}), ("STA $D800 (colour RAM)", "sta $d800", {}),
        ("LDA $DC00 (CIA 1)", "lda $dc00", {}), ("LDA $DD04 (CIA 2)", "lda $dd04", {}),
        ("BIT $D70F (the divider's status)", "bit $d70f", {}), ("STA $D770 (the multiplier)", "sta $d770", {}),
        ("LDA $D400 (SID)", "lda $d400", {}), ("LDA $D100 (palette)", "lda $d100", {}),
        ("LDA $D080 (floppy controller)", "lda $d080", {}), ("LDA $D610 (keyboard queue)", "lda $d610", {}),
        ("LDA $D680 (SD card)", "lda $d680", {}), ("LDA $D703 (DMA)", "lda $d703", {}),
        ("LDA $D7FA (frame counter)", "lda $d7fa", {}), ("LDA $D050 (VIC-IV)", "lda $d050", {}),
        ("LDA ($nn),Y: $D020", by(0xB1, P16IO), {"pre": "ldy #0"}),
        ("STA ($nn),Y: $D020", by(0x91, P16IO), {"pre": "ldy #0"}),
        ("LDA $E000 (ROM)", "lda $e000", {}), ("LDA $A000", "lda $a000", {}), ("LDA $C000", "lda $c000", {}),
        ("LDA $0100 (the stack's page)", "lda $0100", {}), ("LDA $0010 (the base page, as $nnnn)", "lda $0010", {}),
    ):
        T(name, body, **extra)
    # the hardware divider and multiplier: an operand written, k one-cycle instructions, then DIVBUSY
    # (bit 7 of $D70F) or MULBUSY (bit 6) asked with a branch that takes 2 cycles more when it is set
    def operands(a, b):
        return "\n".join(f"lda #${(v >> (8 * k)) & 255:02x}\nsta ${0xD770 + j * 4 + k:04x}"
                         for j, v in enumerate((a, b)) for k in range(4))
    for what, a, b, wr in (("1000 / 7", 1000, 7, "lda #7\nsta $d774"),
                           ("$FFFFFFFF / 3", 0xFFFFFFFF, 3, "lda #3\nsta $d774"),
                           ("5 / $FFFFFFF0", 5, 0xFFFFFFF0, "lda #$ff\nsta $d777")):
        for k in list(range(0, 26)) + [28, 32, 40]:
            T(f"the divider, {what}: busy after {k} cycles?", wr + "\n" + "clc\n" * k + "bit $d70f\nbmi .n{i}\n.n{i}",
              pre=operands(a, b), sub=[("A9 LDA #$nn", 1), ("18 CLC", k)], m=2048, r=(2, 6), about=16 + k)
    for k in range(0, 6):
        T(f"the multiplier: busy after {k} cycles?", "lda #9\nsta $d770\n" + "clc\n" * k + "bit $d70f\nbvs .n{i}\n.n{i}",
          pre=operands(123456, 789), sub=[("A9 LDA #$nn", 1), ("18 CLC", k)], m=2048, r=(2, 6), about=16 + k)
    # the CIA's clock over many video frames: the same instruction as "EE INC $nnnn", for a fifth of a second
    T("EE INC $nnnn, timed over about 11 video frames", by(0xEE, *w(DATA)), m=65535)
    T("EE INC $nnnn, timed over about 40 video frames", by(0xEE, *w(DATA)), m=65535, r=(4, 68))


TRIGGER = "lda #0\nsta $d702\nsta $d704\nlda #>.list\nsta $d701\nlda #<.list\nsta $d705"


def dma_list(cmd, n, src, dst, opts=(), sub=0, chain=False):
    """an enhanced (F018B) DMA list entry; src and dst as 28-bit addresses (megabytes go in as options)"""
    o = list(opts)
    if src >> 20:
        o += [0x80, src >> 20]
    if dst >> 20:
        o += [0x81, dst >> 20]
    return (by(0x0B, *o, 0x00) + "\n" + by(cmd | (4 if chain else 0), *w(n), *w(src & 0xFFFF), (src >> 16) & 15,
                                         *w(dst & 0xFFFF), (dst >> 16) & 15, sub, 0, 0))


def dma(with_mcp):
    """DMA jobs: each at two lengths (what a byte takes, and what a job takes before its first)"""
    chip, chip2, col, attic, io = 0x18000, 0x1C000, 0x0FF80800, 0x08000000, 0x0FFD3020
    kinds = [
        ("fill, chip RAM", 3, 0x55, chip, (), 0), ("copy, chip RAM to chip RAM", 0, chip2, chip, (), 0),
        ("fill, colour RAM", 3, 0x55, col, (), 0), ("copy, chip RAM to colour RAM", 0, chip2, col, (), 0),
        ("copy, colour RAM to chip RAM", 0, col, chip, (), 0),
        ("fill, attic RAM", 3, 0x55, attic, (), 0), ("copy, chip RAM to attic RAM", 0, chip2, attic, (), 0),
        ("copy, attic RAM to chip RAM", 0, attic, chip, (), 0),
        ("copy, attic RAM to attic RAM", 0, attic + 0x4000, attic, (), 0),
        ("fill, I/O (VIC), destination held", 3, 0x00, io, (), 0x08),
        ("copy, chip RAM to I/O (VIC), destination held", 0, chip2, io, (), 0x08),
        ("fill, chip RAM, every second byte", 3, 0x55, chip, (0x85, 0x02), 0),
        ("copy, chip RAM, every 32nd byte of the source", 0, chip2, chip, (0x83, 0x20), 0),
        ("copy, ROM to chip RAM", 0, 0x2A000, chip, (), 0),
        ("copy, I/O (VIC) to chip RAM", 0, 0x0FFD3100, chip, (), 0),
        ("copy, chip RAM where colour RAM's first 2K shows to chip RAM", 0, 0x1F800, chip, (), 0),
    ]
    if with_mcp:
        kinds += [("copy, MCP window to chip RAM", 0, 0x0FFDF800, chip, (), 0),
                  ("copy, chip RAM to the MCP (nothing there), destination held", 0, chip2, 0x0FFDF300, (), 0x08)]
    for name, cmd, src, dst, opts, sub in kinds:
        attic = (src >> 20 == 0x80 if cmd == 0 else False) + (dst >> 20 == 0x80)
        for n in (64, 1088):
            T(f"DMA {name}, {n} bytes", TRIGGER, data=".list\n" + dma_list(cmd, n, src, dst, opts, sub),
              r=(1, 5), about=40 + n * (2 + 12 * attic if cmd == 0 else 1 + 4 * attic))
    for n in (1, 2, 16):
        T(f"DMA fill, chip RAM, {n} byte{'s' if n > 1 else ''}", TRIGGER, data=".list\n" + dma_list(3, n, 0x55, chip),
          r=(1, 5), about=40 + n)
        T(f"DMA copy, chip RAM to chip RAM, {n} byte{'s' if n > 1 else ''}", TRIGGER,
          data=".list\n" + dma_list(0, n, chip2, chip), r=(1, 5), about=40 + 2 * n)
    T("DMA fill, chip RAM, 1 byte, six more option bytes", TRIGGER,
      data=".list\n" + dma_list(3, 1, 0x55, chip, (0x82, 0, 0x83, 1, 0x84, 0)), r=(1, 5), about=50)
    T("DMA two chained fills of 1 byte, chip RAM", TRIGGER,
      data=".list\n" + dma_list(3, 1, 0x55, chip, chain=True) + "\n" + dma_list(3, 1, 0x55, chip + 16), r=(1, 5), about=60)
    T("DMA two chained copies of 16 bytes, chip RAM", TRIGGER,
      data=".list\n" + dma_list(0, 16, chip2, chip, chain=True) + "\n" + dma_list(0, 16, chip2, chip + 16), r=(1, 5), about=110)
    T("DMA fill, chip RAM, 1 byte, an old-style list (F018A, no options)", "lda #0\nsta $d702\nlda #>.list\nsta $d701\nlda #<.list\nsta $d700",
      data=".list\n" + by(3, 1, 0, 0x55, 0, 0, *w(chip & 0xFFFF), chip >> 16, 0, 0, 0), r=(1, 5), about=40)
    T("DMA copy, chip RAM to chip RAM, 16 bytes, an old-style list", "lda #0\nsta $d702\nlda #>.list\nsta $d701\nlda #<.list\nsta $d700",
      data=".list\n" + by(0, 16, 0, *w(chip2 & 0xFFFF), chip2 >> 16, *w(chip & 0xFFFF), chip >> 16, 0, 0, 0), r=(1, 5), about=70)


def source(chunk):
    """ACME source for some tests' records: each test twice, for its two numbers of instances"""
    lines = ["!cpu m65", '!source "timing.inc"', "* = 0"]
    for t in chunk:
        for r in t["r"]:
            body = "\n".join(t["body"].replace("{i}", str(i)) for i in range(r))
            data = "\n".join(t["data"].replace("{i}", str(i)) for i in range(r)) if "{i}" in t["data"] else t["data"]
            lines += ["!zone {", "!byte .pe - .ps", "!word .be - .bs", f"!word {t['m']}",
                      ".ps !pseudopc PRE {", "ldx #1", "ldy #1", "ldz #1", "lda #1", "clc", "clv", t["pre"],
                      "jmp CODE", "}", ".pe",
                      ".bs !pseudopc CODE {", ".top", t["head"], body, "dew CNT", "beq .out", "jmp .top", ".out",
                      t["post"], "jmp BACK", data, "}", ".be", "}"]
    return "\n".join(lines) + "\n"


def run(*args, **kw):
    r = subprocess.run(list(args), capture_output=True, text=True, **kw)
    if r.returncode:
        sys.exit(f"{' '.join(args[:4])} ...:\n{r.stdout}{r.stderr}")
    return r.stdout


def build(with_mcp):
    os.makedirs(OUT, exist_ok=True)
    del tests[:]
    base_opcodes()
    q_opcodes()
    memory_kinds(with_mcp)
    dma(with_mcp)
    inc = dict(CODE=CODE, PRE=PRE, BACK=BACK, SUBS=SUBS, DONE=DONE, DATA=DATA, RESULTS=RESULTS, TESTS=TESTS,
               CNT=CNT, BP_FIRST=BP_FIRST, REPS=REPS)
    open(os.path.join(OUT, "timing.inc"), "w").write("".join(f"{k} = ${v:x}\n" for k, v in inc.items()))
    bp = bytearray(b"\x01" * 0x70)                  # the base page from BP_FIRST: scratch bytes of 1, then
    bp[ZP0 - BP_FIRST], bp[ZPF - BP_FIRST] = 0x00, 0xFF
    for at, v in ((P16, DATA + 0x10), (P16X, DATA + 0xF0), (P16IO, 0xD020)):
        bp[at - BP_FIRST:at - BP_FIRST + 2] = struct.pack("<H", v)
    for at, v in FAR.values():
        bp[at - BP_FIRST:at - BP_FIRST + 4] = struct.pack("<I", v)
    open(os.path.join(OUT, "bp.inc"), "w").write("!byte " + ", ".join(str(b) for b in bp) + "\n")
    blob = bytearray()
    for k in range(0, len(tests), 60):              # (ACME's program counter is 16 bits: in pieces)
        src, binf = os.path.join(OUT, f"tests{k}.asm"), os.path.join(OUT, f"tests{k}.bin")
        open(src, "w").write(source(tests[k:k + 60]))
        run("acme", "-f", "plain", "-I", OUT, "-o", binf, src)
        blob += open(binf, "rb").read()
    blob += b"\xff"
    assert len(blob) < 0x1E000, f"{len(blob)} bytes of tests: they would reach the chip RAM pointer's target"
    assert 2 * len(tests) * 4 <= 0x2000, "more results than fit below $8000"
    open(os.path.join(OUT, "tests.bin"), "wb").write(blob)
    run("acme", "-f", "cbm", "-I", OUT, "-o", os.path.join(OUT, "timing.prg"), os.path.join(HERE, "timing.asm"))
    return len(blob)


link = []


def m65(*args):
    return run(M65RS, *link, *args)


def peek(addr):
    return int(m65("peek", f"{addr:x}").split("$")[-1].split()[0], 16)


def start_xemu():
    link[:] = ["-l", XSOCK]
    if not os.path.exists(XEMU):
        sys.exit(f"no Xemu at {XEMU}: make in targets/mega65, or set XEMU")
    if os.path.exists(XSOCK):
        os.remove(XSOCK)
    p = subprocess.Popen([XEMU, "-headless", "-besure", "-fastboot", "-uartmon", XSOCK], cwd="/tmp",
                         stdout=open(os.path.join(OUT, "xemu.log"), "w"), stderr=subprocess.STDOUT)
    for _ in range(60):
        time.sleep(0.5)
        if subprocess.run([M65RS, *link, "info"], capture_output=True).returncode == 0:
            break
    else:
        p.kill()
        sys.exit("Xemu did not answer")
    time.sleep(4)
    return p


def boot(bit):
    """the bitstream over JTAG (volatile). A load now and then does not take: three tries. Then BASIC's
    READY: this ROM, started so, waits for CIA 2's timer A, which is started by hand"""
    for _ in range(3):
        r = subprocess.run([M65RS, "jtag", "bit", bit], capture_output=True, text=True)
        if r.returncode == 0:
            break
        print("the load did not take: loading again", flush=True)
    else:
        sys.exit(r.stdout + r.stderr)
    time.sleep(10)
    if m65("info").splitlines()[3][:3] == "56E":
        m65("poke", "ffd3d04", "ff", "ff")
        m65("poke", "ffd3d0e", "11")
        time.sleep(6)


def measure(where, bit):
    xemu = None
    if where == "xemu":
        os.makedirs(OUT, exist_ok=True)
        xemu = start_xemu()
    elif bit:
        boot(bit)
    try:
        with_mcp = peek(0xFFDF101) == 0x4D
        size = build(with_mcp)
        print(f"{len(tests)} tests ({size} bytes){', the MCP ones among them' if with_mcp else ''}", flush=True)
        blob = os.path.join(OUT, "tests.bin")
        if xemu:
            m65("raw", f"~load {TESTS:x} {blob}")
        else:
            m65("load", f"{blob}@{TESTS:x}")
        m65("poke", f"{DONE:x}", "00")
        ntsc = peek(0xFFD306F) & 0x80
        m65("prg", "-r", os.path.join(OUT, "timing.prg"))
        t0 = time.time()
        while peek(DONE) != 0xA5:
            if time.time() - t0 > 300:
                done = sum(1 for k in range(0, 2 * len(tests) * 4, 4) if peek(RESULTS + k + 3) != 0)
                sys.exit(f"no end after 300 s (about {done} of {2 * len(tests)} records have a result)")
            time.sleep(0.5)
        path = os.path.join(OUT, f"results_{where}.bin")
        m65("save", f"{RESULTS:x}", f"{RESULTS + 2 * len(tests) * 4:x}", path)
        info = m65("info")
    finally:
        if xemu:
            xemu.terminate()
    raw = open(path, "rb").read()
    hz = 27e6 * 65 / 1716 if ntsc else 27e6 * 63 / 1728
    cyc = {}
    for k, t in enumerate(tests):
        a, b = (0xFFFFFFFF - struct.unpack_from("<I", raw, 4 * (2 * k + j))[0] for j in (0, 1))
        cyc[t["name"]] = (b - a) / hz * CLOCK / (t["m"] * (t["r"][1] - t["r"][0]))
    for t in tests:                                 # what else was in a body
        for other, times in t["sub"]:
            cyc[t["name"]] -= times * raw_cycles(cyc, other)
    core = next((line for line in info.splitlines() if line.startswith("core:")), "")
    return dict(where=where, video="NTSC" if ntsc else "PAL", cia_hz=hz, core=core, cycles=cyc)


def raw_cycles(cyc, name):
    """what is taken off: a whole number of cycles when it is one to the timer's resolution (so that
    taking it off 40 times does not add its hundredths up)"""
    if name not in cyc:
        sys.exit(f"a test takes off '{name}', which is no test")
    c = cyc[name]
    return round(c) if abs(c - round(c)) < 0.03 else c


def compare(a, b, limit=0.1):
    ra, rb = (json.load(open(p)) for p in (a, b))
    ca, cb = ra["cycles"], rb["cycles"]
    names = [n for n in ca if n in cb]
    diff = [n for n in names if abs(ca[n] - cb[n]) > limit]
    print(f"{ra['where']} ({ra['video']}; {ra['core']}) against {rb['where']} ({rb['video']})")
    print(f"{len(names)} tests in both; {len(names) - len(diff)} within {limit} of a cycle, {len(diff)} not:\n")
    print(f"{'':58} {ra['where']:>8} {rb['where']:>8}")
    for n in diff:
        print(f"{n[:58]:58} {ca[n]:8.2f} {cb[n]:8.2f}  {cb[n] - ca[n]:+.2f}")
    only = [n for n in list(ca) + list(cb) if n not in names]
    if only:
        print(f"\nin one only: {len(only)} tests ({', '.join(only[:4])}{' ...' if len(only) > 4 else ''})")
    return len(diff)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    opt = {a: sys.argv[sys.argv.index(a) + 1] for a in ("--bit", "--save") if a in sys.argv}
    args = [a for a in args if a not in opt.values()]
    if not args or args[0] not in ("board", "xemu", "compare", "build", "show"):
        sys.exit(__doc__)
    os.makedirs(RES, exist_ok=True)
    if args[0] == "compare":
        a, b = (args[1:] + [None, None])[:2]
        sys.exit(1 if compare(a or os.path.join(RES, "board.json"), b or os.path.join(RES, "xemu.json")) else 0)
    if args[0] == "build":
        print(f"{build(True)} bytes of tests, {len(tests)} tests")
        return
    if args[0] == "show":
        r = json.load(open(args[1] if len(args) > 1 else os.path.join(RES, "board.json")))
        for n, c in r["cycles"].items():
            print(f"{n[:70]:70} {c:8.2f}")
        return
    res = measure(args[0], opt.get("--bit"))
    path = os.path.join(RES, opt.get("--save", args[0]) + ".json")
    json.dump(res, open(path, "w"), indent=1)
    odd = [n for n, c in res["cycles"].items() if abs(c - round(c)) > 0.08 and not n.startswith("DMA")]
    print(f"{len(res['cycles'])} tests timed ({res['video']}, the CIA at {res['cia_hz']:.0f} Hz): {path}")
    if odd:
        print(f"{len(odd)} are not a whole number of cycles: " + "; ".join(f"{n} {res['cycles'][n]:.2f}" for n in odd[:12]))


if __name__ == "__main__":
    main()
