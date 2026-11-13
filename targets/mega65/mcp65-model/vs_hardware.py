"""vs_hardware: Xemu's M65-MCP unit (../mcp65_unit.c) alone, without Xemu,
against the M65-MCP's reference model and, clock by clock, against its
hardware (the VHDL, simulated).

    python3 targets/mega65/mcp65-model/vs_hardware.py /path/to/m65-mcp [--sim] [seed ...]

Always: random bus scripts (the M65-MCP's own generator, mcpgen: commands
by every path, programs, the vertex path, resets, the overrun and MPABORT
tests) through the unit, every read compared with the value the model
gives. Default seeds 1 2 3, 3,000 commands each (NCMD=n for more).

--sim (needs nvc): the same through the hardware's test bench
(src/mcp/sim/tb_mcp.vhdl over src/vhdl/mcp*.vhdl) too, and the clocks
compared:
  - programs: every command alone in a pass and four in a row (the list
    of the M65-MCP's demo/vtime), and mixes that wait for each other, for
    the divider, for MCOND, for stores: clocks a pass over 1,000 passes
  - idle: after each kind of command, the clock in which MSTATUS first
    says idle, probed at 24 delays
  - the random scripts: the clock at every wait's end, all the way

Needs a C compiler, and src/mcp/sim/fetch.sh run once in the M65-MCP's
repository (mcpgen is built on its SoftFloat).
"""
import os
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
PASSES = 1000


def sh(*args, ok=False, cwd=None):
    """a command's output (ok: whatever its exit code)"""
    r = subprocess.run(list(args), capture_output=True, text=True, cwd=cwd)
    if r.returncode and not ok:
        sys.exit(f"{' '.join(args[:3])} ...: {r.stdout}{r.stderr}")
    return r.stdout + r.stderr


def reg(op, d, s):                      # the three forms, in the stream's format (for programs)
    return bytes((op, d << 4 | s))


def mem(mf, d, v):
    return bytes((0x80 + mf, d << 4)) + (struct.pack("<f", v) if isinstance(v, float) else struct.pack("<I", v))


def wide(op, d, s, t):
    return bytes((0x90, op, d << 4 | s, t << 4))


def bus(stream):
    """the same commands as the writes that issue them by their own windows"""
    out, i = [], 0
    while i < len(stream):
        b = stream[i]
        if b == 0x90:
            out += [(0x200 + 2 * stream[i + 1], stream[i + 2]), (0x201 + 2 * stream[i + 1], stream[i + 3])]
            i += 4
        elif b >= 0x80:
            a = 0x400 + (b & 15) * 64 + (stream[i + 1] >> 4) * 4
            out += [(a + k, stream[i + 2 + k]) for k in range(4)]
            i += 6
        else:
            out.append((0x180 + b, stream[i + 1]))
            i += 2
    return out


class Script:
    def __init__(self):
        self.lines = []

    def w(self, a, v):
        self.lines.append(f"W {a:03X} {v & 255:02X}")

    def add(self, *lines):
        self.lines += lines

    def registers(self, vfmt=0x1F):
        """F0-F15 = 1.25 + i / 4, M the same, the vertex registers"""
        self.add("I")
        for i in range(16):
            v = struct.pack("<I", 0x3FA00000 + i * 0x40000)
            for k in range(4):
                self.w(0x400 + 4 * i + k, v[k])
        self.w(0x240, 0)
        self.w(0x241, 0)                # MLOAD
        self.add("I")
        for a, v in ((0x008, vfmt), (0x009, 0), (0x006, 0), (0x007, 0), (0x00C, 0), (0x00D, 0)):
            self.w(a, v)

    def program(self, entry, stream):
        self.w(0x12C, 0)
        self.w(0x10C, entry & 255)
        self.w(0x10D, entry >> 8)
        for b in stream:
            self.w(0x128, b)

    def save(self, path):
        open(path, "w").write("\n".join(self.lines) + "\n")
        return path


def marks(out):
    return [int(line.split("T ")[-1]) for line in out.splitlines() if line.startswith("T ") or ": T " in line]


LIMIT = "never went idle"       # (a wait of more than 1,000,000 polls: the bench gives up, and so does unit_script)


def verdict(out):
    if LIMIT in out:
        return "stopped at a wait longer than the bench allows (a program of about a million commands)"
    return next((line.split(": ", 1)[-1] if "ps+" in line else line for line in out.splitlines()
                 if "PASS" in line or "FAIL" in line), "no verdict")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    sim = "--sim" in sys.argv
    if not args or set(a for a in sys.argv[1:] if a.startswith("--")) - {"--sim"}:
        sys.exit(__doc__)
    mcp = os.path.abspath(args[0])
    seeds = args[1:] or ["1", "2", "3"]
    ncmd = os.environ.get("NCMD", "3000")
    src, sf = os.path.join(mcp, "src"), os.path.join(mcp, "src", "mcp", "ext", "berkeley-softfloat-3")
    lib = os.path.join(sf, "build", "Linux-ARM-VFPv2-GCC", "softfloat.a")
    if not os.path.exists(lib):
        sys.exit(f"no {lib}: run src/mcp/sim/fetch.sh in the M65-MCP's repository first")
    tmp = tempfile.mkdtemp(prefix="mcp65-")
    unit, gen = os.path.join(tmp, "unit_script"), os.path.join(tmp, "mcpgen")
    up = os.path.join(HERE, "..")
    sh("cc", "-std=gnu11", "-O2", "-Wall", "-o", unit, os.path.join(HERE, "unit_script.c"),
       *[os.path.join(up, f) for f in ("mcp65_unit.c", "mcp65_model.c", "mcp65_softfloat.c")])
    model = os.path.join(src, "mcp", "model")
    sh("cc", "-std=gnu99", "-O2", "-I", os.path.join(sf, "source", "include"), "-I", model, "-o", gen,
       os.path.join(model, "mcpgen.c"), os.path.join(model, "mcp_model.c"), lib)

    def hardware(script):
        d = os.path.join(tmp, "sim")
        os.makedirs(d, exist_ok=True)
        vhdl = os.path.join(src, "vhdl")
        for step in (("-a", *[os.path.join(vhdl, f) for f in ("mcp_ram.vhdl", "mcp_fpu.vhdl", "mcp.vhdl")],
                      os.path.join(src, "mcp", "sim", "tb_mcp.vhdl")),
                     ("-e", "tb_mcp", f"-gSCRIPT={script}", "-gFASTBUS=1")):
            sh("nvc", "--std=2008", *step, cwd=d)
        return sh("nvc", "--std=2008", "-r", "tb_mcp", "--exit-severity=failure", "--ieee-warnings=off", cwd=d, ok=True)

    failed = False

    if sim:
        # ---- programs: clocks a pass
        sys.path.insert(0, os.path.join(src, "mcp", "demo", "vtime"))
        import vtime                                        # (its list of every command)
        progs = [("PEND alone", 0x1F, b"")]
        for t in vtime.TESTS:
            vf = t[2] if len(t) > 2 else 0x1F
            progs += [(t[0], vf, t[1]), (t[0] + ", four in a row", vf, t[1] * 4)]
        r, m, w = reg, mem, wide
        progs += [(n, 0x1F, p) for n, p in (
            ("four FADD3 on different registers", w(0, 1, 2, 3) + w(0, 4, 5, 6) + w(0, 7, 8, 9) + w(0, 10, 11, 12)),
            ("four FDIV on different registers", r(5, 1, 2) + r(5, 3, 4) + r(5, 5, 6) + r(5, 7, 8)),
            ("FDIV, four FADD3 elsewhere", r(5, 1, 2) + w(0, 3, 4, 5) + w(0, 6, 7, 8) + w(0, 9, 10, 11) + w(0, 12, 13, 14)),
            ("FTOI, FTOI", r(0x10, 0, 1) + r(0x10, 0, 2)),
            ("FCMP, FCMP", r(7, 1, 2) + r(7, 3, 4)),
            ("FCMP, PSKIP taken over 2", r(7, 1, 2) + w(0x28, 0, 7, 2) + r(1, 3, 4) + r(1, 5, 6)),
            ("FCMP, PSKIP not taken", r(7, 1, 2) + w(0x28, 0, 8, 2) + r(1, 3, 4) + r(1, 5, 6)),
            ("FADD, FMUL on it, FSQRT on it", r(1, 1, 2) + r(4, 3, 1) + r(0x0A, 4, 3)),
            ("FDIV, VCLIP V0", r(5, 1, 2) + r(0x1C, 0, 0)),
            ("FADD F1, MLOAD", r(1, 1, 2) + w(0x20, 0, 0, 0)),
            ("FADD F1, MSTORE, FADD F1", r(1, 1, 2) + w(0x21, 0, 0, 0) + r(1, 1, 2)),
            ("VOUT, SETFLAGS", w(0x26, 2, 1, 0) + r(0x17, 0, 0)),
            ("VOUT three times, VEND", w(0x26, 2, 1, 0) + w(0x26, 2, 2, 1) + w(0x26, 3, 3, 2) + r(0x1E, 0, 0)),
            ("FDIV F1, VNORM3 V2", r(5, 1, 2) + w(0x18, 2, 2, 0)),
            ("MXV V0, VDOT3 F5, V0, V2", w(0x22, 0, 1, 0) + w(0x15, 5, 0, 2)),
            ("FMUL F4, MXV V1", r(4, 4, 2) + w(0x22, 1, 1, 0)),
            ("FXCHG F1, F2, FADD F1", r(0x14, 1, 2) + r(1, 1, 3)),
            ("FDIV by 0", m(5, 1, 0)),
            ("FDIV by 0, FDIV F3, F4", m(5, 1, 0) + r(5, 3, 4)),
            ("FSQRT of -1", m(10, 1, -1.0)),
            ("FSQRT of 0, four in a row", m(10, 1, 0) * 4),
            ("FRECIP of infinity", m(11, 1, 0x7F800000)),
            ("FRINT of 2^24", m(0, 1, 16777216.0) + r(0x0D, 1, 1)),
            ("FRINT of 1.5", m(0, 1, 1.5) + r(0x0D, 2, 1)),
            ("FTOI of 0", m(0, 1, 0) + r(0x10, 0, 1)),
            ("FTOI of a NaN", m(0, 1, 0x7FC00000) + r(0x10, 0, 1)),
            ("FTOI of infinity", m(0, 1, 0x7F800000) + r(0x10, 0, 1)),
            ("FILD of 0", m(12, 1, 0)),
            ("FXLD of 0, four in a row", m(13, 1, 0) * 4),
            ("VNORM3 of a zero vector", m(0, 4, 0) + m(0, 5, 0) + m(0, 6, 0) + w(0x18, 1, 1, 0)),
            ("VNORM3 of an infinite vector", m(0, 4, 0x7F800000) + w(0x18, 1, 1, 0)),
            ("VNORM3 of a subnormal vector", m(0, 4, 0x00000100) + m(0, 5, 0) + m(0, 6, 0) + w(0x18, 1, 1, 0)),
            ("FDIVR, 0 over F1", m(6, 1, 0)),
            ("FDIV, a subnormal by 3", m(0, 1, 0x00000100) + m(5, 1, 3.0)),
            ("FDIV F1, F3, VNORM3 V2 (the divider taken)", r(5, 1, 3) + w(0x18, 2, 2, 2)),
            ("FDIV F1, F3, FMOVE, VNORM3 V2", r(5, 1, 3) + r(0, 5, 6) + w(0x18, 2, 2, 2)),
            ("FDIV F1, F3, 2 FMOVE, VNORM3 V2", r(5, 1, 3) + r(0, 5, 6) * 2 + w(0x18, 2, 2, 2)),
            ("FDIV F1, F3, 3 FMOVE, VNORM3 V2", r(5, 1, 3) + r(0, 5, 6) * 3 + w(0x18, 2, 2, 2)),
            ("FTOI F3, VNORM3 V2", r(0x10, 0, 3) + w(0x18, 2, 2, 2)),
            ("FDIV F1, F3, VNORM3 of a zero vector", m(0, 8, 0) + m(0, 9, 0) + m(0, 10, 0) + r(5, 1, 3) + w(0x18, 2, 2, 2)),
        )]
        s = Script()
        s.w(0x107, 0)
        for name, vfmt, p in progs:
            s.registers(vfmt)
            s.program(0, p + b"\x19\x00")
            s.add("I")
            for k in range(4):
                s.w(0x780 + k, (PASSES << 16) >> (8 * k))
            s.add("T", "I", "T")
        script = s.save(os.path.join(tmp, "programs.txt"))
        hw, un = marks(hardware(script)), marks(sh(unit, script, ok=True))
        assert len(hw) == len(un) == 2 * len(progs), (len(hw), len(un))
        bad = 0
        for k, (name, _, _) in enumerate(progs):
            h, u = (hw[2 * k + 1] - hw[2 * k]) / PASSES, (un[2 * k + 1] - un[2 * k]) / PASSES
            if abs(h - u) > 0.0101:     # (a poll of MSTATUS is three clocks: 0.003 a pass)
                print(f"  {name}: the hardware {h:.2f} clocks a pass, the unit {u:.2f}")
                bad += 1
        print(f"programs: {len(progs) - bad} of {len(progs)} take the hardware's clocks a pass")
        failed |= bad > 0

        # ---- idle: the clock MSTATUS first says so in
        cmds = [(t[0], bus(t[1]), t[2] if len(t) > 2 else 0x1F) for t in vtime.TESTS]
        cmds += [(n, ws, 0x1F) for n, ws in (
            ("a reserved register form", bus(r(0x1D, 0, 0))),
            ("VPASS without attribute words", [(0x008, 0x07)] + bus(r(0x1B, 0, 0))),
            ("PRUN, 1 pass of PEND", bus(m(14, 0, 1 << 16 | 100))),
            ("PRUN, 3 passes of PEND", bus(m(14, 0, 3 << 16 | 100))),
            ("PRUN, 2 passes of FADD, PEND", bus(m(14, 0, 2 << 16 | 110))),
            ("PRUN past the last entry", bus(m(14, 0, 1 << 16 | 510))),
            ("VOUT, SETFLAGS", bus(w(0x26, 2, 1, 0) + r(0x17, 0, 0))),
            ("VCLIP, SETFLAGS", bus(r(0x1C, 0, 1) + r(0x17, 0, 0))),
            ("FDIV, FADD elsewhere", bus(r(5, 1, 2) + r(1, 3, 4))),
            ("FADD, FADD on it", bus(r(1, 1, 2) * 2)),
            ("FADD by the stream", [(0x120, b) for b in r(1, 1, 2)]),
            ("VPASS by the stream", [(0x120, b) for b in r(0x1B, 0, 0)]),
            ("VOUTC by the stream", [(0x120, b) for b in w(0x27, 0, 1, 0)]),
            ("VCLIP, FTOI16, FXCHG", bus(r(0x1C, 6, 8) + r(0x12, 12, 7) + r(0x14, 3, 14))),
            ("VOUTC, MPABORT with no program", bus(w(0x27, 14, 0, 0)) + [(0x12D, 0)]),
        )]
        delays = [0] + list(range(2, 16)) + [20, 24, 30, 40, 60, 66, 70, 75, 80]
        s = Script()
        s.w(0x107, 0)
        s.program(100, r(0x19, 0, 0))
        s.program(110, r(1, 1, 2) + r(0x19, 0, 0))
        s.program(510, r(1, 1, 2) + r(1, 3, 4))
        for name, ws, vfmt in cmds:
            for dl in delays:
                s.registers(vfmt)
                s.add("I", "T")
                for a, v in ws:
                    s.w(a, v)
                nr = dl % 2                             # accesses that do nothing: a read is 3 clocks, a write 2
                s.add(*["R 300 00"] * nr)
                for _ in range((dl - 3 * nr) // 2):
                    s.w(0x300, 0)
                s.add("I", "T")
        script = s.save(os.path.join(tmp, "idle.txt"))
        hw, un = marks(hardware(script)), marks(sh(unit, script, ok=True))
        assert len(hw) == len(un) == 2 * len(cmds) * len(delays), (len(hw), len(un))
        bad = 0
        for i, (name, _, _) in enumerate(cmds):
            at = [2 * (i * len(delays) + j) for j in range(len(delays))]
            diff = [f"{delays[j]}: {(un[a + 1] - un[a]) - (hw[a + 1] - hw[a]):+d}" for j, a in enumerate(at)
                    if un[a + 1] - un[a] != hw[a + 1] - hw[a]]
            if diff:
                print(f"  {name}: idle not in the hardware's clock (delay: clocks apart) {', '.join(diff)}")
                bad += 1
        print(f"idle: after {len(cmds) - bad} of {len(cmds)} commands MSTATUS says so in the hardware's clock, "
              f"at each of {len(delays)} delays")
        failed |= bad > 0

    # ---- random scripts: every read, and (--sim) the clock at every wait's end
    for seed in seeds:
        body = sh(gen, seed, ncmd, "5", ok=True).splitlines()
        body = [line for line in body if line and line[0] in "WRIOA"]
        lines = ["T"]
        for line in body:
            lines.append(line)
            if line == "I":
                lines.append("T")
        script = os.path.join(tmp, f"random{seed}.txt")
        open(script, "w").write("\n".join(lines) + "\nT\n")
        u = sh(unit, script, ok=True)
        text = f"random script, seed {seed}, {ncmd} commands: the unit: {verdict(u)}"
        failed |= "PASS" not in u and LIMIT not in u
        if sim:
            h = hardware(script)
            hw, un = marks(h), marks(u)
            same = sum(1 for a, b in zip(hw, un) if a - hw[0] == b - un[0])
            text += (f"; the hardware: {verdict(h)}; {same} of {len(hw)} waits end in the same clock "
                     f"({hw[-1] - hw[0]} clocks in all, the unit {un[-1] - un[0]})")
            failed |= same != len(hw) or len(hw) != len(un) or ("PASS" not in h and LIMIT not in h) or (LIMIT in h) != (LIMIT in u)
        print(text)
    print("FAIL" if failed else "PASS")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
