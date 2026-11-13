/* unit_script: Xemu's M65-MCP unit (../mcp65_unit.c) alone, driven by a bus
 * script of the kind the hardware's own test bench runs (tb_mcp.vhdl in the
 * M65-MCP's repository, scripts from its mcpgen): every read compared with
 * the value the script expects (the reference model's), and the clock
 * printed where the script asks, so the unit's timing can be laid beside the
 * hardware's simulation (test.sh does that).
 *
 *   unit_script script.txt
 *
 * Script lines (hex, register offsets from $FFDF000):
 *   W aaa vv   write byte vv to aaa
 *   R aaa vv   read aaa, expect vv
 *   I          wait until MSTATUS bit 0 (busy) is clear
 *   T          print the clock
 *   O          the overrun test: 40 divides back to back fill the queue
 *   A          the MPABORT test: a program of 65,536 passes stopped
 *
 * The bus is the bench's at its fastest (FASTBUS: what a DMA job does): a
 * write every two clocks, a read every three. */
#include <stdio.h>
#include <stdlib.h>
#include "../mcp65_unit.h"

static uint64_t clk = 4;
static long errors, nw, nr, ni;

static void bus_write(unsigned a, unsigned v)
{
    mcp65_unit_write(clk + 1, a, (uint8_t)v);
    clk += 2;
}

static unsigned bus_read(unsigned a)
{
    unsigned v = mcp65_unit_read(clk + 1, a);
    clk += 3;
    return v;
}

static void wait_idle(void)
{
    long k = 0;
    while (bus_read(0x100) & 1)
        if (++k > 1000000) {
            printf("the unit never went idle\n");
            exit(1);
        }
}

static void fail(const char *what, unsigned v)
{
    printf("%s (%02X)\n", what, v);
    errors++;
}

int main(int argc, char **argv)
{
    FILE *f = argc == 2 ? fopen(argv[1], "r") : NULL;
    char line[64];
    unsigned a, v, got;
    if (!f) {
        fprintf(stderr, "usage: unit_script script.txt\n");
        return 2;
    }
    mcp65_unit_init();
    while (fgets(line, sizeof line, f)) {
        switch (line[0]) {
        case 'W':
            if (sscanf(line + 1, "%x %x", &a, &v) == 2) {
                bus_write(a, v);
                nw++;
            }
            break;
        case 'R':
            if (sscanf(line + 1, "%x %x", &a, &v) == 2) {
                got = bus_read(a);
                nr++;
                if (got != v) {
                    if (errors < 25)
                        printf("read $%03X: %02X, expected %02X (read %ld)\n", a, got, v, nr);
                    errors++;
                }
            }
            break;
        case 'I':
            wait_idle();
            ni++;
            break;
        case 'T':
            printf("T %llu\n", (unsigned long long)clk);
            break;
        case 'O':
            bus_write(0x107, 0);                            /* MRESET */
            for (int i = 0; i < 40; i++)
                bus_write(0x185, (unsigned)(i * 37) % 256);  /* FDIV */
            v = bus_read(0x103);
            if (v > 16 || v < 14) fail("overrun test: MQLEN not about 16", v);
            wait_idle();
            v = bus_read(0x100);
            if (!(v & 4)) fail("overrun test: MSTATUS bit 2 clear", v);
            bus_write(0x100, 0);
            v = bus_read(0x100);
            if (v & 0x0C) fail("overrun test: MSTATUS after the write", v);
            bus_write(0x107, 0);
            break;
        case 'A':
            bus_write(0x107, 0);                            /* MRESET */
            bus_write(0x10C, 0);                            /* MPPTR 0 */
            bus_write(0x10D, 0);
            bus_write(0x128, 0x01);                         /* FADD F0, F0 */
            bus_write(0x129, 0x00);
            bus_write(0x12A, 0x19);                         /* PEND */
            bus_write(0x12B, 0x00);
            for (a = 0x114; a <= 0x116; a++)                /* MPLIMIT 0: no limit */
                bus_write(a, 0);
            for (a = 0; a < 4; a++)                         /* PRUN 0, 65,536 passes */
                bus_write(0x780 + a, 0);
            clk += 300;
            v = bus_read(0x100);
            if ((v & 0x11) != 0x11) fail("MPABORT test: MSTATUS while the program runs", v);
            bus_write(0x12D, 0);                            /* MPABORT */
            wait_idle();
            v = bus_read(0x100);
            if ((v & 0x30) != 0x20) fail("MPABORT test: MSTATUS after the abort", v);
            v = bus_read(0x110);
            if (v != 3) fail("MPABORT test: MPERR not 3", v);
            v = bus_read(0x10E);
            if (!v) fail("MPABORT test: no passes counted", v);
            bus_write(0x107, 0);
            break;
        default:
            break;
        }
    }
    fclose(f);
    printf("%ld writes, %ld reads compared, %ld waits, %llu clocks, %u commands dropped by a full queue\n",
           nw, nr, ni, (unsigned long long)clk, mcp65_unit_overruns());
    printf(errors ? "FAIL: %ld differences\n" : "PASS: the unit identical to the model\n", errors);
    return errors != 0;
}
