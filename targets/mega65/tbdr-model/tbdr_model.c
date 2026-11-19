/*
 * tbdr_model - bit-exact reference model of the MEGA65 tile renderer: M1,
 * the GrVertex-shaped 64-byte records (M3) and on-chip textures with point
 * sampling and bilinear filtering (M4a). Every width and rounding here is the
 * hardware's; see README.md "Arithmetic".
 */
#include "tbdr_model.h"

#include <assert.h>
#include <string.h>

#define M34 ((1ULL << 34) - 1)
#define M25 ((1ULL << 25) - 1)

typedef struct {
    int      valid;
    int64_t  a[3], b[3], c[3];   /* edges: E = a*px + b*py + c, inside if all >= 0 */
    int32_t  x0, y0;             /* vertex 0 after orientation, 12.4 */
    uint32_t z0;
    uint32_t l0;
    uint64_t zx, zy;             /* depth per subpixel, 16 fraction bits, mod 2^34 */
    uint32_t lx, ly;             /* light per subpixel, 15 fraction bits, mod 2^25 */
    int      bx0, by0, bx1, by1; /* pixel bounding box, inclusive, on screen */
    uint8_t  colour;
    uint8_t  zmode;              /* M5a: depth function (bits 0-2), no write (bit 3) */
    /* texture planes (64-byte records, textured), around vertex 0 */
    int      tex, bilinear;      /* texture number (-1: flat colour), bilinear */
    int      fog;                /* M5b */
    int      punch;              /* M5c */
    int      trans;              /* M6 */
    uint8_t  bmode, calpha;
    uint8_t  aref, afunc;
    int      m7;                 /* M7: the colour from the combine unit */
    uint8_t  state;              /* M7: the state table entry */
    uint32_t c0[3];              /* M7: G, B, A at vertex 0 (R is the light plane) */
    uint32_t cx[3], cy[3];       /* M7: their gradients, as the light's */
    uint32_t q0;                 /* oow at vertex 0 */
    uint64_t qx, qy;             /* oow per subpixel, 16 fraction bits, mod 2^34 */
    int32_t  a0, b0;             /* s*oow, t*oow at vertex 0 (s, t in 12.4) */
    uint64_t sx, sy, tx, ty;     /* s*oow, t*oow per subpixel, 8 fraction bits, mod 2^42 */
    int      lev;                /* the mip level it samples (record byte 50 bit 4: chosen here) */
    tbdr_tex dl;                 /* its descriptor: the level's base and size (TBDR_TEX_LEVELS) */
} tri_t;

static const uint8_t bayer4[4][4] = {
    { 0,  8,  2, 10},
    {12,  4, 14,  6},
    { 3, 11,  1,  9},
    {15,  7, 13,  5},
};

static int32_t rd_s16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

/* M5a: the depth test, the new depth z against the stored one */
static int zpass(int f, uint32_t z, uint32_t zs)
{
    switch (f) {
    case TBDR_Z_LESS:     return z < zs;
    case TBDR_Z_LEQUAL:   return z <= zs;
    case TBDR_Z_EQUAL:    return z == zs;
    case TBDR_Z_GREATER:  return z > zs;
    case TBDR_Z_GEQUAL:   return z >= zs;
    case TBDR_Z_NOTEQUAL: return z != zs;
    case TBDR_Z_ALWAYS:   return 1;
    default:              return 0;
    }
}
static uint32_t rd_u16(const uint8_t *p) { return p[0] | (p[1] << 8); }

static int64_t sext(uint64_t v, int bits)
{
    uint64_t m = 1ULL << (bits - 1);
    v &= (m << 1) - 1;
    return (int64_t)(v ^ m) - (int64_t)m;
}

/* floor(a / 16) and ceil(a / 16) for signed a (arithmetic shifts). */
static int32_t fdiv16(int32_t a) { return a >> 4; }
static int32_t cdiv16(int32_t a) { return -((-a) >> 4); }

/* The hardware's divider: |n| / d by restoring division, sign applied after,
 * i.e. truncation toward zero, kept modulo 2^bits. d > 0. */
static uint64_t qdiv(int64_t n, int64_t d, int bits)
{
    int64_t q = n / d;   /* C truncates toward zero */
    return (uint64_t)q & ((1ULL << bits) - 1);
}

static int flog2_64(uint64_t v)                /* the leading one's place, v != 0 */
{
    int k = 0;
    while (v >>= 1)
        k++;
    return k;
}

static void setup(const uint8_t *rec, const tbdr_regs *r, tri_t *t)
{
    int32_t x[3], y[3];
    uint32_t z[3], l[3], cv[3][3] = {{0}};
    int64_t qv[3] = {0, 0, 0}, sv[3] = {0, 0, 0}, tv[3] = {0, 0, 0};
    const int W = r->cols * 8, H = r->rows * 8;
    const int stride = r->rec64 ? 16 : 8;

    memset(t, 0, sizeof *t);
    for (int i = 0; i < 3; i++) {
        const uint8_t *v = rec + stride * i;
        x[i] = rd_s16(v);
        y[i] = rd_s16(v + 2);
        z[i] = rd_u16(v + 4);
        if (r->rec64) {
            qv[i] = rd_u16(v + 6);
            sv[i] = rd_s16(v + 8);
            tv[i] = rd_s16(v + 10);
            l[i] = v[12];                         /* r: the light (M7: red) */
            for (int j = 0; j < 3; j++)
                cv[j][i] = v[13 + j];             /* M7: g, b, a */
        } else {
            l[i] = v[6];
        }
    }
    const uint8_t flags = r->rec64 ? rec[49] : rec[15];
    t->colour = r->rec64 ? rec[48] : rec[7];
    t->tex = (r->rec64 && (flags & TBDR_F_TEX)) ? (rec[50] & 15) : -1;
    t->bilinear = (flags & TBDR_F_BILINEAR) != 0;
    t->zmode = r->rec64 ? rec[51] & 15 : 0;
    t->fog = r->rec64 && (flags & TBDR_F_FOG) && r->fogtab;
    t->punch = r->rec64 && (flags & TBDR_F_PUNCH);
    t->trans = r->rec64 && (flags & TBDR_F_TRANS);
    t->bmode = r->rec64 ? (rec[61] >> 4) & 3 : 0;
    t->calpha = r->rec64 ? rec[62] : 0;
    t->aref = r->rec64 ? rec[60] : 0;
    t->afunc = r->rec64 ? rec[61] & 7 : 0;
    t->m7 = r->rec64 && (flags & TBDR_F_M7);
    t->state = r->rec64 ? rec[63] & 63 : 0;

    /* Twice the signed area, 12.4 * 12.4 units. Positive = clockwise on screen. */
    int64_t area2 = (int64_t)(x[1] - x[0]) * (y[2] - y[0]) - (int64_t)(x[2] - x[0]) * (y[1] - y[0]);
    if (area2 == 0)
        return;
    if (area2 > 0 && r->rec64 && (flags & TBDR_F_CULLCW))
        return;
    if (area2 < 0) {
        if (flags & TBDR_F_CULL)
            return;
        int64_t dq = qv[1]; qv[1] = qv[2]; qv[2] = dq;
        int64_t ds = sv[1]; sv[1] = sv[2]; sv[2] = ds;
        int64_t dt = tv[1]; tv[1] = tv[2]; tv[2] = dt;
        int32_t tx = x[1]; x[1] = x[2]; x[2] = tx;
        int32_t ty = y[1]; y[1] = y[2]; y[2] = ty;
        uint32_t tz = z[1]; z[1] = z[2]; z[2] = tz;
        uint32_t tl = l[1]; l[1] = l[2]; l[2] = tl;
        for (int j = 0; j < 3; j++) {
            uint32_t tc = cv[j][1]; cv[j][1] = cv[j][2]; cv[j][2] = tc;
        }
        area2 = -area2;
    }

    /* Bounding box of the pixel centres (16*px + 8) inside the vertex box. */
    int32_t xmin = x[0], xmax = x[0], ymin = y[0], ymax = y[0];
    for (int i = 1; i < 3; i++) {
        if (x[i] < xmin) xmin = x[i];
        if (x[i] > xmax) xmax = x[i];
        if (y[i] < ymin) ymin = y[i];
        if (y[i] > ymax) ymax = y[i];
    }
    t->bx0 = cdiv16(xmin - 8); t->bx1 = fdiv16(xmax - 8);
    t->by0 = cdiv16(ymin - 8); t->by1 = fdiv16(ymax - 8);
    if (t->bx0 < 0) t->bx0 = 0;
    if (t->by0 < 0) t->by0 = 0;
    if (t->bx1 > W - 1) t->bx1 = W - 1;
    if (t->by1 > H - 1) t->by1 = H - 1;
    if (r->rec64) {                         /* M5a: the clip window */
        int cx0 = rd_u16(rec + 52), cx1 = rd_u16(rec + 54);
        int cy0 = rd_u16(rec + 56), cy1 = rd_u16(rec + 58);
        if (cx1 != 0) {
            if (t->bx0 < cx0) t->bx0 = cx0;
            if (t->bx1 > cx1 - 1) t->bx1 = cx1 - 1;
            if (t->by0 < cy0) t->by0 = cy0;
            if (t->by1 > cy1 - 1) t->by1 = cy1 - 1;
        }
    }
    if (t->bx0 > t->bx1 || t->by0 > t->by1)
        return;

    /* Edges i -> j: E(X,Y) = (yi-yj)X + (xj-xi)Y + xi*yj - xj*yi at the pixel
     * centre X = 16px+8, Y = 16py+8. Top-left rule: an edge that is neither a
     * left edge (a > 0) nor a top edge (a == 0, b > 0) owns no pixel centre
     * that lies exactly on it, hence c - 1. */
    for (int k = 0; k < 3; k++) {
        int i = k, j = (k + 1) % 3;
        int64_t dy = y[i] - y[j], dx = x[j] - x[i];
        t->a[k] = 16 * dy;
        t->b[k] = 16 * dx;
        t->c[k] = 8 * dy + 8 * dx + (int64_t)x[i] * y[j] - (int64_t)x[j] * y[i];
        if (!(t->a[k] > 0 || (t->a[k] == 0 && t->b[k] > 0)))
            t->c[k] -= 1;
    }

    /* Depth and light planes around vertex 0, gradients per subpixel. */
    int64_t dx1 = x[1] - x[0], dy1 = y[1] - y[0], dx2 = x[2] - x[0], dy2 = y[2] - y[0];
    int64_t dz1 = (int64_t)z[1] - z[0], dz2 = (int64_t)z[2] - z[0];
    int64_t dl1 = (int64_t)l[1] - l[0], dl2 = (int64_t)l[2] - l[0];
    int64_t nzx = dz1 * dy2 - dz2 * dy1, nzy = dz2 * dx1 - dz1 * dx2;
    int64_t nlx = dl1 * dy2 - dl2 * dy1, nly = dl2 * dx1 - dl1 * dx2;
    t->zx = qdiv(nzx * 65536, area2, 34);
    t->zy = qdiv(nzy * 65536, area2, 34);
    t->lx = (uint32_t)qdiv(nlx * 32768, area2, 25);
    t->ly = (uint32_t)qdiv(nly * 32768, area2, 25);
    t->x0 = x[0]; t->y0 = y[0];
    t->z0 = z[0]; t->l0 = l[0];
    if (t->m7)                              /* M7: G, B, A as the light plane */
        for (int j = 0; j < 3; j++) {
            int64_t dc1 = (int64_t)cv[j][1] - cv[j][0], dc2 = (int64_t)cv[j][2] - cv[j][0];
            t->cx[j] = (uint32_t)qdiv((dc1 * dy2 - dc2 * dy1) * 32768, area2, 25);
            t->cy[j] = (uint32_t)qdiv((dc2 * dx1 - dc1 * dx2) * 32768, area2, 25);
            t->c0[j] = cv[j][0];
        }
    if (t->tex >= 0 || t->fog) {
        /* oow as the depth plane is built (16 fraction bits, mod 2^34); s*oow
         * and t*oow (s, t in 12.4) with 8 fraction bits, mod 2^42 */
        int64_t a[3], b[3];
        for (int i = 0; i < 3; i++) {
            a[i] = sv[i] * qv[i];
            b[i] = tv[i] * qv[i];
        }
        int64_t dq1 = qv[1] - qv[0], dq2 = qv[2] - qv[0];
        int64_t da1 = a[1] - a[0], da2 = a[2] - a[0], db1 = b[1] - b[0], db2 = b[2] - b[0];
        t->qx = qdiv((dq1 * dy2 - dq2 * dy1) * 65536, area2, 34);
        t->qy = qdiv((dq2 * dx1 - dq1 * dx2) * 65536, area2, 34);
        t->sx = qdiv((da1 * dy2 - da2 * dy1) * 256, area2, 42);
        t->sy = qdiv((da2 * dx1 - da1 * dx2) * 256, area2, 42);
        t->tx = qdiv((db1 * dy2 - db2 * dy1) * 256, area2, 42);
        t->ty = qdiv((db2 * dx1 - db1 * dx2) * 256, area2, 42);
        t->q0 = (uint32_t)qv[0];
        t->a0 = (int32_t)a[0];
        t->b0 = (int32_t)b[0];
    }
    if (t->tex >= 0) {
        /* the mip level (2026-11-16, Slide's tri_lod moved here): half the
         * log2 of the triangle's area in texels over its area on screen, both
         * 12.4 x 12.4, from their leading ones; e <= 0: level 0, else
         * (e + 1) / 2, at most the texture's last level */
        t->dl = r->tex[t->tex];
        int nl = ((t->dl.fmt >> 4) & 7) + 1;
        if ((rec[50] & 0x10) && nl > 1) {
            int64_t as = (sv[1] - sv[0]) * (tv[2] - tv[0]) - (tv[1] - tv[0]) * (sv[2] - sv[0]);
            if (as < 0)
                as = -as;
            if (as) {
                int e = flog2_64((uint64_t)as) - flog2_64((uint64_t)area2);
                if (e > 0) {
                    int l = (e + 1) >> 1;
                    t->lev = l < nl - 1 ? l : nl - 1;
                }
            }
        }
        if (t->lev) {
            /* levels one after the other from the base: level i's quads
             * 2^(n - 2 - 2i), n = lw + lh - bc1 (Slide's lev_off) */
            int n = t->dl.lw + t->dl.lh - (t->dl.fmt & 1);
            for (int i = 0; i < t->lev; i++)
                if (n - 2 - 2 * i >= 0 && n - 2 - 2 * i <= 15)   /* (a level count the size allows) */
                    t->dl.base += 1u << (n - 2 - 2 * i);
            t->dl.base &= 0xFFFF;
            t->dl.lw -= t->lev;
            t->dl.lh -= t->lev;
        }
    }
    t->valid = 1;
}

static int inside(const tri_t *t, int px, int py)
{
    for (int k = 0; k < 3; k++)
        if (t->a[k] * px + t->b[k] * py + t->c[k] < 0)
            return 0;
    return 1;
}

static uint32_t depth_at(const tri_t *t, int px, int py)
{
    uint64_t dX = (uint64_t)(int64_t)(16 * px + 8 - t->x0);
    uint64_t dY = (uint64_t)(int64_t)(16 * py + 8 - t->y0);
    uint64_t z = (((uint64_t)t->z0 << 16) + t->zx * dX + t->zy * dY) & M34;
    int64_t s = sext(z, 34);
    if (s < 0) return 0;
    if (s > 0xFFFFFFFFLL) return 0xFFFFFFFFu;
    return (uint32_t)s;
}

/* the light at a pixel in sixteenths of a shade, 0..1023 */
static int light16_at(const tri_t *t, int px, int py)
{
    uint64_t dX = (uint64_t)(int64_t)(16 * px + 8 - t->x0);
    uint64_t dY = (uint64_t)(int64_t)(16 * py + 8 - t->y0);
    uint64_t l = (((uint64_t)t->l0 << 15) + (uint64_t)t->lx * dX + (uint64_t)t->ly * dY) & M25;
    int64_t s16 = sext(l, 25) >> 13;            /* shade in 1/16 steps */
    if (s16 < 0) s16 = 0;
    if (s16 > 1023) s16 = 1023;
    return (int)s16;
}

/* the colormap's shade: the light, dithered */
static int shade_of(int l16, int px, int py, int dither)
{
    int d = dither ? bayer4[py & 3][px & 3] : 8;
    int shade = (l16 + d) >> 4;
    return shade > 63 ? 63 : shade;
}

/* Texture fetch. Every texel read also goes through a model of a 4 KB
 * direct-mapped cache with 32-byte lines, counting misses for the SDRAM
 * texture estimate (M4b); the picture does not depend on it. */
#ifndef TEX_CACHE_SETS
#define TEX_CACHE_SETS 128
#endif
static uint32_t cache_tag[TEX_CACHE_SETS];
static int cache_ok[TEX_CACHE_SETS];

/* the texel at integer (u, v), wrapping, in the 2x2-quad layout */
static uint8_t fetch(const tbdr_tex *d, const tbdr_regs *r, int u, int v, long *misses)
{
    u &= (1 << d->lw) - 1;
    v &= (1 << d->lh) - 1;
    uint32_t addr = ((d->base << 2)
                     + ((((uint32_t)(v >> 1) << (d->lw - 1)) + (uint32_t)(u >> 1)) << 2)
                     + (uint32_t)((v & 1) << 1) + (uint32_t)(u & 1)) & (TBDR_TEXMEM - 1);
    uint32_t line = addr >> 5, set = line % TEX_CACHE_SETS;
    if (!cache_ok[set] || cache_tag[set] != line) {
        cache_ok[set] = 1;
        cache_tag[set] = line;
        (*misses)++;
    }
    return r->texmem[addr];
}

/* a texel as the RGB path sees it: the palette index (8-bit textures, the
 * colormap's input), its RGB and alpha (BC1: decoded, BC1.md) */
typedef struct {
    uint8_t c, rgb[3], a;
} tx_t;

static void tx_of_index(const tbdr_regs *r, uint8_t c, tx_t *o)
{
    o->c = c;
    for (int j = 0; j < 3; j++)
        o->rgb[j] = r->palrgb[3 * c + j];
    o->a = r->palalpha ? r->palalpha[c] : 255;
}

/* BC1 (BC1.md): block (bx, by) in bank (by & 1) * 2 + (bx & 1), word
 * base / 8 + ((by >> 1) << (lw - 3)) + (bx >> 1), its byte k at
 * 32 * word + 4 * k + bank */
static unsigned t3(unsigned a, unsigned b) { return ((2 * a + b) * 683 + 1024) >> 11; }

static void bc1_texel(const tbdr_tex *d, const tbdr_regs *r, int u, int v, long *misses, tx_t *o)
{
    u &= (1 << d->lw) - 1;
    v &= (1 << d->lh) - 1;
    int bx = u >> 2, by = v >> 2;
    uint32_t bank = (uint32_t)((by & 1) * 2 + (bx & 1));
    uint32_t word = (d->base >> 3) + ((uint32_t)(by >> 1) << (d->lw - 3)) + (uint32_t)(bx >> 1);
    uint8_t b[8];
    for (int k = 0; k < 8; k++)
        b[k] = r->texmem[(32 * word + 4 * (uint32_t)k + bank) & (TBDR_TEXMEM - 1)];
    uint32_t line = (32 * word) & (TBDR_TEXMEM - 1), set;
    line >>= 5;
    set = line % TEX_CACHE_SETS;
    if (!cache_ok[set] || cache_tag[set] != line) {
        cache_ok[set] = 1;
        cache_tag[set] = line;
        (*misses)++;
    }
    unsigned c0 = (unsigned)(b[0] | b[1] << 8), c1 = (unsigned)(b[2] | b[3] << 8);
    uint32_t bits = (uint32_t)b[4] | (uint32_t)b[5] << 8 | (uint32_t)b[6] << 16 | (uint32_t)b[7] << 24;
    unsigned idx = (bits >> (2 * (4 * (v & 3) + (u & 3)))) & 3;
    unsigned e[2][3];
    for (int k = 0; k < 2; k++) {
        unsigned c = k ? c1 : c0, r5 = c >> 11, g6 = (c >> 5) & 63, b5 = c & 31;
        e[k][0] = r5 << 3 | r5 >> 2;
        e[k][1] = g6 << 2 | g6 >> 4;
        e[k][2] = b5 << 3 | b5 >> 2;
    }
    o->c = 0;
    o->a = 255;
    for (int j = 0; j < 3; j++) {
        unsigned x;
        if (idx < 2)
            x = e[idx][j];
        else if (c0 > c1)
            x = idx == 2 ? t3(e[0][j], e[1][j]) : t3(e[1][j], e[0][j]);
        else
            x = idx == 2 ? (e[0][j] + e[1][j] + 1) >> 1 : 0;
        o->rgb[j] = (uint8_t)x;
    }
    if (c0 <= c1 && idx == 3)
        o->a = 0;
}

static void fetch_tx(const tbdr_tex *d, const tbdr_regs *r, int u, int v, long *misses, tx_t *o)
{
    if (d->fmt & 1)
        bc1_texel(d, r, u, v, misses, o);
    else
        tx_of_index(r, fetch(d, r, u, v, misses), o);
}

/* 1/q: rcp_tab[k] = 2^29 / (1024 + k), rounded (2^19 .. 2^18), interpolated */
static uint32_t rcp_tab[1025];

static void rcp_init(void)
{
    for (uint32_t k = 0; k <= 1024; k++)
        rcp_tab[k] = ((1u << 29) + (1024 + k) / 2) / (1024 + k);
}

static int clz32(uint32_t v)                  /* v != 0 */
{
    int n = 0;
    while (!(v & 0x80000000u)) {
        v <<= 1;
        n++;
    }
    return n;
}

/* the texture coordinates at pixel (px, py), 12.4 texels: q = clamp(oow plane,
 * 1, 2^32-1), normalised (lz leading zeros, m = q << lz); r = the table at
 * m's 10 bits under the leading one, interpolated by the 16 below them
 * (r ~ 2^50 / m); s = (s*oow plane * (r >> 2)) >> (40 - lz), t likewise */
static void tex_st(const tri_t *t, int px, int py, int64_t *s16, int64_t *t16)
{
    uint64_t dX = (uint64_t)(int64_t)(16 * px + 8 - t->x0);
    uint64_t dY = (uint64_t)(int64_t)(16 * py + 8 - t->y0);
    int64_t q = sext(((uint64_t)t->q0 << 16) + t->qx * dX + t->qy * dY, 34);
    uint32_t qc = q < 1 ? 1u : q > 0xFFFFFFFFLL ? 0xFFFFFFFFu : (uint32_t)q;
    int lz = clz32(qc);
    uint32_t m = qc << lz;
    uint32_t i = (m >> 21) & 1023, f = (m >> 5) & 0xFFFF;
    uint32_t rr = rcp_tab[i] - (((rcp_tab[i] - rcp_tab[i + 1]) * f) >> 16);
    int64_t r18 = rr >> 2;
    int64_t S = sext(((uint64_t)(int64_t)t->a0 << 8) + t->sx * dX + t->sy * dY, 42);
    int64_t T = sext(((uint64_t)(int64_t)t->b0 << 8) + t->tx * dX + t->ty * dY, 42);
    *s16 = (S * r18) >> (40 - lz + t->lev);      /* (the level: its texels, 2^lev of level 0's) */
    *t16 = (T * r18) >> (40 - lz + t->lev);
}

/* point sampling: the texel the coordinate falls in */
static void texel_point(const tri_t *t, const tbdr_regs *r, int px, int py, long *misses, tx_t *o);

/* M5c: the alpha at the pixel - bilinear's four texels weighted, point
 * sampling's texel, a flat triangle's colour, through the palette alpha */
static unsigned alpha_at(const tri_t *t, const tbdr_regs *r, int px, int py)
{
    long m = 0;
    const uint8_t *pa = r->palalpha;
    if (t->tex < 0)
        return pa ? pa[t->colour] : 255;
    if (!t->bilinear) {
        tx_t x;
        texel_point(t, r, px, py, &m, &x);
        return x.a;
    }
    int64_t s16, t16;
    tex_st(t, px, py, &s16, &t16);
    s16 -= 8;
    t16 -= 8;
    int u0 = (int)((s16 >> 4) & 0x7FFF), v0 = (int)((t16 >> 4) & 0x7FFF);
    int fu = (int)(s16 & 15), fv = (int)(t16 & 15);
    const int w[4] = {(16 - fu) * (16 - fv), fu * (16 - fv), (16 - fu) * fv, fu * fv};
    unsigned a = 0;
    for (int k = 0; k < 4; k++) {
        tx_t x;
        fetch_tx(&t->dl, r, u0 + (k & 1), v0 + (k >> 1), &m, &x);
        a += (unsigned)w[k] * x.a;
    }
    return a >> 8;
}

static void texel_point(const tri_t *t, const tbdr_regs *r, int px, int py, long *misses, tx_t *o)
{
    int64_t s16, t16;
    tex_st(t, px, py, &s16, &t16);
    fetch_tx(&t->dl, r, (int)((s16 >> 4) & 0x7FFF), (int)((t16 >> 4) & 0x7FFF), misses, o);
}

/* the RGB path: four palette entries weighted (weights sum to 256), times the
 * light curve at the light (lcurve / 1024), fogged (M5b, f = 0: unchanged), a
 * 0..7 ordered dither, RGB555, the inverse table */
/* the RGB path up to the light and fog: 9 bits a channel */
static void rgb_light(const tbdr_regs *r, int l16, const tx_t cs[4], const int w[4],
                      unsigned f, unsigned v[3])
{
    unsigned rgb[3] = {0, 0, 0};
    for (int k = 0; k < 4; k++)
        for (int j = 0; j < 3; j++)
            rgb[j] += (unsigned)w[k] * cs[k].rgb[j];
    for (int j = 0; j < 3; j++) {
        v[j] = ((rgb[j] >> 8) * r->lcurve[l16]) >> 10;
        v[j] = (v[j] * (256 - f) + r->fogrgb[j] * f) >> 8;
    }
}

/* ... then the ordered dither to RGB555 and the inverse colour table */
static uint8_t rgb_out(const tbdr_regs *r, int px, int py, const unsigned v[3])
{
    unsigned dd = bayer4[py & 3][px & 3] >> 1, c5[3];
    for (int j = 0; j < 3; j++) {
        unsigned u = v[j] + dd;
        c5[j] = u >> 3 > 31 ? 31 : u >> 3;
    }
    return r->inv[(c5[0] << 10) | (c5[1] << 5) | c5[2]];
}

/* the RGB path: four palette entries weighted (weights sum to 256), times the
 * light curve at the light (lcurve / 1024), fogged (M5b, f = 0: unchanged), a
 * 0..7 ordered dither, RGB555, the inverse table */
static uint8_t rgb_path(const tbdr_regs *r, int px, int py, int l16, const tx_t cs[4],
                        const int w[4], unsigned f)
{
    unsigned v[3];
    rgb_light(r, l16, cs, w, f, v);
    return rgb_out(r, px, py, v);
}

/* M5b: the fog factor at the pixel: the table at w's index, from q as the
 * texture coordinates clamp and normalise it */
static unsigned fog_at(const tri_t *t, const tbdr_regs *r, int px, int py)
{
    if (!t->fog)
        return 0;
    uint64_t dX = (uint64_t)(int64_t)(16 * px + 8 - t->x0);
    uint64_t dY = (uint64_t)(int64_t)(16 * py + 8 - t->y0);
    int64_t q = sext(((uint64_t)t->q0 << 16) + t->qx * dX + t->qy * dY, 34);
    uint32_t qc = q < 1 ? 1u : q > 0xFFFFFFFFLL ? 0xFFFFFFFFu : (uint32_t)q;
    int lz = clz32(qc);
    uint32_t m = qc << lz;
    int i = 4 * lz + 3 - (int)((m >> 29) & 3);
    return r->fogtab[i > 63 ? 63 : i];
}

/* M6: a translucent pixel's palette entries and weights - bilinear's four
 * texels, else four equal ones (the point texel or the flat colour) */
static void trans_texels(const tri_t *t, const tbdr_regs *r, int px, int py, tx_t cs[4],
                         int w[4])
{
    long m = 0;
    if (t->tex >= 0 && t->bilinear) {
        int64_t s16, t16;
        tex_st(t, px, py, &s16, &t16);
        s16 -= 8;
        t16 -= 8;
        int u0 = (int)((s16 >> 4) & 0x7FFF), v0 = (int)((t16 >> 4) & 0x7FFF);
        int fu = (int)(s16 & 15), fv = (int)(t16 & 15);
        w[0] = (16 - fu) * (16 - fv); w[1] = fu * (16 - fv);
        w[2] = (16 - fu) * fv;        w[3] = fu * fv;
        for (int k = 0; k < 4; k++)
            fetch_tx(&t->dl, r, u0 + (k & 1), v0 + (k >> 1), &m, &cs[k]);
        return;
    }
    tx_t c;
    if (t->tex >= 0)
        texel_point(t, r, px, py, &m, &c);
    else
        tx_of_index(r, t->colour, &c);
    for (int k = 0; k < 4; k++) {
        cs[k] = c;
        w[k] = k == 0 ? 256 : 0;
    }
}

/* M7: an iterated channel at the pixel (0 R = the light plane, 1 G, 2 B, 3 A),
 * 0..255 */
static unsigned iter_at(const tri_t *t, int k, int px, int py)
{
    uint64_t dX = (uint64_t)(int64_t)(16 * px + 8 - t->x0);
    uint64_t dY = (uint64_t)(int64_t)(16 * py + 8 - t->y0);
    uint32_t c0 = k ? t->c0[k - 1] : t->l0, gx = k ? t->cx[k - 1] : t->lx, gy = k ? t->cy[k - 1] : t->ly;
    int64_t v = sext(((uint64_t)c0 << 15) + (uint64_t)gx * dX + (uint64_t)gy * dY, 25) >> 15;
    return v < 0 ? 0 : v > 255 ? 255 : (unsigned)v;
}

/* M7: Glide's combine for one channel: clamp(f * (A - B) >> 8 + C), invert */
static unsigned combine1(unsigned word, unsigned local, unsigned other, unsigned lalpha,
                         unsigned oalpha, unsigned talpha)
{
    unsigned fs = (word >> 4) & 15, fn = word & 15, f;
    switch (fs & 7) {
    case 1:  f = local;  break;
    case 2:  f = oalpha; break;
    case 3:  f = lalpha; break;
    case 4:  f = talpha; break;
    default: f = 0;      break;
    }
    if (fs & 8)
        f = 255 - f;
    int f9 = (int)(f + (f >> 7));
    int a = 0, b = 0, c = 0;
    if (fn >= 3 && fn <= 8) a = (int)other;
    if (fn >= 6 && fn <= 10) b = (int)local;
    if (fn == 1 || fn == 4 || fn == 7 || fn == 9) c = (int)local;
    if (fn == 2 || fn == 5 || fn == 8 || fn == 10) c = (int)lalpha;
    int v = ((f9 * (a - b)) >> 8) + c;
    v = v < 0 ? 0 : v > 255 ? 255 : v;
    return (word >> 12) & 1 ? 255 - (unsigned)v : (unsigned)v;
}

/* M7a: the colour of an M7 triangle's pixel - the colour combine of the
 * iterated colour, the texture (palette RGB, bilinear weighted) and the
 * constant colour, then fog: 0..255 a channel */
static void m7_colour(const tri_t *t, const tbdr_regs *r, int px, int py, unsigned v[3],
                      unsigned *a)
{
    static const uint8_t zero[8];
    const uint8_t *st = r->state ? r->state + 8 * t->state : zero;
    unsigned cc = st[4] | (st[5] << 8), ac = st[6] | (st[7] << 8);
    unsigned it[4], tx[4], cn[4];
    tx_t cs[4];
    int w[4];
    trans_texels(t, r, px, py, cs, w);
    for (int j = 0; j < 3; j++) {
        unsigned s = 0;
        for (int k = 0; k < 4; k++)
            s += (unsigned)w[k] * cs[k].rgb[j];
        tx[j] = s >> 8;
    }
    tx[3] = alpha_at(t, r, px, py);
    for (int k = 0; k < 4; k++) {
        it[k] = iter_at(t, k, px, py);
        cn[k] = st[k];
    }
    const unsigned *lo = ((cc >> 8) & 3) == 1 ? cn : it;
    unsigned osel = (cc >> 10) & 3;
    const unsigned *ot = osel == 1 ? tx : osel == 2 ? cn : it;
    /* local alpha and other alpha are the ALPHA combine's choices (Voodoo's
     * alocalselect and aselect), for the colour factors too */
    unsigned la = ((ac >> 8) & 3) == 1 ? cn[3] : it[3];
    unsigned asel = (ac >> 10) & 3;
    unsigned oa = asel == 1 ? tx[3] : asel == 2 ? cn[3] : it[3];
    unsigned f = fog_at(t, r, px, py);
    for (int j = 0; j < 3; j++) {
        v[j] = combine1(cc, lo[j], ot[j], la, oa, tx[3]);
        v[j] = (v[j] * (256 - f) + r->fogrgb[j] * f) >> 8;
    }
    if (a)                              /* M7b: the alpha combine (no fog) */
        *a = combine1(ac, la, oa, la, oa, tx[3]);
}

/* M5c/M6/M7b: the alpha the alpha test and the translucency blend use - an
 * M7 triangle's alpha combine, otherwise the palette alpha */
static unsigned alpha_of(const tri_t *t, const tbdr_regs *r, int px, int py)
{
    if (t->m7) {
        unsigned v[3], a;
        m7_colour(t, r, px, py, v, &a);
        return a;
    }
    return alpha_at(t, r, px, py);
}

/* bilinear: the four texels around the coordinate (half a texel back), weighted
 * by the 4-bit fractions, through the RGB path */
static uint8_t texel_bilinear(const tri_t *t, const tbdr_regs *r, int px, int py, int l16,
                              long *misses)
{
    int64_t s16, t16;
    tex_st(t, px, py, &s16, &t16);
    s16 -= 8;
    t16 -= 8;
    int u0 = (int)((s16 >> 4) & 0x7FFF), v0 = (int)((t16 >> 4) & 0x7FFF);
    int fu = (int)(s16 & 15), fv = (int)(t16 & 15);
    const int w[4] = {(16 - fu) * (16 - fv), fu * (16 - fv), (16 - fu) * fv, fu * fv};
    const tbdr_tex *d = &t->dl;
    tx_t cs[4];
    for (int k = 0; k < 4; k++)
        fetch_tx(d, r, u0 + (k & 1), v0 + (k >> 1), misses, &cs[k]);
    return rgb_path(r, px, py, l16, cs, w, fog_at(t, r, px, py));
}

int tbdr_render(const uint8_t *triram, const uint8_t *colormap,
                const tbdr_regs *r, uint8_t *frame, tbdr_stats *st)
{
    static tri_t tri[TBDR_MAXTRI];
    static uint32_t zbuf[TBDR_TILE * TBDR_TILE];
    static uint16_t tag[TBDR_TILE * TBDR_TILE];
    tbdr_stats s;

    if (r->cols < 1 || r->cols > 80 || r->rows < 1 || r->rows > 50 ||
        r->count < 0 || r->count > TBDR_MAXTRI)
        return -1;
    memset(&s, 0, sizeof s);
    const int W = r->cols * 8, H = r->rows * 8;

    const int rec = r->rec64 ? TBDR_REC64 : TBDR_REC;
    const int miss_cycles = r->miss_cycles ? r->miss_cycles : 20;
    static long front[512], back[512], tmiss[512];
    int ntiles = 0;
    long setup_fast = 256;                                /* the tile buffers' first clear */
    memset(cache_ok, 0, sizeof cache_ok);
    rcp_init();
    for (int i = 0; i < r->count; i++) {
        setup(triram + rec * i, r, &tri[i]);
        s.tris_in++;
        s.tris_valid += tri[i].valid;
        setup_fast += tri[i].valid ? 46 : 12;             /* word loads, radix-4 dividers */
        if (r->rec64)                                     /* 13 words, not 6 */
            setup_fast += 7;
        if (tri[i].valid && tri[i].tex >= 0)              /* the 58-bit texture dividers */
            setup_fast += 4;
    }
    s.cycles += 4L * 52 * r->count + 24L * r->count;     /* setup: four divisions */

    for (int ty0 = 0; ty0 < H; ty0 += TBDR_TILE) {
        for (int tx0 = 0; tx0 < W; tx0 += TBDR_TILE) {
            int tx1 = tx0 + TBDR_TILE - 1, ty1 = ty0 + TBDR_TILE - 1;
            if (tx1 > W - 1) tx1 = W - 1;
            if (ty1 > H - 1) ty1 = H - 1;

            for (int i = 0; i < TBDR_TILE * TBDR_TILE; i++) {
                zbuf[i] = r->zclear;
                tag[i] = 0;
            }
            s.cycles += r->count;                          /* binner: one record a clock */
            long isp_fast = 0, hits = 0, misses = 0;

            /* ISP: hidden surface removal into the on-chip depth and tag
             * buffers - the opaque triangles, then (M5c) the punch-through
             * ones, whose pixels also need their alpha to pass */
            for (int pass = 0; pass < 2; pass++)
            for (int i = 0; i < r->count; i++) {
                const tri_t *t = &tri[i];
                if (!t->valid || t->bx1 < tx0 || t->bx0 > tx1 || t->by1 < ty0 || t->by0 > ty1)
                    continue;
                if (t->punch != pass || t->trans)
                    continue;
                int x0 = t->bx0 > tx0 ? t->bx0 : tx0, x1 = t->bx1 < tx1 ? t->bx1 : tx1;
                int y0 = t->by0 > ty0 ? t->by0 : ty0, y1 = t->by1 < ty1 ? t->by1 : ty1;
                s.pairs++;
                s.cycles += 8;
                hits++;
                isp_fast += 5 + (long)(y1 - y0 + 1) * ((x1 >> 2) - ((x0 & ~3) >> 2) + 1);
                for (int py = y0; py <= y1; py++)
                    for (int px = x0; px <= x1; px++) {
                        s.isp_tests++;
                        s.cycles++;
                        if (!inside(t, px, py))
                            continue;
                        uint32_t z = depth_at(t, px, py);
                        int o = (py - ty0) * TBDR_TILE + (px - tx0);
                        if (zpass(t->zmode & 7, z, zbuf[o])
                            && (!t->punch || zpass(t->afunc, alpha_of(t, r, px, py), t->aref))) {
                            if (!(t->zmode & TBDR_Z_NOWRITE))
                                zbuf[o] = z;
                            tag[o] = (uint16_t)(i + 1);
                            s.isp_writes++;
                        }
                    }
            }

            /* TSP: shade each pixel once, from the triangle its tag names. */
            for (int py = ty0; py <= ty1; py++)
                for (int px = tx0; px <= tx1; px++) {
                    int o = (py - ty0) * TBDR_TILE + (px - tx0);
                    uint8_t out = r->bg;
                    if (tag[o]) {
                        const tri_t *t = &tri[tag[o] - 1];
                        uint8_t c = t->colour;
                        int l16 = light16_at(t, px, py);
                        if (t->m7) {                  /* M7a: the combine unit */
                            unsigned v[3];
                            m7_colour(t, r, px, py, v, NULL);
                            out = rgb_out(r, px, py, v);
                            if (t->tex >= 0)
                                s.texels++;
                        } else if (t->tex >= 0 && t->bilinear) {
                            out = texel_bilinear(t, r, px, py, l16, &misses);
                            s.texels++;
                        } else {
                            tx_t x;
                            int bc1 = t->tex >= 0 && (t->dl.fmt & 1);
                            if (t->tex >= 0) {
                                texel_point(t, r, px, py, &misses, &x);
                                c = x.c;
                                s.texels++;
                            } else
                                tx_of_index(r, c, &x);
                            if (t->fog || bc1) {      /* M5b: the RGB path, four equal texels (BC1: always) */
                                const tx_t cs[4] = {x, x, x, x};
                                const int w[4] = {256, 0, 0, 0};
                                out = rgb_path(r, px, py, l16, cs, w, fog_at(t, r, px, py));
                            } else
                                out = colormap[shade_of(l16, px, py, r->dither) * 256 + c];
                        }
                        s.tsp_pixels++;
                    }
                    frame[tbdr_fcm_offset(r->cols, px, py)] = out;
                    s.cycles++;
                }

            /* M6: the translucent triangles, in submission order, blended
             * into the shaded tile */
            for (int i = 0; i < r->count; i++) {
                const tri_t *t = &tri[i];
                if (!t->valid || !t->trans || t->bx1 < tx0 || t->bx0 > tx1 || t->by1 < ty0 || t->by0 > ty1)
                    continue;
                int x0 = t->bx0 > tx0 ? t->bx0 : tx0, x1 = t->bx1 < tx1 ? t->bx1 : tx1;
                int y0 = t->by0 > ty0 ? t->by0 : ty0, y1 = t->by1 < ty1 ? t->by1 : ty1;
                s.pairs++;
                for (int py = y0; py <= y1; py++)
                    for (int px = x0; px <= x1; px++) {
                        if (!inside(t, px, py))
                            continue;
                        uint32_t z = depth_at(t, px, py);
                        int o = (py - ty0) * TBDR_TILE + (px - tx0);
                        if (!zpass(t->zmode & 7, z, zbuf[o]))
                            continue;
                        if (!(t->zmode & TBDR_Z_NOWRITE))
                            zbuf[o] = z;
                        tx_t cs[4];
                        int w[4];
                        unsigned src[3], v[3];
                        if (t->m7)
                            m7_colour(t, r, px, py, src, NULL);
                        else {
                            trans_texels(t, r, px, py, cs, w);
                            rgb_light(r, light16_at(t, px, py), cs, w, fog_at(t, r, px, py), src);
                        }
                        /* M7b: an M7 triangle's alpha is its alpha combine's, without byte 62 */
                        unsigned a = t->m7 ? alpha_of(t, r, px, py)
                                           : (alpha_at(t, r, px, py) * (t->calpha + 1u)) >> 8;
                        a += a >> 7;                                  /* 0..256 */
                        uint8_t *fp = &frame[tbdr_fcm_offset(r->cols, px, py)];
                        for (int j = 0; j < 3; j++) {
                            unsigned sj = src[j] > 255 ? 255 : src[j], dj = r->palrgb[3 * *fp + j];
                            if (t->bmode == 1) {
                                unsigned u = dj + ((sj * a) >> 8);
                                v[j] = u > 255 ? 255 : u;
                            } else if (t->bmode == 2)             /* multiply (Glide's DST_COLOR, ZERO) */
                                v[j] = (dj * (sj + 1)) >> 8;
                            else
                                v[j] = (sj * a + dj * (256 - a)) >> 8;
                        }
                        *fp = rgb_out(r, px, py, v);
                    }
            }
            /* tbdr-fast's tile: the front (binner 4/clk beside the ISP's
             * quads), the back (the TSP's pairs, stalled by cache misses) */
            long scan = (r->count + 3) / 4 + 2 * hits;
            front[ntiles] = (scan > isp_fast ? scan : isp_fast) + 6;
            back[ntiles] = ((long)(tx1 - tx0 + 1) * (ty1 - ty0 + 1) + 1) / 2 + 12;
            tmiss[ntiles] = misses;
            s.tex_misses += misses;
            ntiles++;
        }
    }
    /* the front of tile n+1 overlaps the back of tile n */
    for (int pass = 0; pass < 2; pass++) {
        long mc = pass ? miss_cycles : 0, f = setup_fast + (ntiles ? front[0] : 0);
        for (int i = 0; i < ntiles; i++) {
            long b = back[i] + tmiss[i] * mc, nf = i + 1 < ntiles ? front[i + 1] : 0;
            f += b > nf ? b : nf;
        }
        f += 38;                                          /* the TSP's 25 stages, BUSY */
        if (pass) s.fast = f; else s.fast_nomiss = f;
    }
    if (st)
        *st = s;
    return 0;
}

/* ---- M8: float vertices (FLOAT.md) ---- */

enum { FL_ZERO, FL_NUM, FL_INF, FL_NAN };

static int fl_unpack(const uint8_t *p, int *sign, int *ex, uint32_t *man)
{
    uint32_t b = p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24;
    int e = (b >> 23) & 0xFF;
    *sign = b >> 31;
    *man = (b & 0x7FFFFF) | 0x800000;
    *ex = e - 150;                       /* value = man * 2^ex */
    if (e == 0) return FL_ZERO;          /* zero and denormals */
    if (e == 255) return (b & 0x7FFFFF) ? FL_NAN : FL_INF;
    return FL_NUM;
}

static int64_t clampi(int64_t v, int64_t lo, int64_t hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* floor(v * 2^k + 1/2) - bias, clamped to lo..hi */
/* floor(v * 2^k + 1/2) - bias, then (flip) F minus that, clamped to [lo, hi];
 * zero, denormal and NaN count as 0, infinity as beyond either end */
static int64_t fl_fix(const uint8_t *p, int k, int64_t bias, int flip, int64_t F,
                      int64_t lo, int64_t hi)
{
    int sg, ex;
    uint32_t m;
    int c = fl_unpack(p, &sg, &ex, &m);
    int64_t r;
    if (c == FL_ZERO || c == FL_NAN)
        r = 0;
    else if (c == FL_INF)
        r = sg ? -(1LL << 54) : (1LL << 54);
    else {
        int sh = ex + k;
        if (sh >= 0)
            r = sh > 30 ? (sg ? -(1LL << 54) : (1LL << 54)) : (sg ? -((int64_t)m << sh) : ((int64_t)m << sh));
        else if (sh < -40)
            r = 0;
        else {
            uint64_t mm = (uint64_t)m << 1, h = 1ULL << (-sh);   /* 2v = mm / 2^-sh */
            /* floor(v + 1/2) = floor((2v + 1) / 2) = floor((mm + h) / (2h)); negative: floor((h - mm) / (2h)) */
            if (!sg)
                r = (int64_t)((mm + h) / (2 * h));
            else
                r = mm <= h ? 0 : -(int64_t)((mm - h + 2 * h - 1) / (2 * h));
        }
    }
    r -= bias;
    if (flip)
        r = F - r;
    return clampi(r, lo, hi);
}

/* floor(a / o * 2^k + 1/2), clamped to s16 */
static int fl_div(const uint8_t *pa, const uint8_t *po, int k)
{
    int sa, ea, so, eo;
    uint32_t ma, mo;
    int ca = fl_unpack(pa, &sa, &ea, &ma), co = fl_unpack(po, &so, &eo, &mo);
    if (co != FL_NUM || ca == FL_ZERO || ca == FL_NAN) return 0;
    int sg = sa ^ so;
    if (ca == FL_INF) return sg ? -32768 : 32767;
    int p = ea - eo + k;                 /* q = ma / mo * 2^p, ma/mo in (1/2, 2) */
    if (p >= 17) return sg ? -32768 : 32767;
    if (p <= -2) return 0;               /* |q| < 1/2 */
    /* 2|q| = N / D */
    uint64_t N = (uint64_t)ma << (p + 1 > 0 ? p + 1 : 0), D = (uint64_t)mo << (p + 1 < 0 ? -(p + 1) : 0);
    int64_t r;
    if (!sg)
        r = (int64_t)((N + D) / (2 * D));                 /* floor(|q| + 1/2) */
    else
        r = N <= D ? 0 : -(int64_t)((N - D + 2 * D - 1) / (2 * D));   /* -ceil(|q| - 1/2) */
    return (int)clampi(r, -32768, 32767);
}

static void put16le(uint8_t *p, int64_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

void tbdr_float_tri(const uint8_t *in, uint8_t *rec, const tbdr_tex *tex,
                    int oowexp, int mode)
{
    int snap = mode & 1, wdepth = mode >> 1 & 1, yflip = mode >> 2 & 1;
    memcpy(rec + 48, in, 16);
    const tbdr_tex *t = &tex[in[2] & 15];
    int L = t->lw > t->lh ? t->lw : t->lh;
    for (int i = 0; i < 3; i++) {
        const uint8_t *g = in + 16 + 48 * i;
        uint8_t *v = rec + 16 * i;
        int64_t bias = snap ? 3LL << 22 : 0;
        put16le(v, fl_fix(g + 0, 4, bias, 0, 0, -32768, 32767)); /* x */
        put16le(v + 2, fl_fix(g + 4, 4, bias, yflip, TBDR_YFLIP16, -32768, 32767));   /* y */
        int64_t oow = fl_fix(g + 32, oowexp, 0, 0, 0, 0, 65535);
        put16le(v + 4, wdepth ? oow : fl_fix(g + 24, 0, 0, 0, 0, 0, 65535));   /* ooz */
        put16le(v + 6, oow);                                      /* oow */
        put16le(v + 8, fl_div(g + 36, g + 32, L - 4));           /* s */
        put16le(v + 10, fl_div(g + 40, g + 32, L - 4));          /* t */
        v[12] = (uint8_t)fl_fix(g + 12, 0, 0, 0, 0, 0, 255);     /* r */
        v[13] = (uint8_t)fl_fix(g + 16, 0, 0, 0, 0, 0, 255);     /* g */
        v[14] = (uint8_t)fl_fix(g + 20, 0, 0, 0, 0, 0, 255);     /* b */
        v[15] = (uint8_t)fl_fix(g + 28, 0, 0, 0, 0, 0, 255);     /* a */
    }
}

/* ---- M9c: indexed vertices (FLOAT.md) ---- */

/* T = floor(a / o * 16), saturated to s24 */
static int32_t fl_divT(const uint8_t *pa, const uint8_t *po)
{
    int sa, ea, so, eo;
    uint32_t ma, mo;
    int ca = fl_unpack(pa, &sa, &ea, &ma), co = fl_unpack(po, &so, &eo, &mo);
    if (co != FL_NUM || ca == FL_ZERO || ca == FL_NAN) return 0;
    int sg = sa ^ so;
    const int32_t hi = (1 << 23) - 1, lo = -(1 << 23);
    if (ca == FL_INF) return sg ? lo : hi;
    int p = ea - eo + 4;                 /* |Y| = ma / mo * 2^p, ma/mo in (1/2, 2) */
    if (p >= 24) return sg ? lo : hi;    /* |Y| > 2^23 */
    if (p < -30) return sg ? -1 : 0;     /* 0 < |Y| < 1 */
    uint64_t N = (uint64_t)ma << (p > 0 ? p : 0), D = (uint64_t)mo << (p < 0 ? -p : 0);
    int64_t r = sg ? -(int64_t)((N + D - 1) / D) : (int64_t)(N / D);   /* floor */
    return (int32_t)clampi(r, lo, hi);
}

static void put24le(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
}

static int32_t get24le(const uint8_t *p)
{
    int32_t v = p[0] | p[1] << 8 | p[2] << 16;
    return v & 0x800000 ? v - 0x1000000 : v;
}

void tbdr_float_vtx(const uint8_t *g, uint8_t *pv, int oowexp, int mode)
{
    int snap = mode & 1, wdepth = mode >> 1 & 1, yflip = mode >> 2 & 1;
    int64_t bias = snap ? 3LL << 22 : 0;
    put16le(pv, fl_fix(g + 0, 4, bias, 0, 0, -32768, 32767));
    put16le(pv + 2, fl_fix(g + 4, 4, bias, yflip, TBDR_YFLIP16, -32768, 32767));
    int64_t oow = fl_fix(g + 32, oowexp, 0, 0, 0, 0, 65535);
    put16le(pv + 4, wdepth ? oow : fl_fix(g + 24, 0, 0, 0, 0, 0, 65535));
    put16le(pv + 6, oow);
    pv[8] = (uint8_t)fl_fix(g + 12, 0, 0, 0, 0, 0, 255);
    pv[9] = (uint8_t)fl_fix(g + 16, 0, 0, 0, 0, 0, 255);
    pv[10] = (uint8_t)fl_fix(g + 20, 0, 0, 0, 0, 0, 255);
    pv[11] = (uint8_t)fl_fix(g + 28, 0, 0, 0, 0, 0, 255);
    put24le(pv + 12, fl_divT(g + 36, g + 32));
    put24le(pv + 15, fl_divT(g + 40, g + 32));
}

/* floor(q + 1/2) from T: (floor(T / 2^(7-L)) + 1) >> 1, clamped to s16 */
static int st_from_T(int32_t T, int L)
{
    int32_t f2 = L < 8 ? T >> (7 - L) : T * 2;   /* (arithmetic shift: floor; BC1.md: L = 8, 256 texels) */
    return (int)clampi((f2 + 1) >> 1, -32768, 32767);
}

void tbdr_float_idx(const uint8_t *state, const uint8_t *p0, const uint8_t *p1,
                    const uint8_t *p2, uint8_t *rec, const tbdr_tex *tex)
{
    const uint8_t *pv[3] = {p0, p1, p2};
    memcpy(rec + 48, state, 16);
    const tbdr_tex *t = &tex[state[2] & 15];
    int L = t->lw > t->lh ? t->lw : t->lh;
    for (int i = 0; i < 3; i++) {
        uint8_t *v = rec + 16 * i;
        memcpy(v, pv[i], 8);
        put16le(v + 8, st_from_T(get24le(pv[i] + 12), L));
        put16le(v + 10, st_from_T(get24le(pv[i] + 15), L));
        memcpy(v + 12, pv[i] + 8, 4);
    }
}

/* ---- M10b: the vertex unit (VERTEX.md) ---- */

/* floor(n / d + 1/2), d > 0 */
static int64_t div_round(int64_t n, int64_t d)
{
    int64_t a = 2 * n + d, b = 2 * d, q = a / b;
    if ((a % b) && a < 0)
        q--;
    return q;
}

static void put32le(uint8_t *p, int64_t v)
{
    for (int i = 0; i < 4; i++)
        p[i] = (uint8_t)(v >> (8 * i));
}

static int32_t get32le(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24);
}

void tbdr_vunit_pack(const tbdr_vunit *u, uint8_t *b)
{
    memset(b, 0, TBDR_VREGS);
    for (int r = 0; r < 3; r++) {
        for (int j = 0; j < 3; j++)
            put32le(b + 12 * r + 4 * j, u->m[r][j]);
        put32le(b + 36 + 4 * r, u->t[r]);
    }
    put16le(b + 48, u->kx);
    put16le(b + 50, u->ky);
    put16le(b + 52, u->cx);
    put16le(b + 54, u->cy);
    put32le(b + 56, u->nearw);
    put32le(b + 60, u->za);
    put32le(b + 64, u->zb);
    b[68] = (uint8_t)u->pfrac;
    b[69] = (uint8_t)u->mode;
    b[70] = (uint8_t)u->oowexp;
    for (int j = 0; j < 3; j++)
        put16le(b + 72 + 2 * j, u->lw[j]);
}

void tbdr_vunit_unpack(const uint8_t *b, tbdr_vunit *u)
{
    for (int r = 0; r < 3; r++) {
        for (int j = 0; j < 3; j++)
            u->m[r][j] = (int32_t)sext(get32le(b + 12 * r + 4 * j), 25);
        u->t[r] = get32le(b + 36 + 4 * r);
    }
    u->kx = b[48] | b[49] << 8;
    u->ky = b[50] | b[51] << 8;
    u->cx = (int)sext(b[52] | b[53] << 8, 16);
    u->cy = (int)sext(b[54] | b[55] << 8, 16);
    u->nearw = get32le(b + 56);
    u->za = get32le(b + 60);
    u->zb = get32le(b + 64);
    u->pfrac = b[68] & 15;
    u->mode = b[69];
    u->oowexp = b[70] & 31;
    for (int j = 0; j < 3; j++) {            /* (a camera block: no light) */
        u->lw[j] = (int)sext(b[72 + 2 * j] | b[73 + 2 * j] << 8, 16);
        u->light[j] = 0;
        u->amb[j] = 255;
        u->dif[j] = 0;
    }
}

int tbdr_vtx_xform(const tbdr_vunit *u, const uint8_t *in, uint8_t *pv)
{
    return tbdr_vtx_xform_cs(u, in, pv, NULL);
}

int tbdr_vtx_xform_cs(const tbdr_vunit *u, const uint8_t *in, uint8_t *pv, int32_t *cs)
{
    int64_t P[3], c[3];
    int flags = 0, sh = 6 + (u->pfrac & 15);
    for (int j = 0; j < 3; j++)
        P[j] = sext(in[3 * j] | in[3 * j + 1] << 8 | in[3 * j + 2] << 16, 18);
    for (int r = 0; r < 3; r++) {
        int64_t s = 0;
        for (int j = 0; j < 3; j++)
            s += sext(u->m[r][j], 25) * P[j];
        c[r] = clampi(((s + ((int64_t)1 << (sh - 1))) >> sh) + (int64_t)u->t[r],
                      INT32_MIN, INT32_MAX);
    }
    memset(pv, 0, 8);
    memcpy(pv + 8, in + 10, 10);
    if (cs)
        for (int r = 0; r < 3; r++)
            cs[r] = (int32_t)c[r];
    int64_t w = c[2];
    if (w < (u->nearw > 1 ? u->nearw : 1))
        return 1;
    int64_t x = u->cx + div_round((int64_t)(u->kx & 0xFFFF) * c[0], w);
    int64_t y = u->cy - div_round((int64_t)(u->ky & 0xFFFF) * c[1], w);
    if (x < -32768 || x > 32767 || y < -32768 || y > 32767)
        flags |= 2;
    int64_t oow = clampi(div_round((int64_t)1 << ((u->oowexp & 31) + 14), w), 0, 65535);
    int64_t ooz = u->mode & TBDR_FLT_WDEPTH ? oow
                : clampi(u->za - (((int64_t)u->zb * oow + 32768) >> 16), 0, 65535);
    put16le(pv, clampi(x, -32768, 32767));
    put16le(pv + 2, clampi(y, -32768, 32767));
    put16le(pv + 4, ooz);
    put16le(pv + 6, oow);
    return flags;
}

void tbdr_vtx_tri(const uint8_t *state, const uint8_t *p0, const uint8_t *p1,
                  const uint8_t *p2, int flags, uint8_t *rec, const tbdr_tex *tex)
{
    tbdr_float_idx(state, p0, p1, p2, rec, tex);
    if (flags)
        for (int i = 0; i < 3; i++)
            memset(rec + 16 * i, 0, 4);
}

void tbdr_vobj_pack(const tbdr_vobj *o, uint8_t *b)
{
    memset(b, 0, TBDR_VOBJ);
    for (int r = 0; r < 3; r++) {
        for (int j = 0; j < 3; j++)
            put32le(b + 12 * r + 4 * j, o->m[r][j]);
        put32le(b + 36 + 4 * r, o->t[r]);
    }
    b[48] = (uint8_t)o->pfrac;
    b[49] = (uint8_t)o->flags;
    for (int j = 0; j < 3; j++) {
        put16le(b + 52 + 2 * j, o->light[j]);
        b[58 + j] = (uint8_t)o->amb[j];
        b[61 + j] = (uint8_t)o->dif[j];
    }
}

void tbdr_vobj_unpack(const uint8_t *b, tbdr_vobj *o)
{
    for (int r = 0; r < 3; r++) {
        for (int j = 0; j < 3; j++)
            o->m[r][j] = (int32_t)sext(get32le(b + 12 * r + 4 * j), 25);
        o->t[r] = get32le(b + 36 + 4 * r);
    }
    o->pfrac = b[48] & 15;
    o->flags = b[49];
    for (int j = 0; j < 3; j++) {
        o->light[j] = (int)sext(b[52 + 2 * j] | b[53 + 2 * j] << 8, 16);
        o->amb[j] = b[58 + j];
        o->dif[j] = b[61 + j];
    }
}

void tbdr_vunit_compose(const tbdr_vunit *cam, const tbdr_vobj *o, tbdr_vunit *out)
{
    *out = *cam;
    for (int r = 0; r < 3; r++) {
        for (int j = 0; j < 3; j++) {
            int64_t s = 0;
            for (int k = 0; k < 3; k++)
                s += sext(cam->m[r][k], 25) * sext(o->m[k][j], 25);
            out->m[r][j] = (int32_t)clampi((s + (1 << 19)) >> 20, -(1 << 24), (1 << 24) - 1);
        }
        int64_t s = 0;
        for (int k = 0; k < 3; k++)
            s += sext(cam->m[r][k], 25) * (int64_t)o->t[k];
        out->t[r] = (int32_t)clampi(((s + (1 << 19)) >> 20) + (int64_t)cam->t[r], INT32_MIN, INT32_MAX);
    }
    out->pfrac = o->pfrac & 15;
    for (int j = 0; j < 3; j++) {
        out->light[j] = o->light[j];
        out->amb[j] = o->amb[j] & 255;
        out->dif[j] = o->dif[j] & 255;
        if (o->flags & 1) {                  /* the light from the camera's: o^T lw */
            int64_t s = 0;
            for (int k = 0; k < 3; k++)
                s += sext(o->m[k][j], 25) * (int64_t)cam->lw[k];
            out->light[j] = (int)clampi((s + (1 << 19)) >> 20, -32768, 32767);
        }
    }
}

int tbdr_vtx_xform_lit(const tbdr_vunit *u, const uint8_t *in, uint8_t *pv)
{
    return tbdr_vtx_xform_lit_cs(u, in, pv, NULL);
}

int tbdr_vtx_xform_lit_cs(const tbdr_vunit *u, const uint8_t *in, uint8_t *pv, int32_t *cs)
{
    int flags = tbdr_vtx_xform_cs(u, in, pv, cs);
    int64_t dot = 0;
    for (int k = 0; k < 3; k++)
        dot += (int64_t)(int8_t)in[20 + k] * (int64_t)sext((uint32_t)u->light[k] & 0xFFFF, 16);
    if (dot < 0)
        dot = 0;
    for (int j = 0; j < 3; j++) {
        int64_t I = (u->amb[j] & 255) + (((int64_t)(u->dif[j] & 255) * dot + (1 << 20)) >> 21);
        if (I > 255)
            I = 255;
        int t = pv[8 + j] * (int)I + 128;
        pv[8 + j] = (uint8_t)((t + (t >> 8)) >> 8);
    }
    return flags;
}

/* M10b near-plane clipping: a new vertex where the edge from the inside
 * vertex a to the outside vertex b meets w = N */
int tbdr_vtx_clip_vertex(const tbdr_vunit *u, const uint8_t *pa, const int32_t *ca,
                       const uint8_t *pb, const int32_t *cb, uint8_t *pv)
{
    int64_t N = u->nearw > 1 ? u->nearw : 1;
    int64_t tnum = (int64_t)ca[2] - N, tden = (int64_t)ca[2] - cb[2];
    int64_t al = (tnum << 24) / tden;               /* (0 <= tnum < tden: floor) */
#define CLIP_I(a, b) ((a) + ((((int64_t)(b) - (a)) * al + (1 << 23)) >> 24))
    int64_t c0 = CLIP_I((int64_t)ca[0], cb[0]), c1 = CLIP_I((int64_t)ca[1], cb[1]);
    int flags = 0;
    memset(pv, 0, TBDR_PVTX);
    for (int k = 0; k < 4; k++)                     /* r, g, b, a */
        pv[8 + k] = (uint8_t)CLIP_I((int64_t)pa[8 + k], pb[8 + k]);
    for (int k = 0; k < 2; k++) {                   /* Ts, Tt (s24) */
        int64_t ta = sext(pa[12 + 3 * k] | pa[13 + 3 * k] << 8 | pa[14 + 3 * k] << 16, 24);
        int64_t tb = sext(pb[12 + 3 * k] | pb[13 + 3 * k] << 8 | pb[14 + 3 * k] << 16, 24);
        int64_t t = CLIP_I(ta, tb);
        pv[12 + 3 * k] = (uint8_t)t;
        pv[13 + 3 * k] = (uint8_t)(t >> 8);
        pv[14 + 3 * k] = (uint8_t)(t >> 16);
    }
#undef CLIP_I
    int64_t x = u->cx + div_round((int64_t)(u->kx & 0xFFFF) * c0, N);
    int64_t y = u->cy - div_round((int64_t)(u->ky & 0xFFFF) * c1, N);
    if (x < -32768 || x > 32767 || y < -32768 || y > 32767)
        flags |= 2;
    int64_t oow = clampi(div_round((int64_t)1 << ((u->oowexp & 31) + 14), N), 0, 65535);
    int64_t ooz = u->mode & TBDR_FLT_WDEPTH ? oow
                : clampi(u->za - (((int64_t)u->zb * oow + 32768) >> 16), 0, 65535);
    put16le(pv, clampi(x, -32768, 32767));
    put16le(pv + 2, clampi(y, -32768, 32767));
    put16le(pv + 4, ooz);
    put16le(pv + 6, oow);
    return flags;
}

int tbdr_vtx_tri_clip(const tbdr_vunit *u, const uint8_t *state, const uint8_t *const p[3],
                      const int32_t *const c[3], const int f[3], uint8_t *rec0, uint8_t *rec1,
                      const tbdr_tex *tex)
{
    int near = (f[0] & 1) + (f[1] & 1) + (f[2] & 1);
    if ((f[0] | f[1] | f[2]) & 2 || near == 0 || near == 3) {
        tbdr_vtx_tri(state, p[0], p[1], p[2], f[0] | f[1] | f[2], rec0, tex);
        return 1;
    }
    uint8_t nv[2][TBDR_PVTX];
    const uint8_t *o[4];
    int n = 0, m = 0, fl = 0;
    for (int i = 0; i < 3; i++) {                   /* Sutherland-Hodgman, w >= N */
        int j = (i + 1) % 3, in_i = !(f[i] & 1), in_j = !(f[j] & 1);
        if (in_i)
            o[n++] = p[i];
        if (in_i != in_j) {
            int a = in_i ? i : j, b = in_i ? j : i;  /* (from the inside vertex) */
            fl |= tbdr_vtx_clip_vertex(u, p[a], c[a], p[b], c[b], nv[m]);
            o[n++] = nv[m++];
        }
    }
    if (fl) {                                       /* a new vertex off the guard band */
        tbdr_vtx_tri(state, p[0], p[1], p[2], 2, rec0, tex);
        return 1;
    }
    tbdr_vtx_tri(state, o[0], o[1], o[2], 0, rec0, tex);
    if (n == 3)
        return 1;
    tbdr_vtx_tri(state, o[0], o[2], o[3], 0, rec1, tex);
    return 2;
}
