/*
 * tbdr_model - bit-exact reference model of the MEGA65 tile-based deferred
 * renderer (M1: flat colour + Gouraud light through a colormap, no textures).
 *
 * The model defines the hardware's contract: the triangle RAM image, the
 * registers and the colormap go in, the frame store bytes come out. The VHDL
 * (src/vhdl/tbdr_*.vhdl) must produce the same bytes; the GHDL bench compares
 * them. Formats and arithmetic: src/tbdr/README.md.
 */
#ifndef TBDR_MODEL_H
#define TBDR_MODEL_H

#include <stdint.h>

#define TBDR_TILE      32   /* tile edge in pixels */
#define TBDR_REC       32   /* bytes per triangle record */
#define TBDR_MAXTRI    1024
#define TBDR_CMAP_SIZE (64 * 256)

/* 64-byte records (regs.rec64), shaped after Glide's GrVertex (M3), with
 * textures (M4a: on-chip texture memory, point sampling and bilinear):
 *   vertex i at 16*i: 0 x s16 12.4, 2 y s16 12.4, 4 ooz u16 (depth, 0 = near),
 *                     6 oow u16 (1/w, 65535 = the near plane),
 *                     8 s s16, 10 t s16 (texels, 12.4),
 *                     12 r, 13 g, 14 b, 15 a (u8; r is the light until the
 *                     RGB pipeline, M7; g, b, a are carried, unused)
 *   48 colour (untextured), 49 flags (bit 0 cull counter-clockwise, bit 1
 *   textured, bit 2 bilinear, bit 3 cull clockwise), 50 texture number (0-15),
 *   51 depth (M5a): bits 0-2 the function - 0 less, 1 lequal, 2 equal,
 *   3 greater, 4 gequal, 5 notequal, 6 always, 7 never (the new depth against
 *   the stored one) - bit 3 no depth write; 52 clip window (M5a): x min, x max,
 *   y min, y max, u16 pixels, max exclusive, x max = 0: no window;
 *   60..63 reserved (0)
 * M5b fog (flags bit 4): the RGB path's light-scaled colour v (each channel)
 * becomes (v * (256 - f) + fogrgb * f) >> 8 before the dither, f the fog table
 * at the pixel's w: index 4*lz + 3 - (the two bits under q's leading one), lz
 * the leading zeros of q (oow, 16.16, clamped as for the texture coordinates),
 * at most 63 - Glide's guFogTableIndexToW spacing. A fogged triangle takes the
 * RGB path whatever its texture mode (point and flat colours as four equal
 * texels) and gets the oow planes even when untextured.
 * Point sampling: colormap[shade][texel], as an untextured triangle's colour.
 * Bilinear: the four texels' RGB (palrgb) blended, times the light through the
 * light curve, an ordered dither to RGB555, the inverse colour table back to a
 * palette index.
 * Texture memory (32 KB) is in 2x2 quads, so that a bilinear footprint is one
 * texel from each of four banks: texel (u, v) of a texture at
 *   base*4 + (((v>>1) << (lw-1)) + (u>>1))*4 + (v&1)*2 + (u&1)
 * coordinates wrap. */
#define TBDR_REC64   64
#define TBDR_TEXMEM  262144   /* 256 KB on chip (descriptor base: 16 bits of quads) */
enum { TBDR_F_TEX = 2, TBDR_F_BILINEAR = 4, TBDR_F_CULLCW = 8, TBDR_F_FOG = 16, TBDR_F_PUNCH = 32,
       TBDR_F_TRANS = 64, TBDR_F_M7 = 128 };
/* M7a colour (flags bit 7; COLOUR.md "The contract"): vertex bytes 12-15 R, G,
 * B, A iterated (R on the light plane), byte 63 bits 0-5 an entry of the state
 * table (64 x 8 bytes: constant R, G, B, A, the colour combine word, the alpha
 * combine word, Glide's codes); the pixel's colour is Glide's colour combine of
 * the iterated colour, the texture (palette RGB, bilinear weighted; an
 * untextured triangle's byte 48) and the constant colour, then fog, the dither
 * and the inverse table - or M6's blend for a translucent triangle.
 * M7b: the alpha combine (the same unit on alpha: local = iterated or constant
 * A, other = iterated A, texture alpha or constant A) gives an M7 triangle's
 * alpha for the alpha test (M5c) and the translucency blend (M6, without byte
 * 62); "local alpha" and "other alpha" are the alpha combine's choices for the
 * colour factors too, as on the Voodoo. */
/* M6 translucency (flags bit 6; TRANSLUCENCY.md): per tile after the TSP has
 * shaded it, the translucent triangles in submission order: depth test (byte
 * 51 as for any triangle), the colour on the RGB path up to the light and fog
 * (9 bits a channel, clamped to 255), alpha = palette alpha * (byte 62 + 1) >> 8
 * (0..255, scaled to 0..256 as a + (a >> 7)), blended with the pixel's current
 * colour from the palette - byte 61 bits 4-5: 0 alpha (src a + dst (256 - a))
 * >> 8, 1 additive min(255, dst + (src a >> 8)), 2 multiply dst (src + 1) >> 8
 * (the alpha unused: Glide's DST_COLOR/ZERO, lightmaps) - then the RGB path's dither and
 * the inverse table. The opaque and punch-through passes skip them. */
/* M5c punch-through (flags bit 5; PUNCHTHROUGH.md): such a triangle's pixel
 * counts only where its alpha passes the test - byte 60 the reference, byte 61
 * bits 0-2 the function (the depth functions' encoding: the alpha against the
 * reference). The alpha: palette alpha (palalpha) of the texel, the four
 * texels' weighted like RGB for bilinear (sum w * a >> 8), the colour's for a
 * flat triangle. Per tile, the opaque triangles first, then the punch-through
 * ones in submission order: depth test, alpha test, then depth (unless no
 * write) and tag; a failing pixel writes nothing. */
enum { TBDR_Z_LESS, TBDR_Z_LEQUAL, TBDR_Z_EQUAL, TBDR_Z_GREATER, TBDR_Z_GEQUAL,
       TBDR_Z_NOTEQUAL, TBDR_Z_ALWAYS, TBDR_Z_NEVER, TBDR_Z_NOWRITE = 8 };

typedef struct {
    uint32_t base;     /* in 4-byte quads (BC1: a multiple of 8) */
    int      lw, lh;   /* log2 width, height in texels, 1..8 (BC1: 3..8) */
    int      fmt;      /* descriptor byte 3: bit 0 BC1 (BC1.md), else 8-bit palette indices;
                        * bits 4-6 the mip levels - 1 (TBDR_TEX_LEVELS: one after the
                        * other from base, each a quarter; a triangle with record
                        * byte 50 bit 4 samples the level its size asks for) */
} tbdr_tex;

/* Triangle record, little-endian, TBDR_REC bytes:
 *   0 x0  s16  12.4 screen pixels     8 x1    16 x2
 *   2 y0  s16  12.4                  10 y1    18 y2
 *   4 z0  u16  0 = near              12 z1    20 z2
 *   6 l0  u8   light 0..255          14 l1    22 l2
 *   7 colour (colormap column)       15 flags 23..31 reserved (0)
 * flags bit 0: cull back faces (counter-clockwise on screen, y down). */
enum { TBDR_F_CULL = 1 };

typedef struct {
    int      cols, rows;   /* screen in 8x8 characters: W = 8*cols, H = 8*rows */
    int      count;        /* triangles in the triangle RAM */
    uint8_t  bg;           /* colour of pixels no triangle covers */
    uint32_t zclear;       /* depth each tile starts with */
    int      dither;       /* ordered 4x4 dither of the light level */
    /* M3/M4a (zero = the M1 contract, unchanged) */
    int             rec64;       /* 64-byte records */
    const uint8_t  *texmem;      /* TBDR_TEXMEM bytes of texture memory */
    const tbdr_tex *tex;         /* 16 texture descriptors */
    const uint8_t  *palrgb;      /* 256 x (R, G, B): the palette, for bilinear */
    const uint8_t  *inv;         /* 32768: RGB555 -> palette index, for bilinear */
    const uint16_t *lcurve;      /* 1024: the light (sixteenths of a shade) -> an RGB
                                    multiplier 0..1024, for bilinear (to match the
                                    palette's brightness ramps) */
    const uint8_t  *fogtab;      /* M5b: 64 fog factors (0-255) by the w index; NULL: none */
    uint8_t         fogrgb[3];   /* M5b: the fog colour */
    const uint8_t  *palalpha;    /* M5c: 256 alphas by palette index; NULL: all 255 */
    const uint8_t  *state;       /* M7: the state table, 64 x 8 bytes; NULL: zeros */
    int             miss_cycles; /* estimator: clocks a texture cache miss would stall
                                    (SDRAM textures, M4b; default 20) */
} tbdr_regs;

typedef struct {
    long tris_in, tris_valid;  /* records read, triangles that survived setup */
    long pairs;                /* (triangle, tile) pairs the binner passed on */
    long isp_tests;            /* pixels the ISP evaluated */
    long isp_writes;           /* depth/tag writes */
    long tsp_pixels;           /* pixels shaded (one per covered screen pixel) */
    long cycles;               /* estimate for the M1 VHDL at 1 pixel/clock */
    long texels;               /* textured pixels shaded */
    long tex_misses;           /* misses of a modelled 4 KB texture cache (for M4b) */
    long fast;                 /* estimate for tbdr-fast (81 MHz clocks), misses included */
    long fast_nomiss;          /* the same without miss stalls */
} tbdr_stats;

/* Frame store offset of pixel (x, y): full-colour-mode characters in screen
 * order, 64 bytes each, 8 bytes per pixel row. */
static inline uint32_t tbdr_fcm_offset(int cols, int x, int y)
{
    return ((uint32_t)((y >> 3) * cols + (x >> 3)) << 6) | ((y & 7) << 3) | (x & 7);
}

/* Renders one frame. frame must hold cols*rows*64 bytes. Returns 0, or -1 for
 * registers out of range. */
int tbdr_render(const uint8_t *triram, const uint8_t *colormap,
                const tbdr_regs *r, uint8_t *frame, tbdr_stats *st);

/* M8 float vertices (FLOAT.md): one float triangle as streamed into FLTDATA -
 * the 16 state bytes (record bytes 48-63), then three Glide GrVertex (x, y, z,
 * r, g, b, ooz, a, oow, sow, tow, tmu oow: IEEE single, little-endian, 48
 * bytes each) - converted into the 64-byte record. Every field is floor(v +
 * 1/2), exact, clamped to the field: x, y v*16 (snap: minus 3*2^22; yflip:
 * y = TBDR_YFLIP16 minus that, before the clamp: Glide's lower-left origin,
 * y' = 200 - y, row r shows as row 199 - r), ooz v (wdepth: the oow field's value instead: the w-buffer),
 * oow v*2^oowexp, r g b a v, s t (sow, tow)/oow * 2^(L-4), L = max(lw, lh)
 * of the record's texture (byte 50 bits 0-3). NaN, zero and denormals = 0,
 * infinity clamps; an s, t with an oow that is zero, denormal, infinite or
 * NaN = 0. z and the tmu oow are ignored. */
#define TBDR_FLTTRI 160
#define TBDR_FLT_SNAP   1                  /* FLTMODE bits */
#define TBDR_FLT_WDEPTH 2
#define TBDR_FLT_YFLIP  4
#define TBDR_YFLIP16    (200 * 16)        /* pixel rows mirrored exactly: centres at k + 1/2 */
void tbdr_float_tri(const uint8_t *in, uint8_t *rec, const tbdr_tex *tex,
                    int oowexp, int mode);

/* M9c indexed vertices (FLOAT.md "M9c"): a GrVertex converted once into a
 * TBDR_PVTX-byte pool entry, the same conversions as tbdr_float_tri. */
#define TBDR_PVTX 18
/* pool entry: 0-7 x y ooz oow, 8-11 r g b a, 12-14 Ts (s24), 15-17 Tt (s24):
 * T = floor(sow / oow * 16), saturated to [-2^23, 2^23 - 1] (an oow that is
 * not a number, or a zero or NaN sow: 0) - the texture-independent part of
 * s, t; a triangle takes floor(T / 2^(7-L)) = floor(2q) and (that + 1) >> 1 =
 * floor(q + 1/2), q = sow / oow * 2^(L-4): exactly tbdr_float_tri's s, t. */
void tbdr_float_vtx(const uint8_t *g, uint8_t *pv, int oowexp, int mode);
/* the record from the 16 state bytes and three pool entries */
void tbdr_float_idx(const uint8_t *state, const uint8_t *p0, const uint8_t *p1,
                    const uint8_t *p2, uint8_t *rec, const tbdr_tex *tex);

/* M10b vertex unit (VERTEX.md): a model-space vertex (TBDR_VIN bytes) through
 * the object's registers into a TBDR_PVTX pool entry, all integer:
 *   VIN: 0-2 x, 3-5 y, 6-8 z (s18 in 24 bits, the sign at bit 17, bits 18-23
 *        ignored; pfrac fraction bits), 9 reserved, 10-13 r g b a, 14-16 Ts,
 *        17-19 Tt (s24: the pool's T, Glide's s, t times 16) - bytes 10-19
 *        become the pool entry's 8-17 unchanged.
 *   c_r = sat32(floor(sum_j m[r][j] * P_j / 2^(6+pfrac) + 1/2) + t[r]),
 *        r = x, y, w: the camera-space coordinates, 14 fraction bits (m s25,
 *        20 fraction bits; the products fit a DSP48's 25 x 18, the sum 45 bits)
 *   w < max(nearw, 1): flag bit 0 (behind the near plane), the entry zeros
 *        but the attributes
 *   x = cx + floor(kx * c_x / w + 1/2), y = cy - floor(ky * c_y / w + 1/2)
 *        (12.4), outside s16: clamped and flag bit 1 (outside the guard band)
 *   oow = min(65535, floor(2^(oowexp+14) / w + 1/2))
 *   ooz = mode bit 1 (wdepth) ? oow : clamp(za - floor((zb * oow + 2^15) / 2^16), 0, 65535)
 * A triangle with a flagged vertex is not drawn (no near clipping yet).
 * Register block (TBDR_VREGS bytes, little-endian, for the bench and Slide):
 *   0-35 m (rows x, y, w; s25 in 32 bits), 36-47 t x, y, w (s32), 48 kx, 50 ky
 *   (u16), 52 cx, 54 cy (s16), 56 nearw (s32), 60 za, 64 zb (s32), 68 pfrac
 *   (0..15), 69 mode, 70 oowexp (0..31), 71 reserved, 72-77 the world light
 *   x, y, z (s16; for objects with byte 49 bit 0), 78-79 reserved */
#define TBDR_VIN   20
#define TBDR_VREGS 80
typedef struct {
    int32_t m[3][3], t[3];
    int     kx, ky, cx, cy;
    int32_t nearw, za, zb;
    int     pfrac, mode, oowexp;
    int     light[3], amb[3], dif[3];       /* step 2b: the object's (not in the block) */
    int     lw[3];                          /* the world light (block 72-77, s16) */
} tbdr_vunit;
void tbdr_vunit_pack(const tbdr_vunit *u, uint8_t *b);
void tbdr_vunit_unpack(const uint8_t *b, tbdr_vunit *u);
/* returns the flags (0: the vertex is good) */
int tbdr_vtx_xform(const tbdr_vunit *u, const uint8_t *in, uint8_t *pv);
/* M10b step 2b, lit vertices (VTXINL: TBDR_VINL bytes = a VIN record, then
 * the normal nx, ny, nz s8 (127 = 1), a reserved byte): tbdr_vtx_xform, then
 * the entry's r, g, b by the object's light (light s16 in object space,
 * 2^14 = 1; amb, dif per channel, 255 = 1):
 *   dot = nx * lx + ny * ly + nz * lz
 *   I_j = min(255, amb_j + floor((dif_j * max(0, dot) + 2^20) / 2^21))
 *   c_j = floor((t + floor(t / 256)) / 256), t = c_j * I_j + 128 (= c * I / 255 rounded)
 * Alpha, s, t unchanged. */
#define TBDR_VINL  24
int tbdr_vtx_xform_lit(const tbdr_vunit *u, const uint8_t *in, uint8_t *pv);
/* the same, also giving the camera-space x, y, w (cs[3], s32) */
int tbdr_vtx_xform_cs(const tbdr_vunit *u, const uint8_t *in, uint8_t *pv, int32_t *cs);
int tbdr_vtx_xform_lit_cs(const tbdr_vunit *u, const uint8_t *in, uint8_t *pv, int32_t *cs);
/* M10b near-plane clipping (VERTEX.md "Near clipping"): a triangle of pool
 * entries p, their camera-space coordinates c and flags f. With a vertex off
 * the guard band (flag bit 1), all three or none behind the near plane: as
 * tbdr_vtx_tri, one record. Else the triangle is clipped to w >= N (N =
 * max(nearw, 1)), Sutherland-Hodgman over the edges 0-1, 1-2, 2-0, a new
 * vertex on each crossing edge, interpolated from its inside vertex a towards
 * the outside one b:
 *   al = floor((w_a - N) * 2^24 / (w_a - w_b))
 *   v  = v_a + floor(((v_b - v_a) * al + 2^23) / 2^24)
 *        for the camera x, y and the entry's r, g, b, a, Ts, Tt (s24)
 * then projected at w = N as tbdr_vtx_xform does. A new vertex off the guard
 * band: area 0, one record. Else the polygon o0, o1, o2 (,o3) in that order:
 * records rec0 = (o0, o1, o2) and, from a quad, rec1 = (o0, o2, o3); returns
 * the records written (1 or 2). */
/* (the new vertex of an edge from inside a to outside b, for the tests) */
int tbdr_vtx_clip_vertex(const tbdr_vunit *u, const uint8_t *pa, const int32_t *ca,
                         const uint8_t *pb, const int32_t *cb, uint8_t *pv);
int tbdr_vtx_tri_clip(const tbdr_vunit *u, const uint8_t *state, const uint8_t *const p[3],
                      const int32_t *const c[3], const int f[3], uint8_t *rec0, uint8_t *rec1,
                      const tbdr_tex *tex);
/* the record of a triangle from vertex-unit entries: tbdr_float_idx's, and
 * when any of its entries is flagged, every vertex's x and y 0 (area 0: the
 * setup rejects it; the record number stays taken) */
void tbdr_vtx_tri(const uint8_t *state, const uint8_t *p0, const uint8_t *p1,
                  const uint8_t *p2, int flags, uint8_t *rec, const tbdr_tex *tex);
/* M10b camera x object: an object block (TBDR_VOBJ bytes: 0-35 the object's
 * matrix o, rows x, y, z, s25 with 20 fraction bits in 32; 36-47 its
 * translation p in world units, s32 with 14; 48 pfrac; 49-51 reserved; step
 * 2b: 52-57 the light direction in object space x, y, z s16 (2^14 = 1),
 * 58-60 ambient r g b, 61-63 diffuse r g b (255 = 1); byte 49 bit 0: the
 * light from the camera block's world light instead,
 *   light[j] = sat16(floor((sum_k o[k][j] * lw[k] + 2^19) / 2^20)),
 * o^T lw, exact for a rotation) is
 * combined with the camera block c (the register block of VREGDATA) into the
 * block the vertices use:
 *   m[r][j] = sat25(floor(sum_k c.m[r][k] * o[k][j] / 2^20 + 1/2))
 *   t[r]    = sat32(floor(sum_k c.m[r][k] * p[k] / 2^20 + 1/2) + c.t[r])
 *   pfrac, the light = the object's; everything else the camera's. A
 * camera block alone (VREGDATA) leaves no light: direction 0, ambient 255,
 * diffuse 0 (lit vertices keep their colour). */
#define TBDR_VOBJ 64
typedef struct {
    int32_t m[3][3], t[3];
    int     pfrac, flags;                   /* flags: byte 49 */
    int     light[3], amb[3], dif[3];
} tbdr_vobj;
void tbdr_vobj_pack(const tbdr_vobj *o, uint8_t *b);
void tbdr_vobj_unpack(const uint8_t *b, tbdr_vobj *o);
void tbdr_vunit_compose(const tbdr_vunit *cam, const tbdr_vobj *o, tbdr_vunit *out);
/* M10b ports: $56 VREGIDX (write: the VREGDATA and VOBJDATA streams restart
 * at byte 0), $57 VREGDATA (the TBDR_VREGS bytes, the camera; it applies, with
 * an identity object, to the vertices that come after its last byte), $59
 * VOBJDATA (the TBDR_VOBJ bytes: from its last byte on the vertices use the
 * camera x this object), $58 VTXIN (TBDR_VIN bytes a vertex, into the
 * pool entry VTXIDX, which advances), $5A VTXINL (TBDR_VINL bytes, lit:
 * tbdr_vtx_xform_lit); CAPS bit 3 */

#endif
