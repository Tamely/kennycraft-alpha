// render.c — the render kernel. One ray per pixel, branch-free shading.
//
// Cost model. There is no cache and the LSU is lane-serial: a load or store costs one bus
// cycle per active lane, so on 8 lanes an LW is ~12 cycles (IF, ID, EX, 8 x MEM, WB) where an
// ALU instruction is 4. A load of a uniform value (camera, render params) costs the same 12
// cycles as a per-lane one. So the kernel:
//   * reads the camera block and render params once per frame and keeps them in registers;
//   * never builds the ray (8 VSET fields) or the hit in memory: the fields go straight from
//     registers to VSET, and VTRACE / VGET results stay in registers;
//   * steps the ray direction incrementally (exact: see render_lane) instead of three
//     multiplies per component per pixel;
//   * takes face color and face shade from one 128-byte table indexed by VTRACE result bits
//     7:1, instead of two select chains per pixel;
// and touches memory twice per pixel: that LBU and the SB to the framebuffer.
//
// Order: each pass of the 8 lanes still shades one 4x2 block (D15), so the lanes of a pass
// trace neighbouring rays and the VTRACE walk (which waits for the slowest lane) stays short.
// Blocks go in raster order. The 8x8 tiles that used to group them bought nothing: with no
// cache there is no locality to win, and each block's rays are independent.
#include "render.h"
#include "trace.h"
#include "kc_hal.h"
#include "kc_lanes.h"
#include "kc_tables.h"

const uint8_t kc_face_color[16 * 3] = KC_FACE_COLORS;
// Pixel byte without fog, color << 4 | shade << 2, per VTRACE result bits 7:0 (type, face
// sign, face axis, hit; 0 on a miss). Generated next to KC_FACE_COLORS with the same face and
// shade rule as kc_shade_pixel below.
static const uint8_t kc_face_pix[256] = KC_FACE_PIX;

// Reference: shade one pixel from scratch, exactly as the kernel must. The host build checks
// kc_render_frame against it pixel for pixel (main_host.c, shots).
uint8_t kc_shade_pixel(int32_t px, int32_t py) {
    const kc_camera *c = KC_CAM;
    const kc_rparams *p = KC_RP;

    // Ray through the pixel center: forward + (x+0.5-W/2)*right_step - (y+0.5-H/2)*up_step
    int32_t sx = 2 * px + 1 - p->width;
    int32_t sy = 2 * py + 1 - p->height;
    q16 d[3];
    for (int i = 0; i < 3; i++)
        d[i] = c->forward[i] + ((c->right_step[i] * sx) >> 1) - ((c->up_step[i] * sy) >> 1);

    // Camera origin is absolute; the window (and VTRACE) work in window-relative coordinates.
    q16 o[3] = { c->origin[0] - Q16_INT(p->corner_x), c->origin[1], c->origin[2] - Q16_INT(p->corner_z) };
    kc_ray r;
    kc_hit h;
    ray_setup(&r, o, d);
    kc_trace(&r, p->max_steps, &h);

    int32_t hit  = (int32_t)HIT_HIT(h.res);
    int32_t ax   = (int32_t)HIT_AXIS(h.res);
    int32_t neg  = (int32_t)HIT_NEG(h.res);
    int32_t type = (int32_t)HIT_TYPE(h.res);

    // Face: ray stepping down (neg on y) hits a top face. One flat color per (type, face).
    int32_t ax_x = ax == 0, ax_y = ax == 1;
    int32_t face  = sel(ax_y, sel(neg, 0, 2), 1);
    int32_t color = kc_face_color[type * 3 + face];
    int32_t shade = sel(ax_y, sel(neg, 0, 3), sel(ax_x, 1, 2));
    int32_t fog   = (h.t >= p->fog_d[0]) + (h.t >= p->fog_d[1]) + (h.t >= p->fog_d[2]);

    int32_t sky_i = (py * p->sky_k) >> 16;
    sky_i = sel(sky_i > 15, 15, sky_i);
    int32_t sky = (COLOR_SKY << 4) | sky_i;

    return (uint8_t)sel(hit, PIX(color, shade, fog), sky);
}

static inline uint32_t mulhu32(uint32_t a, uint32_t b) { return (uint32_t)(((uint64_t)a * b) >> 32); }

// All ones when c is 1, 0 when c is 0. The empty asm keeps clang from turning the use into a
// branch, like sel().
static inline uint32_t lane_mask_of(int32_t c) {
    uint32_t m = -(uint32_t)c;
    __asm__("" : "+r"(m));
    return m;
}

// Per-frame values every ray of the frame shares. Kept small: everything here and the ray
// direction must stay in registers through the pixel loop (a spill reload is 12 cycles).
typedef struct {
    uint32_t frac[3];   // frac(origin) << 15
    uint32_t cell;      // start cell, packed (VSET field 0)
    int32_t  max_steps;
    q16      fog1[3];   // fog_d - 1: fog level n when fog1[n-1] < t
} kc_kframe;

// One ray: same VSET fields as ray_setup and the same pixel as kc_shade_pixel.
static inline __attribute__((always_inline))
uint8_t shade_ray(const kc_kframe *k, q16 dx, q16 dy, q16 dz, int32_t sky) {
    q16 dv[3] = { dx, dy, dz };
    uint32_t f[8];
    uint32_t signs = 0;
    for (int i = 0; i < 3; i++) {                     // unrolled; uniform trip count
        int32_t  m  = dv[i] >> 31;                    // all ones when stepping negative
        uint32_t p  = ~(uint32_t)m;                   // all ones when stepping positive
        q16      ad = (dv[i] ^ m) - m;                // |d|
        // tDelta = 1/|d|, 0x7FFF_FFFF for |d| = 0. qrcp(0) is Q16_BIG = 0x4000_0000 (fixed.c,
        // and QRCP is bit-exact with it), so OR-ing in 0x3FFF_FFFF gives exactly that.
        uint32_t td = (uint32_t)qrcp(ad) | (lane_mask_of(ad == 0) >> 2);
        // Distance to the first boundary << 15: frac for d < 0, 1 - frac for d >= 0.
        // (x ^ 0x7FFF_FFFF) + 1 = 2^31 - x, i.e. (1.0 - frac) << 15.
        uint32_t fr = (k->frac[i] ^ (p >> 1)) - p;
        // tMax = qmul(fr, td) for fr in [0, 1.0] and td in [0, 0x7FFF_FFFF]: one MULHU.
        f[1 + i] = mulhu32(fr, td << 1);
        f[4 + i] = td;
        signs |= ((uint32_t)dv[i] >> 31) << i;
    }
    f[0] = k->cell;
    f[7] = signs;

    uint32_t res;
    q16 t;
#if KC_VTRACE
    KC_VSET(0, f[0]); KC_VSET(1, f[1]); KC_VSET(2, f[2]); KC_VSET(3, f[3]);
    KC_VSET(4, f[4]); KC_VSET(5, f[5]); KC_VSET(6, f[6]); KC_VSET(7, f[7]);
    res = kc_vtrace_res(k->max_steps);
    t   = kc_vget_t();
#else
    kc_ray r;
    kc_hit h;
    for (int i = 0; i < 8; i++) r.f[i] = f[i];
    kc_trace(&r, k->max_steps, &h);
    res = h.res;
    t = h.t;
#endif

    int32_t fog = (k->fog1[0] < t) + (k->fog1[1] < t) + (k->fog1[2] < t);
    int32_t pix = kc_face_pix[res & 255u] | fog;
    return (uint8_t)sel((int32_t)(res & 1u), pix, sky);
}

// Every pixel this lane owns: offset (lane & 3, lane >> 2) inside each 4x2 block.
static inline __attribute__((always_inline)) void render_lane(int32_t lane) {
    const kc_camera *c = KC_CAM;
    const kc_rparams *p = KC_RP;
    int32_t W = p->width, H = p->height, sky_k = p->sky_k;

    kc_kframe k;
    q16 o[3] = { c->origin[0] - Q16_INT(p->corner_x), c->origin[1], c->origin[2] - Q16_INT(p->corner_z) };
    for (int i = 0; i < 3; i++) {
        k.frac[i] = (uint32_t)qfrac(o[i]) << 15;
        q16 f1 = p->fog_d[i] - 1;                     // fog_d > 0 (kc_set_max_steps)
        __asm__("" : "+r"(f1));                       // else clang folds the -1 back into SLT + XORI
        k.fog1[i] = f1;
    }
    k.cell = CELL_PACK(qfloor(o[0]), qfloor(o[1]), qfloor(o[2]));
    k.max_steps = p->max_steps;

    // d(px, py) = forward + (right_step * sx) >> 1 - (up_step * sy) >> 1 with sx = 2px + 1 - W,
    // sy = 2py + 1 - H. One block right adds 8 to sx: right_step * 8 is even, so the shift
    // stays exact and d grows by exactly right_step * 4. One block row down: d -= up_step * 2.
    int32_t lx = lane & 3, ly = lane >> 2;
    int32_t sx = 2 * lx + 1 - W, sy = 2 * ly + 1 - H;
    q16 d0[3], dr[3], du[3];
    for (int i = 0; i < 3; i++) {
        d0[i] = c->forward[i] + ((c->right_step[i] * sx) >> 1) - ((c->up_step[i] * sy) >> 1);
        dr[i] = c->right_step[i] * 4;
        du[i] = c->up_step[i] * 2;
    }

    uint32_t row = (uint32_t)(ly * FB_STRIDE + lx);
    int32_t sky_acc = ly * sky_k;                      // py * sky_k
    for (int32_t by = 0; by < H; by += 2) {
        int32_t sky_i = sky_acc >> 16;
        sky_i = sel(sky_i > 15, 15, sky_i);
        int32_t sky = (COLOR_SKY << 4) | sky_i;
        q16 dx = d0[0], dy = d0[1], dz = d0[2];
        for (uint32_t i = row, end = row + (uint32_t)W; i < end; i += 4) {
            hal_fb_put(i, shade_ray(&k, dx, dy, dz, sky));
            dx += dr[0]; dy += dr[1]; dz += dr[2];
        }
        d0[0] -= du[0]; d0[1] -= du[1]; d0[2] -= du[2];
        row += 2 * FB_STRIDE;
        sky_acc += 2 * sky_k;
    }
}

void kc_render_frame(void) {
#if KC_LANES == 8
    render_lane((int32_t)lane_id());                   // called between mask_all() and mask_set(1)
#else
    for (int32_t lane = 0; lane < 8; lane++)           // one lane: walk the frame once per lane slot
        render_lane(lane);
#endif
}
