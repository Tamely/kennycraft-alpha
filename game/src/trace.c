// trace.c — ray setup + voxel DDA. Branch-free per lane; the only loop condition is a
// BALLOT, so the same code runs on 1 lane (laptop, Stage 0) and 8 lanes (Stage 1).
#include "trace.h"
#include "kc_hal.h"
#include "kc_lanes.h"

#if KC_LANES == 1
uint32_t kc_soft_mask = 1;
#endif

#define TDELTA_INF 0x7FFFFFFF

// a + b for a, b in [0, 0x7FFF_FFFF], saturating at 0x7FFF_FFFF. Branch-free.
static inline q16 sat_add(q16 a, q16 b) {
    uint32_t s = (uint32_t)a + (uint32_t)b;           // fits in 32 unsigned bits
    return sel((int32_t)(s >> 31), TDELTA_INF, (q16)s);
}

void ray_setup(kc_ray *r, const q16 o[3], const q16 d[3]) {
    uint32_t signs = 0;
    for (int i = 0; i < 3; i++) {                     // unrolled by -O2; uniform trip count
        int32_t neg = (int32_t)((uint32_t)d[i] >> 31);
        q16 ad = sel(neg, -d[i], d[i]);
        q16 td = sel(ad == 0, TDELTA_INF, qrcp(ad)); // tDelta = 1/|d|; a zero component never steps
        q16 fr = qfrac(o[i]);
        fr = sel(neg, fr, Q16_ONE - fr);              // distance to the first boundary
        r->f[1 + i] = (uint32_t)qmul(fr, td);         // tMax (fr <= 1.0, so <= TDELTA_INF)
        r->f[4 + i] = (uint32_t)td;
        signs |= (uint32_t)neg << i;
    }
    r->f[0] = CELL_PACK(qfloor(o[0]), qfloor(o[1]), qfloor(o[2]));
    r->f[7] = signs;
}

void vtrace_ref(const kc_ray *r, int32_t max_steps, kc_hit *h) {
    int32_t x = CELL_X(r->f[0]), y = CELL_Y(r->f[0]), z = CELL_Z(r->f[0]);
    q16 tmx = (q16)r->f[1], tmy = (q16)r->f[2], tmz = (q16)r->f[3];
    q16 tdx = (q16)r->f[4], tdy = (q16)r->f[5], tdz = (q16)r->f[6];
    int32_t nx = r->f[7] & 1, ny = (r->f[7] >> 1) & 1, nz = (r->f[7] >> 2) & 1;
    int32_t sx = 1 - 2 * nx, sy = 1 - 2 * ny, sz = 1 - 2 * nz;

    int32_t steps = 0, ax = 0, neg = 0, typ = 0, hit = 0;
    q16 t = 0;
    // D20: BALLOT before LMASK, so the loop never ends with M = 0. A cap of 0 never enters
    // (results unchanged), which folds the old early return into the loop test. Results go
    // to *h every step while the lane is still live: once LMASK freezes a lane, clang may
    // already have reused the registers holding hit/cell/t for the next step's values.
    uint32_t saved = mask_get();
    int32_t alive = max_steps > 0;
    while (ballot((uint32_t)alive)) {                // uniform: "any lane still walking?"
        lmask((uint32_t)alive);                      // finished lanes freeze their registers
        // Wiki (Tai, VTRACE "One step"): strict compares, so ties go to z, then y.
        int32_t cx = (tmx < tmy) & (tmx < tmz);
        int32_t cy = (cx ^ 1) & (tmy < tmz);
        int32_t cz = (cx | cy) ^ 1;
        t   = sel(cx, tmx, sel(cy, tmy, tmz));
        ax  = sel(cy, 1, sel(cz, 2, 0));
        neg = sel(cx, nx, sel(cy, ny, nz));
        x += sel(cx, sx, 0);  y += sel(cy, sy, 0);  z += sel(cz, sz, 0);
        tmx = sel(cx, sat_add(tmx, tdx), tmx);       // saturate at 0x7FFF_FFFF
        tmy = sel(cy, sat_add(tmy, tdy), tmy);
        tmz = sel(cz, sat_add(tmz, tdz), tmz);
        steps++;

        int32_t oob = ((uint32_t)x > 63u) | ((uint32_t)y > 31u) | ((uint32_t)z > 63u);
        typ = hal_win_get(x & 63, y & 31, z & 63);   // always a legal address
        hit = (typ != 0) & (oob ^ 1);
        h->res  = (uint32_t)hit | ((uint32_t)ax << 1) | ((uint32_t)neg << 3)
                | ((uint32_t)(typ & -hit) << 4) | ((uint32_t)steps << 8);
        h->cell = CELL_PACK(x, y, z);
        h->t    = t;
        alive = ((hit | oob) ^ 1) & (steps < max_steps);
    }
    mask_set(saved);
}

#if KC_VTRACE
void kc_trace(const kc_ray *r, int32_t max_steps, kc_hit *h) {
    KC_VSET(0, r->f[0]); KC_VSET(1, r->f[1]); KC_VSET(2, r->f[2]); KC_VSET(3, r->f[3]);
    KC_VSET(4, r->f[4]); KC_VSET(5, r->f[5]); KC_VSET(6, r->f[6]); KC_VSET(7, r->f[7]);
    h->res  = kc_vtrace_res(max_steps);
    h->cell = kc_vget_cell();
    h->t    = kc_vget_t();
}
#else
void kc_trace(const kc_ray *r, int32_t max_steps, kc_hit *h) { vtrace_ref(r, max_steps, h); }
#endif
