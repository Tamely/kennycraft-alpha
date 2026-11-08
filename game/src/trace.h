// trace.h — ray setup and voxel traversal (Amanatides & Woo DDA).
// kc_trace() has exactly the VTRACE contract from the Interfaces section, so:
//   * the software build of it IS the "C reference for one ray" for the VTRACE testbench,
//   * swapping in the hardware unit (KC_VTRACE=1) changes nothing above this file.
#ifndef KC_TRACE_H
#define KC_TRACE_H
#include <stdint.h>
#include "fixed.h"

// VSET fields. f[0] start cell packed (x 5:0, y 10:6, z 16:11); f[1..3] tMax x,y,z;
// f[4..6] tDelta x,y,z (Q16.16); f[7] step signs (bit0 x, bit1 y, bit2 z; 1 = negative).
typedef struct { uint32_t f[8]; } kc_ray;

// VTRACE result + VGET fields.
typedef struct {
    uint32_t res;    // bit0 hit, 2:1 face axis, 3 face sign, 7:4 type, 15:8 steps
    uint32_t cell;   // VGET 0: hit cell packed like f[0]
    q16      t;      // VGET 1: t at hit
} kc_hit;

#define HIT_HIT(r)   ((r) & 1u)
#define HIT_AXIS(r)  (((r) >> 1) & 3u)
#define HIT_NEG(r)   (((r) >> 3) & 1u)
#define HIT_TYPE(r)  (((r) >> 4) & 15u)
#define HIT_STEPS(r) (((r) >> 8) & 255u)
#define CELL_PACK(x, y, z) (((uint32_t)(x) & 63u) | (((uint32_t)(y) & 31u) << 6) | (((uint32_t)(z) & 63u) << 11))
#define CELL_X(c) ((int32_t)((c) & 63u))
#define CELL_Y(c) ((int32_t)(((c) >> 6) & 31u))
#define CELL_Z(c) ((int32_t)(((c) >> 11) & 63u))

// Build VSET fields from a window-relative origin and a direction (both Q16.16).
// The origin must be inside the window. Branch-free: safe on 8 lanes.
void ray_setup(kc_ray *r, const q16 o[3], const q16 d[3]);

// Step the ray up to max_steps (0..255) until it hits a solid block or leaves the window.
// Rules = wiki Tai page, VTRACE "One step" (vtrace.sv must match):
//   x if tMax.x < tMax.y && tMax.x < tMax.z, else y if tMax.y < tMax.z, else z (strict <);
//   tMax additions saturate at 0x7FFF_FFFF; a zero direction component has tDelta 0x7FFF_FFFF;
//   the start cell is never tested; max_steps = 0 returns at once and leaves *h unchanged.
// On a miss (left the window or hit the cap): hit = 0, type = 0, and axis, sign, t and cell
// are those of the last step taken (cell packed from its low bits).
void kc_trace(const kc_ray *r, int32_t max_steps, kc_hit *h);

// Always the software version (used by tests to compare against VTRACE).
void vtrace_ref(const kc_ray *r, int32_t max_steps, kc_hit *h);

#if KC_VTRACE
// Ray unit, custom-1 (0x2B), for callers that keep the ray in registers (the render kernel).
// VSET: funct7 carries the field number.
#define KC_VSET(field, v) __asm__ volatile(".insn r 0x2B, 0, " #field ", x0, %0, x0" :: "r"(v))
// VTRACE rd = {16'b0, result}: the core zero-extends, so no mask is needed.
static inline uint32_t kc_vtrace_res(int32_t max_steps) {
    uint32_t res;
    __asm__ volatile(".insn r 0x2B, 1, 0, %0, %1, x0" : "=r"(res) : "r"(max_steps) : "memory");
    return res;
}
static inline uint32_t kc_vget_cell(void) {
    uint32_t v; __asm__ volatile(".insn r 0x2B, 2, 0, %0, x0, x0" : "=r"(v)); return v;
}
static inline q16 kc_vget_t(void) {
    uint32_t v; __asm__ volatile(".insn r 0x2B, 2, 1, %0, x0, x0" : "=r"(v)); return (q16)v;
}
#endif

#endif
