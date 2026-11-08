// kc_hal.h — the only file that knows whether we're on the laptop or the chip.
// Chip build: direct loads/stores to the memory map in the Interfaces section (v1.0).
// Host build (-DKC_HOST): hal_host.c emulates the same memory map, including the world
// window ring (WIN_ORIGIN, SLOT_VALID) and the DDR3 chunk engine, so chunk code is tested
// on the laptop exactly as it will run on the board.
#ifndef KC_HAL_H
#define KC_HAL_H
#include <stdint.h>

// ---- World window (D14) ---------------------------------------------------------------
#define WIN_X 64
#define WIN_Y 32
#define WIN_Z 64
#define WIN_INDEX(x, y, z) (((uint32_t)(y) << 12) | ((uint32_t)(z) << 6) | (uint32_t)(x))

// ---- Framebuffer ----------------------------------------------------------------------
#define FB_STRIDE 320

// ---- Textures: 16 types x 3 faces (0 top, 1 side, 2 bottom) x 16x16, one byte per texel
#define TEX_INDEX(type, face, u, v) \
    ((((uint32_t)(type) * 3u + (uint32_t)(face)) << 8) | ((uint32_t)(v) << 4) | (uint32_t)(u))
#define TEX_BYTES (16 * 3 * 256)

// ---- MMIO offsets (0x5000_0000 + off) -------------------------------------------------
#define MMIO_FB_SWAP     0x00
#define MMIO_FRAME_COUNT 0x04
#define MMIO_VSYNC       0x08
#define MMIO_CYCLES      0x0C
#define MMIO_UART_RX     0x10
#define MMIO_UART_STATUS 0x14
#define MMIO_UART_TX     0x18
#define MMIO_ERR         0x1C
#define MMIO_ERR_PC      0x20
#define MMIO_LEDS        0x24
#define MMIO_FB_MODE     0x28
#define MMIO_DDR_STATUS  0x2C
#define MMIO_CHUNK_CMD   0x30
#define MMIO_CHUNK_DONE  0x34
#define MMIO_SLOT_VALID  0x38
#define MMIO_WIN_ORIGIN  0x3C

#define CORE_HZ 50000000u   // Q5 proposal: 50 MHz core clock

// ---- Shared state in data memory --------------------------------------------------------
// Camera block, Q16.16, at 0x1000_0000 (frozen layout, issue #2).
typedef struct {
    int32_t origin[3];      // ABSOLUTE world block coordinates (kernels subtract the corner)
    int32_t forward[3];
    int32_t right_step[3];  // right_step[1] must be 0 (no camera roll): the kernel hoists y per row
    int32_t up_step[3];
} kc_camera;

// Render parameters, at 0x1000_0030 (inside "reserved shared state", Kenny-owned).
typedef struct {
    int32_t width, height;      // 160x120 or 320x240
    int32_t max_steps;          // DDA step cap, 1..255
    int32_t sky_k;              // sky row index = (py * sky_k) >> 16, 0..15
    int32_t fog_d[3];           // fog level n starts at t >= fog_d[n-1] (Q16 blocks)
    int32_t right[3], up[3];    // unit camera axes, Q16
    int32_t focal;              // pixels per unit of tangent, Q16
    int32_t corner_x, corner_z; // committed window corner (absolute blocks) for the kernel
} kc_rparams;                   // 64 bytes, ends at 0x1000_0070

#ifdef KC_HOST
// ---------------------------------------------------------------- laptop emulation ----
extern kc_camera  host_camera;
extern kc_rparams host_rparams;
#define KC_CAM (&host_camera)
#define KC_RP  (&host_rparams)

uint8_t  hal_win_get(int32_t x, int32_t y, int32_t z);
void     hal_win_set(int32_t x, int32_t y, int32_t z, uint8_t t);
uint8_t  hal_tex_get(uint32_t idx);
void     hal_fb_put(uint32_t idx, uint8_t v);
uint8_t  hal_fb_get(uint32_t idx);
uint32_t hal_mmio_r(uint32_t off);
void     hal_mmio_w(uint32_t off, uint32_t v);

#else
// ---------------------------------------------------------------- chip ---------------------
#define KC_CAM ((kc_camera *)0x10000000u)
#define KC_RP  ((kc_rparams *)0x10000030u)
#define KC_WORLD ((volatile uint8_t *)0x20000000u)
#define KC_TEX   ((volatile uint8_t *)0x30000000u)
#define KC_FB    ((volatile uint8_t *)0x40000000u)
#define KC_MMIO  ((volatile uint32_t *)0x50000000u)

// Hardware adds WIN_ORIGIN and returns air for invalid slots. Low 4 bits = type.
static inline uint8_t  hal_win_get(int32_t x, int32_t y, int32_t z) { return KC_WORLD[WIN_INDEX(x, y, z)] & 15; }
static inline void     hal_win_set(int32_t x, int32_t y, int32_t z, uint8_t t) { KC_WORLD[WIN_INDEX(x, y, z)] = t; }
static inline uint8_t  hal_tex_get(uint32_t idx) { return KC_TEX[idx]; }
static inline void     hal_fb_put(uint32_t idx, uint8_t v) { KC_FB[idx] = v; }
static inline uint8_t  hal_fb_get(uint32_t idx) { return KC_FB[idx]; }
static inline uint32_t hal_mmio_r(uint32_t off) { return KC_MMIO[off >> 2]; }
static inline void     hal_mmio_w(uint32_t off, uint32_t v) { KC_MMIO[off >> 2] = v; }
#endif

// Debug text over UART_TX (game code only, lane 0).
static inline void hal_puts(const char *s) {
    while (*s) {
        while (hal_mmio_r(MMIO_UART_STATUS) & 2) {}
        hal_mmio_w(MMIO_UART_TX, (uint8_t)*s++);
    }
}

#endif
