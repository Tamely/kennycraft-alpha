// kc_budget.h — the 192 KB rendering + real-time budget, checked at compile time.
// Chunk storage (memory API, up to 256 MB) is outside this budget.
//
// Framebuffers: the two framebuffers live in main memory, so they are listed
// but not counted by default. Build with -DKC_BUDGET_COUNT_FB=1 to count them too; at
// 160x120 (1 byte per pixel, double-buffered) everything still fits.
#ifndef KC_BUDGET_H
#define KC_BUDGET_H
#include "kc_hal.h"

#define KC_BUDGET_BYTES      (192 * 1024)

#define BUDGET_WORLD         (WIN_X * WIN_Y * WIN_Z / 2)           // 4 bits per block, 1 copy
#define BUDGET_TEXTURES      TEX_BYTES                              // 1 byte per texel
#define BUDGET_PALETTE       (256 * 12 / 8)                         // 256 x RGB444 (12-bit VGA), not 16-bit
#define BUDGET_CHUNKS        (16 * 2 + 16 + 16 * 4 + 2 + 4)        // slot tables + tag key (KC_GEN_BITMAP: +8 KB in data memory)
#define BUDGET_FB_160        (2 * 160 * 120)

#ifndef KC_BUDGET_COUNT_FB
#define KC_BUDGET_COUNT_FB 0
#endif

#define BUDGET_TOTAL (BUDGET_WORLD + BUDGET_TEXTURES + BUDGET_PALETTE \
                      + BUDGET_CHUNKS + KC_BUDGET_COUNT_FB * BUDGET_FB_160)

_Static_assert(BUDGET_TOTAL <= KC_BUDGET_BYTES, "rendering memory is over the 192 KB budget");
_Static_assert(BUDGET_TOTAL + BUDGET_FB_160 <= KC_BUDGET_BYTES,
               "over 192 KB if the 160x120 framebuffers end up inside the budget");

#endif
