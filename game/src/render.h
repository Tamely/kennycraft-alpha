// render.h — the render kernel (the only GPU kernel).
#ifndef KC_RENDER_H
#define KC_RENDER_H
#include <stdint.h>

// Pixel byte (frozen): 7:4 color index, 3:2 face shade, 1:0 fog level.
// Color index 15 is reserved for sky: its 16 bytes are a vertical gradient, not shades.
#define PIX(color, shade, fog) ((uint8_t)(((color) << 4) | ((shade) << 2) | (fog)))
#define COLOR_SKY   15
#define COLOR_WHITE 14
#define COLOR_BLACK 0

// Vertex colors (no textures): one color index per (block type, face), face 0 top, 1 side,
// 2 bottom. Generated from the block art by tools/gen_assets.py (gen/kc_tables.h).
extern const uint8_t kc_face_color[16 * 3];

// Render the whole frame from KC_CAM / KC_RP into the back framebuffer.
// Kernel: call with all lanes active (mask_all(); kc_render_frame(); mask_set(1);).
// Each pass of the 8 lanes shades a 4x2 block (D15); blocks go in raster order (no cache, so
// tiles bought nothing). Per frame state stays in registers: see the cost model in render.c.
void kc_render_frame(void);

// Shade one pixel (exposed for tests and the single-pixel debug path).
uint8_t kc_shade_pixel(int32_t px, int32_t py);

#endif
