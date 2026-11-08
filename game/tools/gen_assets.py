#!/usr/bin/env python3
"""gen_assets.py — KennyCraft textures, output palette and math tables.

Writes into gen/:
  textures.hex     12,288 lines, one byte per texel   ($readmemh, texture BRAM at 0x3000_0000)
                   index = ((type*3 + face) << 8) | (v << 4) | u ; face 0 top, 1 side, 2 bottom
  palette.hex      256 lines, RGB444 (bits 11:8 R, 7:4 G, 3:0 B)  (palette at 0x6000_0000)
  kc_tables.h      1/x seed and sine constants (C)
  kc_assets.h      textures + palette as C arrays (laptop build only)
  atlas.png        preview of every texture, shades and fog levels

Pixel byte: 7:4 color index, 3:2 face shade, 1:0 fog level. Color 15 is sky: its 16 bytes are
a top-to-bottom gradient instead of shades. Deterministic: same seed, same files.
"""
import math, os, random, sys

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "gen")
os.makedirs(OUT, exist_ok=True)

# ---------------------------------------------------------------- 16 base colors ----------
BASE = [
    (24, 22, 28),     # 0  near-black: outlines, holes, UI
    (110, 172, 66),   # 1  grass light
    (74, 128, 46),    # 2  grass dark
    (138, 100, 70),   # 3  dirt light
    (102, 72, 50),    # 4  dirt dark
    (140, 140, 142),  # 5  stone light
    (102, 102, 108),  # 6  stone dark
    (92, 70, 42),     # 7  bark / plank seam
    (182, 146, 92),   # 8  planks
    (222, 210, 152),  # 9  sand
    (50, 98, 38),     # 10 leaves dark
    (156, 64, 50),    # 11 brick
    (192, 188, 180),  # 12 mortar / highlight
    (204, 174, 104),  # 13 log rings, light wood
    (245, 245, 245),  # 14 white UI
    None,             # 15 sky (gradient)
]
SKY_TOP, SKY_HORIZON = (72, 122, 204), (172, 206, 240)
FOG = SKY_HORIZON
SHADE = [1.00, 0.82, 0.68, 0.52]   # top, x sides, z sides, bottom
FOG_MIX = [0.0, 0.35, 0.65, 0.90]

def lerp(a, b, t): return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))

def to444(c):
    return tuple(max(0, min(15, int(round(v / 255 * 15)))) for v in c)

palette_rgb = []
for byte in range(256):
    ci, shade, fog = byte >> 4, (byte >> 2) & 3, byte & 3
    if ci == 15:
        c = lerp(SKY_TOP, SKY_HORIZON, (byte & 15) / 15)
    else:
        c = tuple(v * SHADE[shade] for v in BASE[ci])
        c = lerp(c, FOG, FOG_MIX[fog])
    palette_rgb.append(to444(c))

# ---------------------------------------------------------------- textures ----------------
# Blocky style: every image is drawn at 8 x 8 with few colors, then each pixel is doubled to
# fill the 16 x 16 texture the hardware expects (format and addressing unchanged).
T = 16                      # texture size in memory
S = 8                       # drawing resolution
NTYPES, NFACES = 16, 3
tex = [[[[0] * T for _ in range(T)] for _ in range(NFACES)] for _ in range(NTYPES)]
rng = random.Random(0x4B43)

def new(): return [[0] * S for _ in range(S)]

def noise(colors, weights):
    img = new()
    for v in range(S):
        for u in range(S):
            img[v][u] = rng.choices(colors, weights)[0]
    return img

def up(img):                # 8 x 8 -> 16 x 16, each pixel becomes 2 x 2
    return [[img[v // 2][u // 2] for u in range(T)] for v in range(T)]

def grass_top(): return noise([1, 2], [3, 1])
def dirt():      return noise([3, 4], [3, 1])
def grass_side():
    img = dirt()
    for u in range(S):
        for v in range(2 + (rng.random() < 0.4)):     # green lip, 2-3 pixels deep
            img[v][u] = rng.choices([1, 2], [3, 1])[0]
    return img
def stone():
    img = noise([5, 6], [5, 1])
    for _ in range(2):                                # two short cracks
        u, v = rng.randrange(S), rng.randrange(S)
        img[v][u] = img[v][(u + 1) % S] = 6
    return img
def cobble():
    img = new()
    for v in range(S):
        band = v // 4
        for u in range(S):
            edge = v % 4 == 0 or (u + 2 * band) % 4 == 0
            img[v][u] = (0 if rng.random() < 0.5 else 6) if edge else rng.choices([5, 12], [5, 1])[0]
    return img
def log_side():
    img = new()
    for u in range(S):
        for v in range(S):
            img[v][u] = 7 if u % 4 < 2 else 4
    return img
def log_top():
    img = new()
    for v in range(S):
        for u in range(S):
            d = max(abs(u - 3.5), abs(v - 3.5))
            img[v][u] = 7 if d > 3 else (13 if d > 2 else (8 if d > 1 else 13))
    return img
def leaves(): return noise([10, 2, 0], [5, 3, 1])
def planks():
    img = new()
    for v in range(S):
        for u in range(S):
            seam = v % 4 == 3 or u == (5 if v < 4 else 1)
            img[v][u] = 7 if seam else (13 if rng.random() < 0.12 else 8)
    return img
def sand(): return noise([9, 12], [6, 1])
def brick():
    img = new()
    for v in range(S):
        for u in range(S):
            mortar = v % 4 == 3 or u == (3 if v < 4 else 7)
            img[v][u] = 12 if mortar else (4 if rng.random() < 0.1 else 11)
    return img
def bedrock(): return noise([0, 6, 5], [3, 3, 1])
def missing():
    img = new()
    for v in range(S):
        for u in range(S):
            img[v][u] = 14 if ((u >> 1) ^ (v >> 1)) & 1 else 0
    return img

def same(f): img = up(f()); return (img, img, img)
def faces(top, side, bottom): return (up(top), up(side), up(bottom))

FACES = {
    0: faces(new(), new(), new()),            # air
    1: faces(grass_top(), grass_side(), dirt()),
    2: same(dirt),
    3: same(stone),
    4: same(cobble),
    5: faces(log_top(), log_side(), log_top()),
    6: same(leaves),
    7: same(planks),
    8: same(sand),
    9: same(brick),
    10: same(bedrock),
}
for t in range(NTYPES):
    imgs = FACES.get(t) or same(missing)
    for f in range(NFACES):
        tex[t][f] = imgs[f]

# ---------------------------------------------------------------- editable atlas ----------
# assets/textures.png (if present) overrides the procedural textures. Layout: 48 x 256,
# column = face (top, side, bottom) x 16 px, row = block type x 16 px. Indexed PNG whose
# pixel values ARE the color indices; RGB images are snapped to the nearest base color.
ATLAS_IN = os.path.join(os.path.dirname(__file__), "..", "assets", "textures.png")
BASE_RGB = [c if c else (255, 0, 255) for c in BASE]
if os.path.exists(ATLAS_IN):
    from PIL import Image
    im = Image.open(ATLAS_IN)
    if im.size != (NFACES * T, NTYPES * T):
        sys.exit(f"gen_assets: {ATLAS_IN} must be {NFACES * T}x{NTYPES * T}, got {im.size[0]}x{im.size[1]}")
    indexed = im.mode == "P"
    if not indexed:
        im = im.convert("RGB")
    snapped = 0
    for t in range(NTYPES):
        for f in range(NFACES):
            for v in range(T):
                for u in range(T):
                    p = im.getpixel((f * T + u, t * T + v))
                    if indexed:
                        c = p if p < 15 else 0
                    else:
                        c = min(range(15), key=lambda i: sum((p[k] - BASE_RGB[i][k]) ** 2 for k in range(3)))
                        snapped += p != BASE_RGB[c]
                    tex[t][f][v][u] = c
    print(f"gen_assets: textures from {os.path.abspath(ATLAS_IN)}" + (f" ({snapped} pixels snapped to the palette)" if snapped else ""))

flat = []
for t in range(NTYPES):
    for f in range(NFACES):
        for v in range(T):
            for u in range(T):
                c = tex[t][f][v][u]
                assert 0 <= c < 15, "color 15 is reserved for sky"
                flat.append(c)
assert len(flat) == 12288

# ---------------------------------------------------------------- math tables -------------
# 1/m for m in [1, 2): 256 seeds at bucket midpoints, Q16.16. qrcp() adds one Newton step.
# The chip computes each seed instead of storing the table (fixed.c rcp_seed: linear guess,
# two Newton steps, +1 fix-up); this checks those constants reproduce every entry exactly.
rcp = [int(round(65536 / (1 + (i + 0.5) / 256))) for i in range(256)]
RCP_K0, RCP_K1, RCP_N1, RCP_N2 = 92000, 60, 8, 3
def rcp_seed(i):
    D = 2 * (256 + i) + 1
    Y = RCP_K0 - RCP_K1 * D
    for n in (RCP_N1, RCP_N2):
        E = (1 << 25) - D * Y
        t = Y * (E >> n)
        assert -2**31 <= D * Y < 2**31 and -2**31 <= t < 2**31
        Y += t >> (25 - n)
    return Y + (D * (2 * Y + 1) < (1 << 26))
assert all(rcp_seed(i) == rcp[i] for i in range(256)), "rcp seed constants are off"

# Quarter-wave sine for angles 0..256 of 1024 per turn, Q16.16. The chip evaluates an odd
# degree-9 polynomial in Q29 (fixed.c sin_quarter) that rounds to exactly these values.
sin_t = [int(round(65536 * math.sin(i * 2 * math.pi / 1024))) for i in range(257)]
SIN_C = [843314842, -346799014, 42782736, -2508838, 81191]     # Q29, least squares fit
def sin_quarter(i):
    u = i << 21
    u2 = (u * u) >> 29
    p = SIN_C[4]
    for c in reversed(SIN_C[:4]):
        p = c + ((p * u2) >> 29)
    return (((u * p) >> 29) + (1 << 12)) >> 13
assert all(sin_quarter(i) == sin_t[i] for i in range(257)), "sine polynomial is off"

# ---------------------------------------------------------------- write --------------------
def write(name, text):
    with open(os.path.join(OUT, name), "w") as f:
        f.write(text)

write("textures.hex", "".join(f"{c:02x}\n" for c in flat))
write("palette.hex", "".join(f"{(r << 8) | (g << 4) | b:03x}\n" for r, g, b in palette_rgb))

def c_array(ctype, name, vals, per_line=16):
    lines = []
    for i in range(0, len(vals), per_line):
        lines.append("    " + ", ".join(str(v) for v in vals[i:i + per_line]) + ",")
    return f"static const {ctype} {name}[{len(vals)}] = {{\n" + "\n".join(lines) + "\n};\n"

# Vertex colors: one flat color per (block type, face), the most common color of that face's
# art (on a tie, the color that reached the top count first wins). Both renderers and the hotbar use this table;
# nothing on the chip reads texture memory. Recolor a block by editing its art or this rule.
face_color = []
for t in range(NTYPES):
    for f in range(NFACES):
        cnt, best = [0] * 16, 0
        for v in range(T):
            for u in range(T):
                c = tex[t][f][v][u] & 15
                cnt[c] += 1
                if cnt[c] > cnt[best]:
                    best = c
        face_color.append(best)

# Kernel shading table: the pixel byte without fog (color << 4 | shade << 2) for every VTRACE
# result, indexed by result bits 7:0 = (type << 4) | (face sign << 3) | (face axis << 1) | hit.
# One ANDI + LBU per pixel replaces the per-lane face and shade selects. Same rule as render.c
# kc_shade_pixel: y faces are top (sign 1, ray stepping down, shade 0) or bottom (shade 3),
# x sides shade 1, z sides shade 2. Misses (hit 0) and axis 3 (never produced) are 0.
face_pix = []
for i in range(256):
    hit, ax, neg, t = i & 1, (i >> 1) & 3, (i >> 3) & 1, i >> 4
    if not hit or t == 0 or ax == 3:
        face_pix.append(0)
        continue
    face = (0 if neg else 2) if ax == 1 else 1
    shade = (0 if neg else 3) if ax == 1 else (1 if ax == 0 else 2)
    face_pix.append((face_color[t * 3 + face] << 4) | (shade << 2))

write("kc_tables.h",
      "// Generated by tools/gen_assets.py. Do not edit.\n#ifndef KC_TABLES_H\n#define KC_TABLES_H\n#include <stdint.h>\n"
      + "// Color index per (block type * 3 + face), face 0 top, 1 side, 2 bottom\n"
      + "#define KC_FACE_COLORS { " + ", ".join(str(c) for c in face_color) + " }\n"
      + "// Pixel byte without fog per VTRACE result bits 7:0 (type, face sign, face axis, hit)\n"
      + "#define KC_FACE_PIX { " + ", ".join(str(c) for c in face_pix) + " }\n"
      + "".join(f"#define KC_RCP_{k} {v}\n" for k, v in (("K0", RCP_K0), ("K1", RCP_K1), ("N1", RCP_N1), ("N2", RCP_N2)))
      + "".join(f"#define KC_SIN_C{k} ({v})\n" for k, v in enumerate(SIN_C))
      + "#if KC_RCP_TABLE   // optional: 512 B of data memory for a shorter qrcp\n"
      + c_array("uint16_t", "kc_rcp_table", rcp) + "#endif\n#endif\n")

pal16 = [(r << 8) | (g << 4) | b for r, g, b in palette_rgb]
write("kc_assets.h",
      "// Generated by tools/gen_assets.py. Laptop build only (the chip gets these through $readmemh).\n"
      "#ifndef KC_ASSETS_H\n#define KC_ASSETS_H\n#include <stdint.h>\n"
      + c_array("uint8_t", "kc_textures", flat, 32) + c_array("uint16_t", "kc_palette", pal16) + "#endif\n")

# Editable atlas: indexed PNG, 48 x 256, palette = the 15 base colors (+ magenta for 15).
try:
    from PIL import Image
    at = Image.new("P", (NFACES * T, NTYPES * T), 0)
    pal_flat = []
    for c in BASE_RGB: pal_flat += list(c)
    at.putpalette(pal_flat + [0] * (768 - len(pal_flat)))
    for t in range(NTYPES):
        for f in range(NFACES):
            for v in range(T):
                for u in range(T):
                    at.putpixel((f * T + u, t * T + v), tex[t][f][v][u])
    at.save(os.path.join(OUT, "textures_atlas.png"))
    at.resize((NFACES * T * 8, NTYPES * T * 8), Image.NEAREST).save(os.path.join(OUT, "textures_atlas_x8.png"))
    pal_img = Image.new("P", (15 * 16, 16), 0)        # swatches 0..14 to pick colors from
    pal_img.putpalette(pal_flat + [0] * (768 - len(pal_flat)))
    for i in range(15):
        for y in range(16):
            for x in range(16): pal_img.putpixel((i * 16 + x, y), i)
    pal_img.save(os.path.join(OUT, "texture_palette.png"))
except ImportError:
    pass

# Atlas preview: rows = block types, columns = faces x shade, plus a fog strip and the palette.
try:
    from PIL import Image
    S = 4
    img = Image.new("RGB", (NFACES * 4 * (T + 2) * S, (NTYPES + 2) * (T + 2) * S), (40, 40, 40))
    px = img.load()
    def rgb(byte):
        r, g, b = palette_rgb[byte]
        return (r * 17, g * 17, b * 17)
    for t in range(NTYPES):
        for f in range(NFACES):
            for sh in range(4):
                ox, oy = ((f * 4 + sh) * (T + 2)) * S, (t * (T + 2)) * S
                for v in range(T):
                    for u in range(T):
                        col = rgb((tex[t][f][v][u] << 4) | (sh << 2))
                        for dy in range(S):
                            for dx in range(S):
                                px[ox + u * S + dx, oy + v * S + dy] = col
    oy = NTYPES * (T + 2) * S                         # palette: 64 columns x 4 rows of 13x16 cells
    for b in range(256):
        cx, cy = (b % 64) * 13, oy + (b // 64) * 16
        for dy in range(15):
            for dx in range(12):
                px[cx + dx, cy + dy] = rgb(b)
    img.save(os.path.join(OUT, "atlas.png"))
except ImportError:
    pass

print(f"gen_assets: wrote textures.hex palette.hex kc_tables.h kc_assets.h to {os.path.abspath(OUT)}")
