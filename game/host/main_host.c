// main_host.c — headless laptop front-end.
//   kc_host shots   <dir>   golden camera shots: PNG (x4) + raw back buffer (.fb, 320x240 bytes)
//   kc_host play    <dir>   scripted play-through with checks (physics, editing, chunk round trip)
//   kc_host vectors <dir>   VTRACE test vectors: world.hex + rays.txt from the C reference
//   kc_host bench           render timing and world reads per frame
//   kc_host sweep           arena high water over many locations x frames x edits, plus
//                           a digest of the window (must not change with optimizations)
//   kc_host gencost         terrain generation cost per chunk (terrain_height, window reads/writes)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "kc_hal.h"
#include "hal_host.h"
#include "kc_main.h"
#include "game.h"
#include "chunks.h"
#include "render.h"
#include "trace.h"
#include "blocks.h"
#include "arena.h"
#include "terrain.h"
#define TERRAIN_GEN(cx, cz, x0, z0) terrain_gen_chunk(cx, cz, x0, z0, gen_scratch)

static const char *outdir = ".";
static int failures;

// Tag cells must never be visible: count y = 0 cells that aren't bedrock in slots whose
// SLOT_VALID bit is set (raw BRAM, so a tag nibble of 0 = air is caught too).
static int tag_leaks(void) {
    uint32_t v = host_slot_valid();
    int bad = 0;
    for (uint32_t s = 0; s < 16; s++) {
        if (!((v >> s) & 1)) continue;
        for (uint32_t z = 0; z < 16; z++)
            for (uint32_t x = 0; x < 16; x++)
                bad += host_bram_phys((s & 3) * 16 + x, 0, (s >> 2) * 16 + z) != B_BEDROCK;
    }
    return bad;
}
static uint64_t swap_scans, swap_leaks, hit_scans, hit_leaks;

void host_on_swap(void) { swap_scans++; swap_leaks += (uint64_t)tag_leaks(); }

// world_update, then (if a load's tag matched) check before anything else can look.
static void update_checked(int x, int z) {
    uint32_t h0 = world_stats_tag_hits();
    world_update(x, z);
    if (world_stats_tag_hits() != h0) { hit_scans++; hit_leaks += (uint64_t)tag_leaks(); }
}

static void check(int ok, const char *what) {
    printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

static void save_frame(const char *name) {
    char path[512];
    int w = KC_RP->width, h = KC_RP->height;
    snprintf(path, sizeof path, "%s/%s.png", outdir, name);
    png_write_fb(path, host_front_buffer(), host_palette(), w, h, w == 160 ? 4 : 2);
    snprintf(path, sizeof path, "%s/%s.fb", outdir, name);
    FILE *f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "kc_host: cannot write %s (does the folder exist?)\n", path); exit(2); }
    fwrite(host_front_buffer(), 1, 320 * 240, f);
    fclose(f);
}

static void step_frames(int n) {
    for (int i = 0; i < n; i++) {
        host_advance_cycles(CORE_HZ / 30);
        kc_frame();
    }
}

static void key(uint8_t code, int down) { host_push_key((uint8_t)(code | (down ? 0x80 : 0))); }

// Teleport, then run world_update once per "frame" until the window is complete.
// Returns how many frames that took (at most one chunk generation per frame).
static int pose(q16 x, q16 y, q16 z, q16 yaw, q16 pitch) {
    kc_me.pos[0] = x; kc_me.pos[1] = y; kc_me.pos[2] = z;
    kc_me.yaw = yaw; kc_me.pitch = pitch; kc_me.vel_y = 0;
    int frames = 0;
    do { arena_reset(); update_checked(qfloor(x), qfloor(z)); frames++; } while (!world_ready() && frames < 200);
    return frames;
}

// The kernel (incremental directions, shading table, raster block order) must give exactly
// the pixels of the from-scratch reference kc_shade_pixel.
static void check_kernel(const char *name) {
    int32_t w = KC_RP->width, h = KC_RP->height, bad = 0;
    uint64_t reads = hstats.world_reads;              // keep the reference's reads out of the stats
    for (int32_t y = 0; y < h; y++)
        for (int32_t x = 0; x < w; x++)
            bad += hal_fb_get((uint32_t)(y * FB_STRIDE + x)) != kc_shade_pixel(x, y);
    hstats.world_reads = reads;
    if (bad) { fprintf(stderr, "kc_host: %s: %d pixels differ from kc_shade_pixel\n", name, bad); exit(1); }
}

// Render one frame at a fixed pose without running physics.
static void shot(const char *name) {
    arena_reset();
    write_camera();
    kc_draw_world();
    check_kernel(name);
    game_draw_overlay();
    hal_mmio_w(MMIO_FB_SWAP, 1);
    save_frame(name);
}

static int cmd_shots(void) {
    struct { const char *name; double x, y, z, yaw, pitch; int w; } s[] = {
        { "g0_spawn",     0, 0, 0, 128, -40, 160 },
        { "g1_horizon",   0, 0, 0, 300,   0, 160 },
        { "g2_down",      0, 0, 0, 0,  -200, 160 },
        { "g3_up",        0, 0, 0, 700,  120, 160 },
        { "g4_high",      0, 6, 0, 900,  -60, 160 },
        { "g5_spawn_320", 0, 0, 0, 128,  -40, 320 },
    };
    q16 bx = kc_me.pos[0], by = kc_me.pos[1], bz = kc_me.pos[2];
    for (unsigned i = 0; i < sizeof s / sizeof s[0]; i++) {
        kc_set_resolution(s[i].w);
        pose(bx + (q16)(s[i].x * 65536), by + (q16)(s[i].y * 65536), bz + (q16)(s[i].z * 65536),
             (q16)(s[i].yaw * 65536), (q16)(s[i].pitch * 65536));
        uint64_t r0 = hstats.world_reads;
        shot(s[i].name);
        printf("  %-14s %dx%d  world reads %llu\n", s[i].name, s[i].w, s[i].w * 3 / 4,
               (unsigned long long)(hstats.world_reads - r0));
    }
    kc_set_resolution(160);
    return 0;
}

static int box_overlap_any(void) {
    for (int dy = 0; dy < 2; dy++) {
        int32_t y = qfloor(kc_me.pos[1] + Q16(0.9) * dy);
        if (world_get(qfloor(kc_me.pos[0]), y, qfloor(kc_me.pos[2])) != B_AIR) return 1;
    }
    return 0;
}

static int cmd_play(void) {
    printf("physics\n");
    step_frames(60);
    check(kc_me.on_ground, "player lands on the ground after spawning");
    q16 y_ground = kc_me.pos[1];
    check((kc_me.pos[1] & 0xFFFF) == 0, "feet rest exactly on a block top");
    save_frame("p0_start");

    key(KEY_SPACE, 1); step_frames(8); key(KEY_SPACE, 0);
    check(kc_me.pos[1] > y_ground + Q16(0.9), "jump rises about a block");
    step_frames(40);
    check(kc_me.on_ground, "lands again after the jump");

    printf("editing\n");
    key('9', 1); key('9', 0);
    kc_me.pitch = Q16(-110.0);                         // look at the ground a couple of blocks ahead
    step_frames(1);
    check(kc_me.aim_hit, "aiming at the ground ahead");
    {   // selection box: same pose with and without an aimed block must differ only by the box
        static uint8_t with_box[320 * 240];
        shot("p1_aim");
        memcpy(with_box, host_front_buffer(), sizeof with_box);
        kc_me.aim_hit = 0;
        shot("p1_noaim");
        kc_me.aim_hit = 1;
        const uint8_t *no_box = host_front_buffer();
        int32_t w = KC_RP->width, h = KC_RP->height, drawn = 0, other = 0;
        for (int32_t y = 0; y < h; y++)
            for (int32_t x = 0; x < w; x++) {
                int32_t i = y * FB_STRIDE + x;
                if (with_box[i] != no_box[i]) { if (with_box[i] == PIX(COLOR_BLACK, 0, 0)) drawn++; else other++; }
            }
        printf("  selection box: %d px\n", drawn);
        check(drawn > 20 && other == 0, "selection box outlines the aimed block (black pixels only)");
    }
    {
        int32_t px = kc_me.place[0], py = kc_me.place[1], pz = kc_me.place[2];
        key(KEY_PLACE, 1); key(KEY_PLACE, 0); step_frames(1);
        check(world_get(px, py, pz) == B_BRICK, "K places brick (slot 9) on the aimed face");
        save_frame("p1_placed");
        key(KEY_BREAK, 1); key(KEY_BREAK, 0); step_frames(1);
        check(world_get(px, py, pz) == B_AIR, "J breaks it again");
    }
    kc_me.pitch = Q16(-200.0);                         // look down at the feet
    step_frames(1);
    check(kc_me.aim_hit, "aiming at the block under the player");
    int32_t ax = kc_me.aim[0], ay = kc_me.aim[1], az = kc_me.aim[2];
    uint8_t before = world_get(ax, ay, az);
    key(KEY_BREAK, 1); key(KEY_BREAK, 0); step_frames(1);
    check(before != B_AIR && world_get(ax, ay, az) == B_AIR, "J breaks the aimed block");
    step_frames(30);
    check(kc_me.pos[1] < y_ground, "player falls into the hole");
    kc_me.pitch = Q16(-200.0);
    step_frames(1);
    check(!kc_me.aim_hit || world_get(kc_me.place[0], kc_me.place[1], kc_me.place[2]) == B_AIR,
          "placing into our own cell is refused");
    key(KEY_PLACE, 1); key(KEY_PLACE, 0); step_frames(1);
    check(!box_overlap_any(), "player is not inside a block after K at the feet");

    printf("chunk round trip\n");
    // Leave a marker tower at a known spot, walk far enough that its chunk leaves the window,
    // come back, and make sure it was stored to DDR3 and loaded back.
    int32_t mx = qfloor(kc_me.pos[0]) + 3, mz = qfloor(kc_me.pos[2]);
    for (int32_t y = 26; y <= 28; y++) world_set(mx, y, mz, B_BRICK);
    uint64_t stores0 = hstats.chunk_stores, loads0 = hstats.chunk_loads;
    uint32_t gen0 = world_stats_generated(), miss0 = world_stats_tag_misses();
    printf("  ddr3 power-on fill: %s, %s%s\n", host_ddr_fill_name(),
           KC_GEN_BITMAP ? "8 KB generated bitmap" : "chunk tags",
           KC_GEN_BITMAP ? "" : KC_TAG_PERSIST ? " (KC_TAG_PERSIST: no boot nonce)" : " with boot nonce");
    q16 home[3] = { kc_me.pos[0], kc_me.pos[1], kc_me.pos[2] };
    int far_frames = pose(home[0] + Q16(200.0), Q16(30.0), home[2], 0, 0);
    check(world_get(mx, 27, mz) == B_AIR, "marker chunk is out of the window");
    check(world_stats_generated() - gen0 == 16, "16 new chunks generated far away");
    check(far_frames >= 16, "at most one generation per frame after startup");
    if (!world_storage_ok())
        check(hstats.chunk_loads == loads0, "power-on DDR3: fallback never loads, nothing from DDR3 accepted");
    else if (KC_GEN_BITMAP)
        check(hstats.chunk_loads == loads0, "power-on DDR3: bitmap never loads a chunk it didn't generate");
    else
        check(world_stats_tag_misses() - miss0 == 16 && hstats.chunk_loads - loads0 == 16,
              "power-on DDR3: 16 loads of never-stored chunks, no tag accepted, all generated");
    int back_frames = pose(home[0], home[1], home[2], 0, 0);
    if (world_storage_ok()) {
        check(hstats.chunk_stores >= stores0 + 16, "evicted chunks (generated = dirty) were stored");
        check(hstats.chunk_loads >= loads0 + 16, "coming back loads all 16 chunks instead of generating");
        check(world_stats_generated() - gen0 == 16, "nothing regenerated on the way back");
        check(world_get(mx, 27, mz) == B_BRICK, "marker survived leaving and coming back");
        check(world_get(ax, ay, az) == B_AIR, "the broken block is still broken");
    } else {
        check(world_get(mx, 27, mz) == B_AIR, "fallback: chunk regenerated, edit gone (issue #2 rule)");
    }
    {
        int bedrock = 1;
        for (int32_t z = mz & ~15; z < (mz & ~15) + 16; z++)
            for (int32_t x = mx & ~15; x < (mx & ~15) + 16; x++) bedrock &= world_get(x, 0, z) == B_BEDROCK;
        check(bedrock, "marker chunk has its full bedrock layer (tag cells restored)");
    }

    // A stored chunk's bytes at another chunk's DDR3 address (addressing mix-up) must not be
    // accepted: the tag names the chunk. The copy target was never stored this boot.
    {
        int32_t mcx = mx >> 4, mcz = mz >> 4, dcx = mcx + 30;
        host_ddr_copy((uint32_t)(mcz * 256 + mcx), (uint32_t)(mcz * 256 + dcx));
        uint32_t gen1 = world_stats_generated();
        pose(home[0] + Q16(30.0 * 16), home[1], home[2], 0, 0);
        check(world_stats_generated() - gen1 == 16, "misaddressed chunk: copy at the wrong address is regenerated");
        check(world_get(mx + 30 * 16, 27, mz) == B_AIR, "misaddressed chunk: the copy's marker did not appear");
        pose(home[0], home[1], home[2], 0, 0);
        check(world_get(mx, 27, mz) == (world_storage_ok() ? B_BRICK : B_AIR),
              world_storage_ok() ? "marker still there after a second round trip" : "fallback: marker gone after a second round trip");
    }
    printf("  storage: %s | far window %d frames, back %d frames | %llu stores, %llu loads, %u generated\n",
           world_storage_ok() ? "DDR3 chunk store" : "fallback (no DDR3)", far_frames, back_frames,
           (unsigned long long)hstats.chunk_stores, (unsigned long long)hstats.chunk_loads,
           (unsigned)world_stats_generated());
    printf("  arena high water: %u of %u bytes\n", (unsigned)arena_high_water(), (unsigned)ARENA_SIZE);

    printf("walk\n");
    kc_me.pos[1] = Q16(30.0); kc_me.vel_y = 0;          // drop in from above (fallback may have refilled the hole)
    step_frames(45);
    q16 z0 = kc_me.pos[2];
    kc_me.yaw = 0; kc_me.pitch = Q16(-20.0);
    key(KEY_W, 1); key(KEY_SPACE, 1); step_frames(90); key(KEY_W, 0); key(KEY_SPACE, 0);
    q16 moved = kc_me.pos[2] - z0;
    printf("  walked %.2f blocks in 3 s (blocked by terrain is fine)\n", moved / 65536.0);
    check(moved > 0, "W moves forward");
    save_frame("p2_walk");

    printf("reset\n");
    // Edit a block, move away so its chunk is stored, press reset (DDR3 keeps its contents,
    // CYCLES restarts with different jitter, so a new boot nonce), boot at spawn again.
    {
        int32_t rx = 2048 + 5, rz = 2048 + 3;                // inside the spawn window
        pose(Q16_INT(rx), Q16(30.0), Q16_INT(rz), 0, 0);
        world_set(rx, 27, rz, B_PLANKS);
        check(world_get(rx, 27, rz) == B_PLANKS, "reset test: edit placed");
        uint64_t st0 = hstats.chunk_stores;
        pose(Q16_INT(rx) + Q16(200.0), Q16(30.0), Q16_INT(rz), 0, 0);
        check(!world_storage_ok() || hstats.chunk_stores > st0, "reset test: edited chunk stored on leaving");
        host_reset(0x5A17u);
        world_reboot();
        kc_init();                                         // game_init: first world_update at spawn
        uint64_t hs0 = hit_scans, hl0 = hit_leaks;
        if (world_stats_tag_hits()) { hit_scans++; hit_leaks += (uint64_t)tag_leaks(); }
        pose(Q16_INT(rx), Q16(30.0), Q16_INT(rz), 0, 0);
        int persist = world_storage_ok() && !KC_GEN_BITMAP && KC_TAG_PERSIST;
        uint32_t g = world_stats_generated(), hits = world_stats_tag_hits();
        printf("  after reset: %u generated, %u tag hits, %u tag misses\n", (unsigned)g, (unsigned)hits,
               (unsigned)world_stats_tag_misses());
        if (persist) {
            check(world_get(rx, 27, rz) == B_PLANKS, "KC_TAG_PERSIST: same seed after reset reloads, edit survives");
            check(hits == 16 && g == 0, "KC_TAG_PERSIST: all 16 chunks tag-matched, none generated");
            check(hit_scans > hs0 && hit_leaks == hl0, "KC_TAG_PERSIST: y = 0 all bedrock right after the boot's tag-matched loads");
        } else if (world_storage_ok()) {
            check(world_get(rx, 27, rz) == B_AIR, "same seed after reset regenerates instead of reloading: edit gone");
            check(g == 16 && hits == 0, KC_GEN_BITMAP ? "after reset: all 16 chunks generated (bitmap starts empty)"
                                                      : "after reset: all 16 chunks generated, no tag from the old boot accepted");
        } else {
            check(world_get(rx, 27, rz) == B_AIR, "fallback: same seed after reset regenerates, edit gone");
            check(g == 16 && hstats.chunk_loads == 0, "fallback: after reset all 16 chunks generated, nothing loaded");
        }
    }

    // Reset again with exactly the same CYCLES jitter as the last boot. The nonce must still
    // change (DDR3 boot counter), or chunks stored by the last boot would be trusted.
    if (world_storage_ok() && !KC_GEN_BITMAP && !KC_TAG_PERSIST) {
        int32_t rx = 2048 + 5, rz = 2048 + 3;
        uint32_t n1 = world_boot_nonce();
        world_set(rx, 27, rz, B_PLANKS);
        pose(Q16_INT(rx) + Q16(200.0), Q16(30.0), Q16_INT(rz), 0, 0);   // store the spawn window
        host_reset(0x5A17u);                               // same jitter as the reset above
        world_reboot();
        kc_init();
        uint32_t n2 = world_boot_nonce();
        printf("  same-jitter reset: nonce 0x%08x -> 0x%08x\n", (unsigned)n1, (unsigned)n2);
        check(n1 != n2, "same-jitter reset: boot nonce still changes (boot counter)");
        pose(Q16_INT(rx), Q16(30.0), Q16_INT(rz), 0, 0);
        check(world_get(rx, 27, rz) == B_AIR && world_stats_tag_hits() == 0,
              "same-jitter reset: no tag from the previous boot accepted, edit gone");
        // Visit the world corner: chunk (0, 0) is generated (bedrock over the counter cells)
        // and stored on leaving, so its tag write must put the counter back.
        uint32_t c2 = world_boot_count();
        pose(Q16(8.0), Q16(30.0), Q16(8.0), 0, 0);
        pose(Q16_INT(rx), Q16(30.0), Q16_INT(rz), 0, 0);
        host_reset(0x5A17u);
        world_reboot();
        kc_init();
        printf("  boot counter across a corner visit: %u -> %u\n", (unsigned)c2, (unsigned)world_boot_count());
        check(world_boot_count() == c2 + 1, "boot counter survives chunk (0, 0) being generated and stored in play");
        check(world_boot_nonce() != n2, "nonce changes again after the corner visit");
    }

    printf("tag visibility\n");
    check(tag_leaks() == 0, "window settled: every y = 0 cell of every valid slot is bedrock");
    printf("  %llu frame swaps and %llu world_updates with tag-matched loads checked\n",
           (unsigned long long)swap_scans, (unsigned long long)hit_scans);
    check(swap_leaks == 0, "every frame swap: y = 0 of every valid slot is bedrock");
    if (world_storage_ok() && !KC_GEN_BITMAP)
        check(hit_scans > 0 && hit_leaks == 0, "right after each tag-matched load: y = 0 of every valid slot is bedrock");
    check(hstats.stores_from_valid == 0, "every chunk_store came from a slot whose SLOT_VALID bit was clear");

    printf("%s\n", failures ? "SOME CHECKS FAILED" : "all checks passed");
    return failures != 0;
}

// VTRACE contract cases (wiki Tai page, "One step"), checked against vtrace_ref.
static kc_ray contract_ray(q16 tx, q16 ty, q16 tz, q16 dx, q16 dy, q16 dz) {
    kc_ray r;
    r.f[0] = CELL_PACK(10, 20, 10);                     // open air above the spawn terrain
    r.f[1] = (uint32_t)tx; r.f[2] = (uint32_t)ty; r.f[3] = (uint32_t)tz;
    r.f[4] = (uint32_t)dx; r.f[5] = (uint32_t)dy; r.f[6] = (uint32_t)dz;
    r.f[7] = 0;
    return r;
}

static int cmd_contract(void) {
    kc_hit h;
    kc_ray r;
    printf("vtrace contract\n");
    r = contract_ray(0x8000, 0x8000, 0x10000, Q16_ONE, Q16_ONE, Q16_ONE);
    vtrace_ref(&r, 1, &h);
    check(HIT_AXIS(h.res) == 1, "tie x == y < z steps y (strict <)");
    r = contract_ray(0x8000, 0x10000, 0x8000, Q16_ONE, Q16_ONE, Q16_ONE);
    vtrace_ref(&r, 1, &h);
    check(HIT_AXIS(h.res) == 2, "tie x == z < y steps z");
    r = contract_ray(0x8000, 0x8000, 0x8000, Q16_ONE, Q16_ONE, Q16_ONE);
    vtrace_ref(&r, 1, &h);
    check(HIT_AXIS(h.res) == 2, "three-way tie steps z");
    r = contract_ray(0x7FFF0000, Q16_ONE, 0x7FFFFFFF, 0x10000, 0x7FFFFFFF, 0x7FFFFFFF);
    vtrace_ref(&r, 2, &h);       // step 1: y (smallest); tMax.y += 0x7FFF_FFFF saturates instead of
                                 // wrapping negative, so step 2 is x, not y again
    check(HIT_AXIS(h.res) == 0 && HIT_STEPS(h.res) == 2, "tMax saturates at 0x7FFF_FFFF (no wrap)");
    q16 o[3] = { Q16(10.5), Q16(20.5), Q16(10.5) }, d[3] = { 0, -Q16_ONE, 0 };
    ray_setup(&r, o, d);
    check(r.f[4] == 0x7FFFFFFF && r.f[6] == 0x7FFFFFFF, "zero direction component gets tDelta 0x7FFF_FFFF");
    vtrace_ref(&r, 64, &h);
    check(HIT_AXIS(h.res) == 1 && CELL_X(h.cell) == 10 && CELL_Z(h.cell) == 10, "straight-down ray never steps x or z");
    kc_hit before = { 0x1234, 0x5678, 0x9ABC };
    h = before;
    vtrace_ref(&r, 0, &h);
    check(h.res == before.res && h.cell == before.cell && h.t == before.t, "max_steps 0 leaves results unchanged");
    printf("%s\n", failures ? "SOME CHECKS FAILED" : "all checks passed");
    return failures != 0;
}

static int cmd_vectors(void) {
    char path[512];
    snprintf(path, sizeof path, "%s/world.hex", outdir);
    FILE *w = fopen(path, "w");
    for (int y = 0; y < 32; y++)                       // window-relative order, WIN_ORIGIN = 0
        for (int z = 0; z < 64; z++)
            for (int x = 0; x < 64; x++)
                fprintf(w, "%x\n", hal_win_get(x, y, z));
    fclose(w);

    snprintf(path, sizeof path, "%s/rays.txt", outdir);
    FILE *f = fopen(path, "w");
    fprintf(f, "# f0 f1 f2 f3 f4 f5 f6 f7 max_steps -> res cell t   (hex, from vtrace_ref)\n");
    uint32_t seed = 1;
    int n = 0;
    for (int i = 0; i < 2000; i++) {
        seed = seed * 1664525u + 1013904223u;
        q16 o[3] = { (q16)(4 + (seed >> 8) % 56) << 16, 0, 0 };
        seed = seed * 1664525u + 1013904223u; o[1] = (q16)((18 + (seed >> 8) % 12) << 16);
        seed = seed * 1664525u + 1013904223u; o[2] = (q16)((4 + (seed >> 8) % 56) << 16);
        for (int k = 0; k < 3; k++) { seed = seed * 1664525u + 1013904223u; o[k] |= (seed >> 12) & 0xFFFF; }
        q16 d[3];
        for (int k = 0; k < 3; k++) { seed = seed * 1664525u + 1013904223u; d[k] = (q16)((seed >> 14) & 0x1FFFF) - 0x10000; }
        if (i < 6) { d[0] = d[1] = d[2] = 0; d[i % 3] = i < 3 ? Q16_ONE : -Q16_ONE; }   // axis-aligned
        int steps = i == 6 ? 1 : (i % 7 == 0 ? 8 : 160);
        kc_ray r; kc_hit h;
        ray_setup(&r, o, d);
        vtrace_ref(&r, steps, &h);
        for (int k = 0; k < 8; k++) fprintf(f, "%08x ", r.f[k]);
        fprintf(f, "%02x -> %04x %05x %08x\n", steps, h.res, h.cell, (uint32_t)h.t);
        n++;
    }
    fclose(f);
    printf("  wrote %s/world.hex and %d rays to %s/rays.txt\n", outdir, n, outdir);
    return 0;
}

static int cmd_bench(void) {
    for (int w = 160; w <= 320; w += 160) {
        kc_set_resolution(w);
        uint64_t r0 = hstats.world_reads;
        clock_t c0 = clock();
        for (int i = 0; i < 20; i++) { write_camera(); kc_draw_world(); }
        double ms = (double)(clock() - c0) * 1000.0 / CLOCKS_PER_SEC / 20;
        double reads = (double)(hstats.world_reads - r0) / 20;
        printf("  %dx%d: %.2f ms/frame on the laptop, %.0f DDA steps/frame (%.1f per pixel)\n",
               w, w * 3 / 4, ms, reads, reads / (w * w * 3 / 4));
    }
    return 0;
}


// ---------------------------------------------------------------- sweep / gencost --------
static uint32_t lcg_state = 12345;
static uint32_t lcg(void) { lcg_state = lcg_state * 1664525u + 1013904223u; return lcg_state >> 8; }
static uint64_t fnv(uint64_t h, uint32_t v) { for (int i = 0; i < 4; i++) { h ^= (v >> (8 * i)) & 255; h *= 1099511628211ull; } return h; }

static uint64_t window_digest(uint64_t h) {
    int32_t x0 = world_corner_x(), z0 = world_corner_z();
    for (int32_t y = 0; y < 32; y++)
        for (int32_t z = 0; z < 64; z++)
            for (int32_t x = 0; x < 64; x += 4) {
                uint32_t v = 0;
                for (int k = 0; k < 4; k++) v |= (uint32_t)world_get(x0 + x + k, y, z0 + z) << (8 * k);
                h = fnv(h, v);
            }
    return h;
}

static void tap(uint8_t code) { key(code, 1); key(code, 0); }

static int cmd_sweep(void) {
    uint64_t wd = 1469598103934665603ull;
    int locations = 48, frames = 0, edits = 0;
    uint32_t worst_loc = 0, worst = 0;
    for (int l = 0; l < locations; l++) {
        int32_t x = 64 + (int32_t)(lcg() % 3968), z = 64 + (int32_t)(lcg() % 3968);
        if (l == 0) { x = qfloor(kc_me.pos[0]); z = qfloor(kc_me.pos[2]); }
        if (l % 6 == 5) { x = (x & ~15) + 15; z = (z & ~15); }   // chunk corners
        kc_me.pos[0] = Q16_INT(x) + Q16_HALF; kc_me.pos[2] = Q16_INT(z) + Q16_HALF;
        kc_me.pos[1] = Q16(30.0); kc_me.vel_y = 0;
        int n = 0;
        do { step_frames(1); n++; kc_me.pos[1] = Q16(30.0); kc_me.vel_y = 0; } while (!world_ready() && n < 100);
        kc_me.pos[1] = Q16_INT(terrain_height(x, z) + 1);
        for (int f = 0; f < 48; f++) {
            kc_me.yaw = (q16)(lcg() % 1024) << 16;
            kc_me.pitch = (q16)((int32_t)(lcg() % 400) - 250) << 16;
            if (f % 3 == 0) tap((uint8_t)('1' + lcg() % 9));
            if (f % 3 == 1) { tap(KEY_BREAK); edits++; }
            if (f % 3 == 2) { tap(KEY_PLACE); edits++; }
            if (f % 8 == 0) {                               // direct edits on chunk borders
                for (int k = 0; k < 6; k++) {
                    int32_t bx = world_corner_x() + (int32_t)(lcg() % 64), bz = world_corner_z() + (int32_t)(lcg() % 64);
                    if (k & 1) bx = (bx & ~15) + (k & 2 ? 15 : 0);
                    else       bz = (bz & ~15) + (k & 2 ? 15 : 0);
                    world_set(bx, 1 + (int32_t)(lcg() % 31), bz, (uint8_t)(lcg() % 16));
                    edits++;
                }
            }
            if (f == 40) key(KEY_W, 1);
            if (f == 47) key(KEY_W, 0);
            step_frames(1);
            n++;
        }
        frames += n;
        wd = window_digest(wd);
        if (arena_high_water() > worst) { worst = arena_high_water(); worst_loc = (uint32_t)l; }
    }
    printf("  sweep: %d locations, %d frames, %d edits, %u chunks generated\n",
           locations, frames, edits, (unsigned)world_stats_generated());
    printf("  arena high water: %u of %u bytes (first reached at location %u)\n",
           (unsigned)arena_high_water(), (unsigned)ARENA_SIZE, (unsigned)worst_loc);
    printf("  window digest %016llx\n", (unsigned long long)wd);
    return 0;
}

static int cmd_gencost(void) {
    int n = 256;
    arena_reset();
    void *gen_scratch = arena_alloc(TERRAIN_SCRATCH_BYTES);
    uint64_t wd = 1469598103934665603ull;
    uint32_t th0 = terrain_height_calls;
    uint64_t r0 = hstats.world_reads, w0 = hstats.world_writes;
    clock_t c0 = clock();
    for (int i = 0; i < n; i++) {
        int32_t cx = (int32_t)(lcg() % 256), cz = (int32_t)(lcg() % 256);
        if (i < 4) { cx = i & 1 ? 255 : 0; cz = i & 2 ? 255 : 0; }   // world edges
        TERRAIN_GEN(cx, cz, world_corner_x(), world_corner_z());
        uint64_t r1 = hstats.world_reads;
        int32_t x0 = (cx * 16 - world_corner_x()) & 63, z0 = (cz * 16 - world_corner_z()) & 63;
        for (int32_t y = 0; y < 32; y++)
            for (int32_t z = 0; z < 16; z++)
                for (int32_t x = 0; x < 16; x++) wd = fnv(wd, hal_win_get((x0 + x) & 63, y, (z0 + z) & 63));
        r0 += hstats.world_reads - r1;                  // don't count the digest reads
    }
    double ms = (double)(clock() - c0) * 1000.0 / CLOCKS_PER_SEC;
    printf("  gencost: %d chunks | per chunk: terrain_height %.1f, hal_win_set %.1f, hal_win_get %.1f | %.1f ms total (incl. digest)\n",
           n, (double)(terrain_height_calls - th0) / n, (double)(hstats.world_writes - w0) / n,
           (double)(hstats.world_reads - r0) / n, ms);
    printf("  chunk digest %016llx\n", (unsigned long long)wd);
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: kc_host shots|play|vectors <dir> | contract | bench | sweep | gencost\n");
        return 2;
    }
    if (argc > 2) outdir = argv[2];
    host_init(0);
    kc_init();
    if (!strcmp(argv[1], "shots"))   return cmd_shots();
    if (!strcmp(argv[1], "play"))    return cmd_play();
    if (!strcmp(argv[1], "vectors")) return cmd_vectors();
    if (!strcmp(argv[1], "contract")) return cmd_contract();
    if (!strcmp(argv[1], "bench"))   return cmd_bench();
    if (!strcmp(argv[1], "sweep"))   return cmd_sweep();
    if (!strcmp(argv[1], "gencost")) return cmd_gencost();
    fprintf(stderr, "unknown command %s\n", argv[1]);
    return 2;
}
