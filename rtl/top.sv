// KennyCraft top level on the Digilent Arty A7-100T (Tai page, CPU Core,
// Module hierarchy).
//
//   clk (E3, 100 MHz) -> BUFG -> clk_gen  50 MHz core, 25 MHz pixel
//                             -> ddr_clk  166.67 / 200 MHz for the MIG
//   kc_soc      core, imem, dmem, GPU memory, decoder, MMIO, UART, textures
//               (EXT_WORLD = 1: its world port goes through world_remap)
//   display     framebuffers, palette, VGA timing -> Pmod VGA on JA + JB
//   world_remap window origin and slot-valid between the core / VTRACE and
//               world_mem
//   world_mem   two copies (D17); copy n serves VTRACE lanes 4n..4n+3
//   vtrace      HAS_RAY_UNIT = 1: the ray unit behind the core's custom-1
//               instructions (Stage 2), WORLD_LATENCY = 2
//   chunk_engine + ddr_bridge (u_bridge) + mig_ddr3   DDR3 chunk store (D14)
//
// Memory images: IMEM_HEX / DMEM_HEX (program, kc.ld layout), TEX_HEX
// (textures, one texel per line) and PAL_HEX (palette, RGB444 per line).
// fpga/build.tcl writes an .mmi so updatemem can replace all four in a
// built bitstream (see the commands there).
//
// LANES, HAS_DIV_STACK and HAS_RAY_UNIT configure the core: LANES = 8 with the
// divergence stack is Stage 1 (the default, and the KC_TARGET=top board
// build), plus HAS_RAY_UNIT = 1 Stage 2; LANES = 1 without either is Stage 0
// (KC_TARGET=top_s0). HAS_QRCP = 1 adds the per-lane QRCP unit (D22, Stage 2
// top first). FB_COUNT = 3 gives the display three framebuffers (triple
// buffering, D24; the Stage 2 builds) instead of two.
//
// led is the error LED: the core halted on an illegal instruction, a
// misaligned access or a fetch outside instruction memory. The MIG owns the
// XADC (temperature monitor); nothing else may instantiate one.
/* verilator lint_off PINCONNECTEMPTY */  // unused MIG status outputs
module top #(
    parameter int    LANES         = 8,
    parameter bit    HAS_DIV_STACK = 1'b1,
    parameter string IMEM_HEX = "kc_imem.hex",
    parameter string DMEM_HEX = "kc_dmem.hex",
    parameter string TEX_HEX  = "textures.hex",
    parameter string PAL_HEX  = "palette.hex",
    parameter bit    HAS_RAY_UNIT  = 1'b0,
    parameter bit    HAS_QRCP      = 1'b0,
    parameter int    FB_COUNT      = 2
) (
    input  logic        clk,            // E3,  100 MHz
    input  logic        ck_rst,         // C2,  active low
    output logic        uart_rxd_out,   // D10, FPGA -> USB-UART bridge
    input  logic        uart_txd_in,    // A9,  USB-UART bridge -> FPGA (keys)
    output logic        led,            // H5,  error LED

    // Pmod VGA on JA + JB
    output logic [3:0]  vga_r,
    output logic [3:0]  vga_g,
    output logic [3:0]  vga_b,
    output logic        vga_hs,
    output logic        vga_vs,

    // DDR3L (pins and standards come from the MIG's own XDC)
    output logic [13:0] ddr3_addr,
    output logic [2:0]  ddr3_ba,
    output logic        ddr3_cas_n,
    output logic [0:0]  ddr3_ck_n,
    output logic [0:0]  ddr3_ck_p,
    output logic [0:0]  ddr3_cke,
    output logic        ddr3_ras_n,
    output logic        ddr3_reset_n,
    output logic        ddr3_we_n,
    inout  wire  [15:0] ddr3_dq,
    inout  wire  [1:0]  ddr3_dqs_n,
    inout  wire  [1:0]  ddr3_dqs_p,
    output logic [0:0]  ddr3_cs_n,
    output logic [1:0]  ddr3_dm,
    output logic [0:0]  ddr3_odt
);

    localparam int VT_LANES = 8;   // world_mem / world_remap serve 8 VTRACE lanes

    // ------------------------------------------------------------------
    // Clocks and resets
    // ------------------------------------------------------------------
    logic clk100, clk_core, clk_pix, core_locked, rst_core, rst_pix;
    logic clk_sys, clk_ref, ddr_locked;

    BUFG u_bufg_in (.I(clk), .O(clk100));

    clk_gen u_clk_gen (
        .clk(clk100), .rst_n(ck_rst),
        .clk_core, .clk_pix, .locked(core_locked), .rst_core, .rst_pix
    );

    ddr_clk u_ddr_clk (
        .clk_in(clk100), .rst(1'b0), .clk_sys, .clk_ref, .locked(ddr_locked)
    );

    // ------------------------------------------------------------------
    // SoC
    // ------------------------------------------------------------------
    logic        fb_we, fb_re, pal_we, swap_req, swap_pending, vblank, fb_mode;
    logic [16:0] fb_addr;
    logic [7:0]  fb_wdata, fb_rdata, pal_addr;
    logic [11:0] pal_wdata;
    logic [31:0] frame_count;

    logic        ddr_reg_en, ddr_reg_we, ddr_reg_stall;
    logic [5:0]  ddr_reg_off;
    logic [31:0] ddr_reg_wdata, ddr_reg_rdata;

    logic        world_req, world_we;
    logic [16:0] world_addr;
    logic [7:0]  world_wdata;
    logic [3:0]  core_world_rdata;

    logic        vt_cfg_we, vt_start, vt_done;
    logic [2:0]  vt_cfg_field;
    logic [LANES-1:0] vt_lane_mask;
    logic [31:0] vt_cfg_data [LANES];
    logic [7:0]  vt_max_steps;
    logic [15:0] vt_result   [LANES];
    logic [16:0] vt_hit_cell [LANES];
    logic [31:0] vt_t_hit    [LANES];

    kc_soc #(
        .LANES    (LANES),
        .CLK_HZ   (50_000_000),
        .BAUD     (1_000_000),
        .IMEM_HEX (IMEM_HEX),
        .DMEM_HEX (DMEM_HEX),
        .TEX_HEX  (TEX_HEX),
        .EXT_WORLD(1'b1),
        .HAS_DIV_STACK(HAS_DIV_STACK),
        .HAS_RAY_UNIT (HAS_RAY_UNIT),
        .HAS_QRCP     (HAS_QRCP)
    ) u_soc (
        .clk(clk_core), .rst(rst_core),
        .uart_tx(uart_rxd_out), .uart_rx(uart_txd_in),
        .leds(), .err_led(led),
        .fb_we, .fb_re, .fb_addr, .fb_wdata, .fb_rdata,
        .pal_we, .pal_addr, .pal_wdata,
        .swap_req, .swap_pending, .vblank, .frame_count, .fb_mode,
        .ddr_reg_en, .ddr_reg_we, .ddr_reg_off, .ddr_reg_wdata,
        .ddr_reg_rdata, .ddr_reg_stall,
        .world_req, .world_we, .world_addr, .world_wdata,
        .world_ready(world_req), .world_rdata({4'b0, core_world_rdata}),
        .vt_cfg_we, .vt_cfg_field, .vt_lane_mask, .vt_cfg_data, .vt_start, .vt_max_steps,
        .vt_done, .vt_result, .vt_hit_cell, .vt_t_hit
    );

    // ------------------------------------------------------------------
    // Display
    // ------------------------------------------------------------------
    display #(.PAL_HEX(PAL_HEX), .FB_COUNT(FB_COUNT)) u_display (
        .clk_core, .rst_core, .clk_pix, .rst_pix,
        .fb_we, .fb_re, .fb_addr, .fb_wdata, .fb_rdata,
        .pal_we, .pal_addr, .pal_wdata,
        .swap_req, .swap_pending, .vblank, .frame_count, .fb_mode,
        .vga_r, .vga_g, .vga_b, .vga_hs, .vga_vs
    );

    // ------------------------------------------------------------------
    // World window: world_remap -> world_mem (two copies)
    // ------------------------------------------------------------------
    logic [15:0] slot_valid;
    logic [5:0]  win_x, win_z;
    logic [16:0] core_paddr;
    logic [3:0]  a_rdata;
    logic        eng_we, eng_re;
    logic [16:0] eng_addr;
    logic [3:0]  eng_wdata;

    // VTRACE world ports: one window-relative address per lane, data
    // WORLD_LATENCY = 2 cycles later through world_remap and world_mem's
    // two-phase sharing of each copy's ports (D17). Lanes the core does not
    // have read address 0.
    logic        vt_busy;
    logic        vt_phase;      // vtrace WAIT cycle index: world_mem's read phase
    logic [16:0] vt_addr       [VT_LANES];
    logic [16:0] vt_paddr      [VT_LANES];
    logic [3:0]  vt_rdata_phys [VT_LANES];
    logic [3:0]  vt_rdata      [VT_LANES];

    if (HAS_RAY_UNIT) begin : g_vtrace
        logic [16:0] world_addr_l [LANES];
        logic [3:0]  world_data_l [LANES];

        vtrace #(.LANES(LANES), .WORLD_LATENCY(2)) u_vtrace (
            .clk(clk_core), .rst(rst_core),
            .cfg_we(vt_cfg_we), .cfg_field(vt_cfg_field), .lane_mask(vt_lane_mask),
            .cfg_data(vt_cfg_data),
            .start(vt_start), .max_steps(vt_max_steps), .busy(vt_busy), .done(vt_done),
            .result(vt_result), .hit_cell(vt_hit_cell), .t_hit(vt_t_hit),
            .world_addr(world_addr_l), .world_phase(vt_phase), .world_data(world_data_l)
        );

        always_comb begin
            for (int l = 0; l < VT_LANES; l++) vt_addr[l] = '0;
            for (int l = 0; l < LANES; l++) begin
                vt_addr[l]      = world_addr_l[l];
                world_data_l[l] = vt_rdata[l];
            end
        end
    end else begin : g_no_vtrace
        assign vt_busy  = 1'b0;
        assign vt_phase = 1'b0;
        assign vt_done  = 1'b0;
        always_comb begin
            for (int l = 0; l < VT_LANES; l++) vt_addr[l] = '0;
            for (int l = 0; l < LANES; l++) begin
                vt_result[l]   = '0;
                vt_hit_cell[l] = '0;
                vt_t_hit[l]    = '0;
            end
        end
    end

    world_remap #(.LANES(VT_LANES)) u_remap (
        .clk(clk_core), .win_x, .win_z, .slot_valid,
        .core_en(world_req), .core_addr(world_addr), .core_paddr,
        .core_rdata_phys(a_rdata), .core_rdata(core_world_rdata),
        .vt_addr, .vt_paddr, .vt_rdata_phys, .vt_rdata
    );

    // Copy 1 feeds only VTRACE lanes 4..7; without the ray unit synthesis
    // removes it (Stage 0 and 1 builds).
    world_mem #(.LANES(VT_LANES)) u_world (
        .clk(clk_core),
        .vt_busy, .vt_phase, .vt_paddr, .vt_rdata(vt_rdata_phys),
        .core_req(world_req), .core_we(world_we), .core_paddr,
        .core_wdata(world_wdata[3:0]),
        .eng_we, .eng_addr, .eng_wdata,
        .a_rdata
    );

    // ------------------------------------------------------------------
    // DDR3 chunk store
    // ------------------------------------------------------------------
    logic         calib, cmd_valid, cmd_ready, cmd_store;
    logic [15:0]  cmd_chunk;
    logic         wr_valid, wr_ready, rd_valid, rd_ready;
    logic [127:0] wr_data, rd_data;

    chunk_engine u_eng (
        .clk(clk_core), .rst(rst_core),
        .reg_en(ddr_reg_en), .reg_we(ddr_reg_we), .reg_off(ddr_reg_off),
        .reg_wdata(ddr_reg_wdata), .reg_rdata(ddr_reg_rdata), .reg_stall(ddr_reg_stall),
        .slot_valid, .win_x, .win_z,
        .vtrace_busy(vt_busy), .core_world_req(world_req),
        .eng_we, .eng_re, .eng_addr, .eng_wdata, .eng_rdata(a_rdata),
        .calib, .cmd_valid, .cmd_ready, .cmd_store, .cmd_chunk,
        .wr_valid, .wr_ready, .wr_data, .rd_valid, .rd_ready, .rd_data
    );

    logic         ui_clk, ui_rst, init_calib_complete;
    logic [27:0]  app_addr;
    logic [2:0]   app_cmd;
    logic         app_en, app_rdy, app_wdf_end, app_wdf_wren, app_wdf_rdy;
    logic         app_rd_data_valid, app_rd_data_end;
    logic [127:0] app_wdf_data, app_rd_data;
    logic [15:0]  app_wdf_mask;

    ddr_bridge u_bridge (
        .clk(clk_core), .rst(rst_core), .calib, .cmd_valid, .cmd_ready, .cmd_store, .cmd_chunk,
        .wr_valid, .wr_ready, .wr_data, .rd_valid, .rd_ready, .rd_data,
        .ui_clk, .ui_rst, .init_calib_complete, .app_addr, .app_cmd, .app_en, .app_rdy,
        .app_wdf_data, .app_wdf_end, .app_wdf_wren, .app_wdf_mask, .app_wdf_rdy,
        .app_rd_data, .app_rd_data_valid
    );

    mig_ddr3 u_mig (
        .ddr3_addr, .ddr3_ba, .ddr3_cas_n, .ddr3_ck_n, .ddr3_ck_p, .ddr3_cke,
        .ddr3_ras_n, .ddr3_reset_n, .ddr3_we_n, .ddr3_dq, .ddr3_dqs_n, .ddr3_dqs_p,
        .init_calib_complete, .ddr3_cs_n, .ddr3_dm, .ddr3_odt,
        .app_addr, .app_cmd, .app_en, .app_wdf_data, .app_wdf_end, .app_wdf_wren,
        .app_rd_data, .app_rd_data_end, .app_rd_data_valid, .app_rdy, .app_wdf_rdy,
        .app_sr_req(1'b0), .app_ref_req(1'b0), .app_zq_req(1'b0),
        .app_sr_active(), .app_ref_ack(), .app_zq_ack(),
        .ui_clk, .ui_clk_sync_rst(ui_rst), .app_wdf_mask,
        .sys_clk_i(clk_sys), .clk_ref_i(clk_ref), .sys_rst(ddr_locked & ck_rst)
    );

    /* verilator lint_off UNUSEDSIGNAL */
    logic unused;
    /* verilator lint_on UNUSEDSIGNAL */
    always_comb begin
        unused = core_locked ^ eng_re ^ app_rd_data_end ^ (^world_wdata[7:4]);
        for (int l = LANES; l < VT_LANES; l++) unused ^= ^vt_rdata[l];
        if (!HAS_RAY_UNIT) begin
            unused ^= vt_cfg_we ^ vt_start ^ (^vt_cfg_field) ^ (^vt_lane_mask) ^ (^vt_max_steps);
            for (int l = 0; l < LANES; l++) unused ^= (^vt_cfg_data[l]) ^ (^vt_rdata[l]);
        end
    end

endmodule
