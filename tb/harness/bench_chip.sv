// Chip composition for the bench harness only (make vl, check-bench).
//
// rtl/top.sv has no LANES or HAS_DIV_STACK parameter, so Stage 1 (8 lanes,
// divergence stack) cannot run on it; bench_chip is the same composition as
// rtl/top.sv with those two parameters. It instantiates the shipped RTL:
// clk_gen + kc_soc + display + world_remap + world_mem (two copies, D17) +
// chunk_engine + ddr_bridge + MIG, with TEX_HEX / PAL_HEX passed to kc_soc
// and display as top.sv does. The ports match top.sv, so the harness binds to
// either (CHIP_TOP=top). Delete this file once top.sv takes those parameters.
//
// Stub limits (tb/harness/stubs, also used for CHIP_TOP=top):
//   clk_gen  clk_core IS the harness clk (the harness counts core cycles:
//            50 per UART bit, fps at 50 MHz); clk_pix = clk / 2. Reset
//            releases combinationally from ck_rst with locked = 1, so the
//            real clk_gen reset synchronizers never run.
//   ddr_clk  clk_sys = clk_ref = clk, so with the MIG stub ui_clk == clk_core:
//            the ddr_bridge async FIFOs never see unrelated clocks
//            (tb/ddr runs ui_clk separately for that).
//   mig_ddr3 ddr_model behind the MIG user interface, no DDR3 pins.
//
// Ports (Arty A7-100T pins, as in rtl/top.sv):
//   clk          harness clk = clk_core (50 MHz); E3 100 MHz osc on the board
//   ck_rst       RESET button (C2, active low)
//   uart_rxd_out FPGA TX -> USB-UART bridge (D10)
//   uart_txd_in  FPGA RX <- USB-UART bridge (A9)
//   led          LD4 (H5, error LED)
//   VGA          Digilent Pmod VGA on JA (J1: R, B) + JB (J2: G, HS, VS)
//   DDR3         MIG DDR3L interface on Bank 34

module bench_chip #(
    parameter int    LANES    = 1,
    parameter bit    HAS_DIV_STACK = 1'b0,
    parameter string IMEM_HEX = "out/kc_imem.hex",
    parameter string DMEM_HEX = "out/kc_dmem.hex",
    parameter string PAL_HEX  = "",
    parameter string TEX_HEX  = ""
) (
    input  logic        clk,            // harness clk = clk_core (stub clk_gen)
    input  logic        ck_rst,         // C2,  active low
    output logic        uart_rxd_out,   // D10, FPGA -> USB-UART bridge
    input  logic        uart_txd_in,    // A9,  FPGA <- USB-UART bridge
    output logic        led,            // H5,  error LED

    // Pmod VGA pins (clk_pix)
    output logic [3:0]  vga_r,
    output logic [3:0]  vga_g,
    output logic [3:0]  vga_b,
    output logic        vga_hs,
    output logic        vga_vs,

    // DDR3 memory pins (MIG)
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

    // ------------------------------------------------------------------
    // Clocks and resets
    // ------------------------------------------------------------------
    logic clk_core;     // 50 MHz
    logic clk_pix;      // 25 MHz
    logic clk_locked;
    logic rst_core;     // synchronous active-high in clk_core domain
    logic rst_pix;      // synchronous active-high in clk_pix domain

    clk_gen u_clk (
        .clk      (clk),
        .rst_n    (ck_rst),
        .clk_core (clk_core),
        .clk_pix  (clk_pix),
        .locked   (clk_locked),
        .rst_core (rst_core),
        .rst_pix  (rst_pix)
    );

    // Drive DDR MMCM from a BUFG as the core MMCM shares the same input pin
    logic clk_ddr_in;
    BUFG u_bufg_ddr (
        .I(clk),
        .O(clk_ddr_in)
    );

    logic clk_sys;      // 166.67 MHz for MIG (passed through in stub)
    logic clk_ref;      // 200 MHz for MIG IDELAYCTRL
    logic ddr_locked;

    ddr_clk u_ddr_clk (
        .clk_in  (clk_ddr_in),
        .rst     (1'b0),
        .clk_sys (clk_sys),
        .clk_ref (clk_ref),
        .locked  (ddr_locked)
    );

    // ------------------------------------------------------------------
    // Display subsystem (P1)
    // ------------------------------------------------------------------
    logic        fb_we, fb_re;
    logic [16:0] fb_addr;
    logic [7:0]  fb_wdata, fb_rdata;
    logic        pal_we;
    logic [7:0]  pal_addr;
    logic [11:0] pal_wdata;
    logic        swap_req, swap_pending, vblank;
    logic [31:0] frame_count;
    logic        fb_mode;

    display #(.PAL_HEX(PAL_HEX)) u_display (
        .clk_core     (clk_core),
        .rst_core     (rst_core),
        .clk_pix      (clk_pix),
        .rst_pix      (rst_pix),
        .fb_we        (fb_we),
        .fb_re        (fb_re),
        .fb_addr      (fb_addr),
        .fb_wdata     (fb_wdata),
        .fb_rdata     (fb_rdata),
        .pal_we       (pal_we),
        .pal_addr     (pal_addr),
        .pal_wdata    (pal_wdata),
        .swap_req     (swap_req),
        .swap_pending (swap_pending),
        .vblank       (vblank),
        .frame_count  (frame_count),
        .fb_mode      (fb_mode),
        .vga_r        (vga_r),
        .vga_g        (vga_g),
        .vga_b        (vga_b),
        .vga_hs       (vga_hs),
        .vga_vs       (vga_vs)
    );

    // ------------------------------------------------------------------
    // SoC: Core, memories, MMIO, UART
    // ------------------------------------------------------------------
    logic [7:0]  leds;
    logic        err_led;

    assign led = err_led;

    logic        ddr_reg_en, ddr_reg_we, ddr_reg_stall;
    logic [5:0]  ddr_reg_off;
    logic [31:0] ddr_reg_wdata, ddr_reg_rdata;

    logic        world_req, world_we, world_ready;
    logic [16:0] world_addr;
    logic [7:0]  world_wdata;
    logic [7:0]  world_rdata;

    // No ray unit here (kc_soc HAS_RAY_UNIT = 0): Stage 2 runs on rtl/top.sv
    // (CHIP_TOP=top HAS_RAY_UNIT=1), so the vt_* port is tied off.
    logic [15:0] vt_result   [LANES];
    logic [16:0] vt_hit_cell [LANES];
    logic [31:0] vt_t_hit    [LANES];

    always_comb for (int l = 0; l < LANES; l++) begin
        vt_result[l]   = '0;
        vt_hit_cell[l] = '0;
        vt_t_hit[l]    = '0;
    end

    kc_soc #(
        .LANES     (LANES),
        .CLK_HZ    (50_000_000),
        .BAUD      (1_000_000),
        .IMEM_HEX  (IMEM_HEX),
        .DMEM_HEX  (DMEM_HEX),
        .TEX_HEX   (TEX_HEX),
        .EXT_WORLD (1'b1),
        .HAS_DIV_STACK (HAS_DIV_STACK)
    ) u_soc (
        .clk           (clk_core),
        .rst           (rst_core),
        .uart_tx       (uart_rxd_out),
        .uart_rx       (uart_txd_in),
        .leds          (leds),
        .err_led       (err_led),
        .fb_we         (fb_we),
        .fb_re         (fb_re),
        .fb_addr       (fb_addr),
        .fb_wdata      (fb_wdata),
        .fb_rdata      (fb_rdata),
        .pal_we        (pal_we),
        .pal_addr      (pal_addr),
        .pal_wdata     (pal_wdata),
        .swap_req      (swap_req),
        .swap_pending  (swap_pending),
        .vblank        (vblank),
        .frame_count   (frame_count),
        .fb_mode       (fb_mode),
        .ddr_reg_en    (ddr_reg_en),
        .ddr_reg_we    (ddr_reg_we),
        .ddr_reg_off   (ddr_reg_off),
        .ddr_reg_wdata (ddr_reg_wdata),
        .ddr_reg_rdata (ddr_reg_rdata),
        .ddr_reg_stall (ddr_reg_stall),
        .world_req     (world_req),
        .world_we      (world_we),
        .world_addr    (world_addr),
        .world_wdata   (world_wdata),
        .world_ready   (world_ready),
        .world_rdata   (world_rdata),
        /* verilator lint_off PINCONNECTEMPTY */
        .vt_cfg_we     (),
        .vt_cfg_field  (),
        .vt_lane_mask  (),
        .vt_cfg_data   (),
        .vt_start      (),
        .vt_max_steps  (),
        /* verilator lint_on PINCONNECTEMPTY */
        .vt_done       (1'b0),
        .vt_result     (vt_result),
        .vt_hit_cell   (vt_hit_cell),
        .vt_t_hit      (vt_t_hit)
    );

    // ------------------------------------------------------------------
    // World window: world_remap -> world_mem (two copies, D17), as rtl/top.sv
    // ------------------------------------------------------------------
    localparam int VT_LANES = 8;   // world_mem / world_remap serve 8 VTRACE lanes

    logic [15:0] slot_valid;
    logic [5:0]  win_x, win_z;
    logic [16:0] core_paddr;
    logic [3:0]  a_rdata;
    logic [3:0]  core_world_rdata;
    logic        eng_we, eng_re;
    logic [16:0] eng_addr;
    logic [3:0]  eng_wdata;

    // VTRACE (Stage 2) is not built yet: busy low, addresses 0.
    logic        vt_busy;
    logic [16:0] vt_addr       [VT_LANES];
    logic [16:0] vt_paddr      [VT_LANES];
    logic [3:0]  vt_rdata_phys [VT_LANES];
    logic [3:0]  vt_rdata      [VT_LANES];

    assign vt_busy = 1'b0;
    always_comb for (int l = 0; l < VT_LANES; l++) vt_addr[l] = '0;

    world_remap #(.LANES(VT_LANES)) u_remap (
        .clk(clk_core), .win_x, .win_z, .slot_valid,
        .core_en(world_req), .core_addr(world_addr), .core_paddr,
        .core_rdata_phys(a_rdata), .core_rdata(core_world_rdata),
        .vt_addr, .vt_paddr, .vt_rdata_phys, .vt_rdata
    );

    world_mem #(.LANES(VT_LANES)) u_world (
        .clk(clk_core),
        .vt_busy, .vt_phase(1'b0), .vt_paddr, .vt_rdata(vt_rdata_phys),
        .core_req(world_req), .core_we(world_we), .core_paddr,
        .core_wdata(world_wdata[3:0]),
        .eng_we, .eng_addr, .eng_wdata,
        .a_rdata
    );

    // The core's world access completes in one cycle (rtl/top.sv).
    assign world_ready = world_req;
    assign world_rdata = {4'b0, core_world_rdata};

`ifdef VERILATOR
    always_ff @(posedge clk_core) begin
        if (!rst_core && (eng_we || eng_re) && (vt_busy || world_req)) begin
            $fatal(1, "engine accessed the world while vt_busy=%0d world_req=%0d",
                   vt_busy, world_req);
        end
    end
`endif

    // ------------------------------------------------------------------
    // Chunk Engine (P2b)
    // ------------------------------------------------------------------
    logic         calib;
    logic         cmd_valid, cmd_ready, cmd_store;
    logic [15:0]  cmd_chunk;
    logic         wr_valid, wr_ready;
    logic [127:0] wr_data;
    logic         rd_valid, rd_ready;
    logic [127:0] rd_data;

    chunk_engine u_chunk (
        .clk            (clk_core),
        .rst            (rst_core),
        .reg_en         (ddr_reg_en),
        .reg_we         (ddr_reg_we),
        .reg_off        (ddr_reg_off),
        .reg_wdata      (ddr_reg_wdata),
        .reg_rdata      (ddr_reg_rdata),
        .reg_stall      (ddr_reg_stall),
        .slot_valid     (slot_valid),
        .win_x          (win_x),
        .win_z          (win_z),
        .vtrace_busy    (vt_busy),
        .core_world_req (world_req),
        .eng_we         (eng_we),
        .eng_re         (eng_re),
        .eng_addr       (eng_addr),
        .eng_wdata      (eng_wdata),
        .eng_rdata      (a_rdata),
        .calib          (calib),
        .cmd_valid      (cmd_valid),
        .cmd_ready      (cmd_ready),
        .cmd_store      (cmd_store),
        .cmd_chunk      (cmd_chunk),
        .wr_valid       (wr_valid),
        .wr_ready       (wr_ready),
        .wr_data        (wr_data),
        .rd_valid       (rd_valid),
        .rd_ready       (rd_ready),
        .rd_data        (rd_data)
    );

    // ------------------------------------------------------------------
    // DDR Bridge (XPM async FIFOs between clk_core and ui_clk)
    // ------------------------------------------------------------------
    logic         ui_clk, ui_rst, init_calib_complete;
    logic [27:0]  app_addr;
    logic [2:0]   app_cmd;
    logic         app_en, app_rdy;
    logic [127:0] app_wdf_data;
    logic         app_wdf_end, app_wdf_wren;
    logic [15:0]  app_wdf_mask;
    logic         app_wdf_rdy;
    logic [127:0] app_rd_data;
    logic         app_rd_data_valid, app_rd_data_end;

    ddr_bridge u_bridge (
        .clk                 (clk_core),
        .rst                 (rst_core),
        .calib               (calib),
        .cmd_valid           (cmd_valid),
        .cmd_ready           (cmd_ready),
        .cmd_store           (cmd_store),
        .cmd_chunk           (cmd_chunk),
        .wr_valid            (wr_valid),
        .wr_ready            (wr_ready),
        .wr_data             (wr_data),
        .rd_valid            (rd_valid),
        .rd_ready            (rd_ready),
        .rd_data             (rd_data),
        .ui_clk              (ui_clk),
        .ui_rst              (ui_rst),
        .init_calib_complete (init_calib_complete),
        .app_addr            (app_addr),
        .app_cmd             (app_cmd),
        .app_en              (app_en),
        .app_rdy             (app_rdy),
        .app_wdf_data        (app_wdf_data),
        .app_wdf_end         (app_wdf_end),
        .app_wdf_wren        (app_wdf_wren),
        .app_wdf_mask        (app_wdf_mask),
        .app_wdf_rdy         (app_wdf_rdy),
        .app_rd_data         (app_rd_data),
        .app_rd_data_valid   (app_rd_data_valid)
    );

    // ------------------------------------------------------------------
    // MIG DDR3 Controller (behavioral replacement in harness)
    // ------------------------------------------------------------------
    /* verilator lint_off PINCONNECTEMPTY */
    mig_ddr3 u_mig (
        .ddr3_addr           (ddr3_addr),
        .ddr3_ba             (ddr3_ba),
        .ddr3_cas_n          (ddr3_cas_n),
        .ddr3_ck_n           (ddr3_ck_n),
        .ddr3_ck_p           (ddr3_ck_p),
        .ddr3_cke            (ddr3_cke),
        .ddr3_ras_n          (ddr3_ras_n),
        .ddr3_reset_n        (ddr3_reset_n),
        .ddr3_we_n           (ddr3_we_n),
        .ddr3_dq             (ddr3_dq),
        .ddr3_dqs_n          (ddr3_dqs_n),
        .ddr3_dqs_p          (ddr3_dqs_p),
        .init_calib_complete (init_calib_complete),
        .ddr3_cs_n           (ddr3_cs_n),
        .ddr3_dm             (ddr3_dm),
        .ddr3_odt            (ddr3_odt),
        .app_addr            (app_addr),
        .app_cmd             (app_cmd),
        .app_en              (app_en),
        .app_wdf_data        (app_wdf_data),
        .app_wdf_end         (app_wdf_end),
        .app_wdf_wren        (app_wdf_wren),
        .app_rd_data         (app_rd_data),
        .app_rd_data_end     (app_rd_data_end),
        .app_rd_data_valid   (app_rd_data_valid),
        .app_rdy             (app_rdy),
        .app_wdf_rdy         (app_wdf_rdy),
        .app_sr_req          (1'b0),
        .app_ref_req         (1'b0),
        .app_zq_req          (1'b0),
        .app_sr_active       (),
        .app_ref_ack         (),
        .app_zq_ack          (),
        .ui_clk              (ui_clk),
        .ui_clk_sync_rst     (ui_rst),
        .app_wdf_mask        (app_wdf_mask),
        .sys_clk_i           (clk_sys),
        .clk_ref_i           (clk_ref),
        .sys_rst             (ddr_locked & ck_rst)
    );
    /* verilator lint_on PINCONNECTEMPTY */

    // ------------------------------------------------------------------
    // Unused signals reduction
    // ------------------------------------------------------------------
    /* verilator lint_off UNUSEDSIGNAL */
    logic unused;
    assign unused = ^{
        clk_locked,
        eng_re,
        app_rd_data_end,
        leds,
        vt_rdata[0],
        world_wdata[7:4]
    };
    /* verilator lint_on UNUSEDSIGNAL */

endmodule
