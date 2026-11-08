// t_mmio: runs sw/t_mmio.S against the harness's display and chunk-engine
// models and checks every MMIO register (LEDS, FB_MODE, FB_SWAP, FRAME_COUNT,
// VSYNC, CYCLES, UART_STATUS/RX/TX, DDR register forwarding with a CHUNK_CMD
// stall), the framebuffer, palette, world and texture byte regions and the
// GPU memory region (word BRAM, byte enables, no aliasing past 68 KB).
module t_mmio_tb;
    parameter string IMEM_HEX = "";
    parameter string DMEM_HEX = "";
    `include "soc_harness.svh"

    localparam logic [31:0] RES   = 32'h1000_0100;
    localparam int          STALL = 37;

    int errors = 0;

    task automatic expect32(input string what, input logic [31:0] got, input logic [31:0] exp);
        if (got !== exp) begin
            $display("  FAIL %-30s got %08h expected %08h", what, got, exp);
            errors++;
        end else begin
            $display("  ok   %-30s %08h", what, got);
        end
    endtask

    // Display model: a swap request is pending for 300 cycles, then vblank
    // flips it and FRAME_COUNT advances.
    initial begin
        forever begin
            @(posedge clk);
            if (swap_req) begin
                swap_pending = 1'b1;
                repeat (300) @(posedge clk);
                vblank       = 1'b1;
                swap_pending = 1'b0;
                frame_count  = 32'h1234_5678;
            end
        end
    end

    initial begin
        ddr_regs[0]   = 32'h0000_0001;   // DDR_STATUS: calibrated
        ddr_cmd_stall = STALL;
        do_reset();
        run_until_done(50000);
        if (err_led) $fatal(1, "FAIL: core halted, ERR_PC=%08h", dut.u_mmio.err_pc);

        expect32("LEDS pins", {24'b0, leds}, 32'hA5);
        expect32("LEDS read (write only)", dmem_rd(RES + 0), 32'h0);
        expect32("FB_MODE read", dmem_rd(RES + 4), 32'h1);
        expect32("fb_mode pin", {31'b0, fb_mode}, 32'h1);
        expect32("FB_SWAP after write", dmem_rd(RES + 8), 32'h1);
        expect32("swap_req pulses", swap_req_count, 32'd1);
        if (dmem_rd(RES + 12) < 32'd10) begin
            $display("  FAIL FB_SWAP poll count %0d, expected >= 10", dmem_rd(RES + 12));
            errors++;
        end
        expect32("FRAME_COUNT", dmem_rd(RES + 16), 32'h1234_5678);
        expect32("VSYNC", dmem_rd(RES + 20), 32'h1);
        expect32("CYCLES delta (one LW)", dmem_rd(RES + 24), 32'd4);   // ID EX MEM WB (fetch overlap)
        expect32("UART_STATUS idle", dmem_rd(RES + 28), 32'h0);
        expect32("UART_RX empty", dmem_rd(RES + 32), 32'h0);
        expect32("UART_STATUS TX busy", dmem_rd(RES + 36), 32'h2);
        expect32("SLOT_VALID via ddr port", dmem_rd(RES + 40), 32'hBEEF);
        expect32("DDR_STATUS via ddr port", dmem_rd(RES + 44), 32'h1);
        expect32("CHUNK_CMD stall cycles", ddr_stalled_cycles, STALL + 1);
        expect32("CHUNK_CMD time (LW + SW + stall)", dmem_rd(RES + 48), 32'(4 + 3 + STALL + 1));   // LW 4, SW 3 (fetch overlap)
        expect32("CHUNK_CMD value", ddr_regs[1], 32'h0000_0302);
        expect32("WIN_ORIGIN value", ddr_regs[4], 32'h0000_1020);
        expect32("fb LW after SW+SB", dmem_rd(RES + 52), 32'h7733_2211);
        expect32("fb LHU", dmem_rd(RES + 56), 32'h7733);
        expect32("fb model bytes 16..19", {fb_mem[19], fb_mem[18], fb_mem[17], fb_mem[16]}, 32'h7733_2211);
        expect32("fb last pixel", dmem_rd(RES + 60), 32'h9C);
        expect32("palette read (write only)", dmem_rd(RES + 64), 32'h0);
        expect32("palette entry 5", {20'b0, pal_mem[5]}, 32'hABC);
        expect32("world LW (4-bit stub)", dmem_rd(RES + 68), 32'h0F0E_0F0C);
        expect32("texture LBU (4-bit stub)", dmem_rd(RES + 72), 32'h0A);
        expect32("world LH", dmem_rd(RES + 76), 32'h0F0C);
        expect32("gpu LW at GPU_BASE", dmem_rd(RES + 80), 32'h1234_5678);
        expect32("gpu LW last word", dmem_rd(RES + 84), 32'hCAFE_F00D);
        expect32("gpu backdoor last word", dut.u_gpu_hi.mem[1023], 32'hCAFE_F00D);
        expect32("gpu LW after SB+SH", dmem_rd(RES + 88), 32'h8001_8044);
        expect32("gpu LBU", dmem_rd(RES + 92), 32'h0000_0080);
        expect32("gpu LB sign extends", dmem_rd(RES + 96), 32'hFFFF_FF80);
        expect32("gpu LHU", dmem_rd(RES + 100), 32'h0000_8001);
        expect32("gpu LH sign extends", dmem_rd(RES + 104), 32'hFFFF_8001);
        expect32("gpu +0x11000 load (unmapped)", dmem_rd(RES + 108), 32'h0);
        expect32("gpu +0x20000 load (unmapped)", dmem_rd(RES + 112), 32'h0);
        expect32("gpu offset 0 not aliased", dmem_rd(RES + 116), 32'h1234_5678);
        expect32("ERR after gpu accesses", {29'b0, dut.u_mmio.err}, 32'd0);

        if (errors != 0) $fatal(1, "FAIL: t_mmio, %0d mismatches", errors);
        $display("PASS: t_mmio");
        $finish;
    end
endmodule
