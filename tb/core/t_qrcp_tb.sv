// t_qrcp: runs sw/t_qrcp.S on a core with HAS_QRCP = 1 (D22) and checks, in
// every lane that can be active: the six QRCP results (a2..a7) against
// fixed.c qrcp() of that lane's inputs, QRCP with rd = rs1 (s7), the
// program's own XOR check (s9 = 0), and QRCP under the mask 0xA5 (inactive
// lanes keep a1 = 0x5A5A0000 | lane). Monitors: no register file ever
// writes x0 (QRCP x0), and every QRCP retires exactly QRCP_CYCLES after the
// instruction before it (fixed latency: ID, EX, 5 x QR, WB; the core fetches
// during the previous instruction's retiring cycle).
module t_qrcp_tb;
    parameter string IMEM_HEX = "";
    parameter string DMEM_HEX = "";
    `include "soc_harness.svh"

    localparam int QRCP_CYCLES = 8;
    localparam int NQRCP       = 9;   // QRCP instructions in t_qrcp.S

    // fixed.c qrcp() of qr_in in t_qrcp.S, [row][lane]
    localparam logic [31:0] EXP [6][8] = '{
        '{32'h40000000, 32'h40000000, 32'h40000000, 32'h3FFFC000, 32'h40000000, 32'h40000000, 32'h40000000, 32'h00000002},
        '{32'h0000FFFF, 32'h00010000, 32'h0000FFFF, 32'h00007FFF, 32'h0001FFFE, 32'h33334000, 32'h24924000, 32'h1FFFE000},
        '{32'h0000E100, 32'h0000017D, 32'h000000FF, 32'h00000004, 32'h00000003, 32'h003FFFC0, 32'h00008000, 32'h00401000},
        '{32'h000AAAA0, 32'h0007FFF8, 32'h00080040, 32'h0007FFC0, 32'h2AAA8000, 32'h40000000, 32'h40000000, 32'h00000153},
        '{32'h00005328, 32'h00068C60, 32'h00000015, 32'h00000111, 32'h00FFFF00, 32'h01010000, 32'h00000002, 32'h0000FF01},
        '{32'h0003FFFC, 32'h0003FFF0, 32'h00040010, 32'h0000000E, 32'h0001999A, 32'h1C71C000, 32'h0F0F0000, 32'h001FFC00}
    };

    int errors = 0;

    task automatic expect32(input string what, input logic [31:0] got, input logic [31:0] exp);
        if (got !== exp) begin
            $display("  FAIL %-28s got %08h expected %08h", what, got, exp);
            errors++;
        end
    endtask

    // x0 is never written, in any lane
    int x0_writes = 0;
    for (genvar l = 0; l < LANES; l++) begin : g_x0_mon
        always @(posedge clk)
            if (!rst && `KC_RF(l).we && `KC_RF(l).wr == 5'd0) x0_writes <= x0_writes + 1;
    end

    // Fixed latency: cycles from the previous retirement to each QRCP's
    int          qrcp_seen = 0, qrcp_bad_lat = 0;
    int unsigned last_retire = 0;
    always @(posedge clk) begin
        if (!rst && dut.u_core.retire_valid) begin
            if (dut.u_core.retire_inst[6:0] == 7'h0B && dut.u_core.retire_inst[14:12] == 3'b010) begin
                qrcp_seen <= qrcp_seen + 1;
                if (cycle - last_retire != QRCP_CYCLES) begin
                    $display("  FAIL QRCP at %08h retired %0d cycles after the previous instruction",
                             dut.u_core.retire_pc, cycle - last_retire);
                    qrcp_bad_lat <= qrcp_bad_lat + 1;
                end
            end
            last_retire <= cycle;
        end
    end

    initial begin
        logic [7:0] full, m;
        full = 8'((1 << ALANES) - 1);
        m    = 8'hA5 & full;

        do_reset();
        run_until_done(5000);
        repeat (10) @(posedge clk);

        expect32("error LED", {31'b0, err_led}, 32'd0);
        for (int l = 0; l < ALANES; l++) begin
            for (int r = 0; r < 6; r++)
                expect32($sformatf("lane %0d row %0d (x%0d)", l, r, 12 + r), lane_x[l][12 + r], EXP[r][l]);
            expect32($sformatf("lane %0d rd = rs1 (s7)", l), lane_x[l][23], EXP[5][l]);
            expect32($sformatf("lane %0d self-check s9", l), lane_x[l][25], 32'd0);
            expect32($sformatf("lane %0d under 0xA5 (a1)", l), lane_x[l][11],
                     m[l] ? EXP[1][l] : (32'h5A5A_0000 | 32'(l)));
        end
        expect32("x0 writes", 32'(x0_writes), 32'd0);
        expect32("QRCP retired", 32'(qrcp_seen), 32'(NQRCP));
        expect32("QRCP latency errors", 32'(qrcp_bad_lat), 32'd0);
        expect32("mask at the end", 32'(dut.u_core.mask), 32'd1);

        if (errors != 0) $fatal(1, "FAIL: t_qrcp, %0d mismatches", errors);
        $display("PASS: t_qrcp (LANES = %0d, %0d can be active, %0d QRCP of %0d cycles)",
                 LANES, ALANES, NQRCP, QRCP_CYCLES);
        $finish;
    end
endmodule
