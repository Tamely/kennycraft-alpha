// t_cpi: reports the CPI of every instruction class, measured by the program
// with the CYCLES register (sw/t_cpi.S), and checks it against the state
// machine in rtl/core/core.sv so a timing change cannot go unnoticed.
module t_cpi_tb;
    parameter string IMEM_HEX = "";
    parameter string DMEM_HEX = "";
    `include "soc_harness.svh"

    localparam int          N       = 32;
    localparam int          NSLOTS  = 25;
    localparam logic [31:0] RESULTS = 32'h1000_0100;

    function automatic string slot_name(input int k);
        case (k)
            1: return "ALU (add)";            2: return "ALU imm (addi)";
            3: return "LUI";                  4: return "AUIPC";
            5: return "MUL";                  6: return "MULHU";
            7: return "branch not taken";     8: return "branch taken";
            9: return "JALR";                 10: return "JAL";
            11: return "load LW dmem";        12: return "load LBU dmem";
            13: return "store SW dmem";       14: return "store SB dmem";
            15: return "load LW MMIO";        16: return "load LBU world";
            17: return "load LW world";       18: return "store SB world";
            19: return "store SW world";      20: return "load LW textures";
            21: return "store SB framebuffer"; 22: return "FENCE";
            23: return "lane ctl (MASKGET)"; 24: return "lane ctl (MASKALL)";
            default: return "?";
        endcase
    endfunction

    // The core fetches the next instruction in the retiring cycle (fetch
    // overlap, core.sv), so there is no IF cycle per instruction.
    function automatic int expected_cpi(input int k);
        case (k)
            5, 6, 11, 12, 15, 16: return 4;
            17, 20:               return 7;   // four byte reads
            19:                   return 6;   // four byte writes
            default:              return 3;
        endcase
    endfunction

    initial begin
        int base, delta, auipc_cycles, errors;
        real cpi;
        errors = 0;
        do_reset();
        run_until_done(100000);
        if (err_led) $fatal(1, "FAIL: core halted, ERR_PC=%08h", dut.u_mmio.err_pc);

        base = int'(dmem_rd(RESULTS));
        auipc_cycles = int'(dmem_rd(RESULTS + 4 * 4)) - base;
        $display("  CYCLES overhead (empty slot) = %0d, N = %0d", base, N);
        $display("  %-22s %s", "class", "CPI");
        for (int k = 1; k < NSLOTS; k++) begin
            delta = int'(dmem_rd(RESULTS + 4 * k)) - base;
            if (k == 9) delta -= auipc_cycles;   // AUIPC + JALR pairs
            cpi = real'(delta) / N;
            $display("  %-22s %0.2f", slot_name(k), cpi);
            if (delta != expected_cpi(k) * N) begin
                $display("  FAIL %s: expected CPI %0d", slot_name(k), expected_cpi(k));
                errors++;
            end
        end
        if (errors != 0) $fatal(1, "FAIL: t_cpi, %0d classes off", errors);
        $display("PASS: t_cpi");
        $finish;
    end
endmodule
