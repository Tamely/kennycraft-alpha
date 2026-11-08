// KennyCraft core, Stages 1 and 2: multi-cycle RV32I + Zmmul + custom-0 lane
// control, plus custom-2 divergence stack when HAS_DIV_STACK = 1, custom-1
// ray unit (VSET, VTRACE, VGET) when HAS_RAY_UNIT = 1 and QRCP when
// HAS_QRCP = 1, on LANES SIMD lanes with one shared control path (Tai page,
// GPU Core; Tai_2 sections 1 and 2).
//
// Grown from Tai's multicycle core (datapath.sv in RISC-V-Core-progression).
// No pipeline registers, no forwarding, no hazard logic. One FSM walks every
// instruction through the states of the Tai page, GPU Core, SIMD state
// machine:
//
//   IF    send PC to the instruction BRAM (reset only, see below)  1 cycle
//   ID    instruction arrives, decode once, read every lane's registers
//   EX    every lane's ALU, comparator, address generator, multiply;
//         lane control updates the mask                             1
//   MUL   second EXECUTE cycle for MUL family (registered product) 1
//   VT    VTRACE: wait for the ray unit's done pulse              walk + 1
//   QR    QRCP: step every lane's QRCP unit, latch the result      5
//   MEM   lane-serial LSU, one bus access per active lane, ascending
//         order; stores retire on the last lane's access           N + slave wait
//   WB    every lane writes rd together (the last load lane's data
//         arrives here), update PC                                  1
//
// Fetch overlap. next_pc is registered in EX, so the cycle that retires an
// instruction (WB, or a store's last MEM cycle) already drives next_pc to the
// instruction BRAM and the FSM goes straight to ID: IF runs only after reset.
// That is one cycle less for every instruction (ALU 3, MUL 4, an 8-lane load
// 11). Register writes land at the end of WB and the next ID reads the
// register file after them, so nothing else changes.
//
// Shared state here: PC, fetch, decode, FSM, the LANES-bit active mask (reset
// 1) and the scalar-lane mux. rtl/core/lane.sv is one lane's datapath and
// rtl/core/lsu.sv the lane-serial LSU.
//
// Lane control (custom-0). LANEID returns the lane index, LMASK clears the
// bits of lanes whose rs1 = 0, MASKSET keeps rs1[LANES-1:0] of the scalar
// lane, MASKALL sets every bit, BALLOT ({0, M & rs1 != 0}) and MASKGET ({0,
// M}) write rd in every lane, zero-extended. Lane n writes rd when
// wb_en & (M[n] | uniform_write) & (rd != 0), uniform_write being BALLOT and
// MASKGET only. Loads and stores access memory for active lanes only, so an
// inactive lane's misaligned address raises no error.
//
// Scalar lane: the lowest active lane, lane 0 when M = 0. Branch decisions and
// JALR targets come from it. Uniform checks (EX): a branch whose outcome or a
// JALR whose target differs between active lanes halts with ERR.DIVERGE and
// err_pc = its PC, before anything is written; with M = 0 nothing is checked.
// custom-2 (DIF, DELSE, DENDIF) runs on rtl/core/divstack.sv when
// HAS_DIV_STACK = 1 and decodes as illegal otherwise.
//
// custom-1 (Tai_2 sections 1 and 2) drives the ray unit (rtl/vtrace/vtrace.sv,
// instantiated in rtl/top.sv) through the vt_* ports when HAS_RAY_UNIT = 1 and
// decodes as illegal otherwise. vt_lane_mask is always M. VSET pulses
// vt_cfg_we in EX with vt_cfg_field = funct7 and vt_cfg_data = each lane's
// rs1. VTRACE pulses vt_start in EX with vt_max_steps = the scalar lane's
// rs1[7:0], waits in VT for vt_done, latches each lane's {16'b0, result} and
// writes it to rd in active lanes; a cap of 0 or M = 0 writes nothing. VGET
// latches hit_cell (field 0, zero-extended) or t_hit (field 1) in EX and
// writes rd in active lanes.
//
// QRCP (D22, HAS_QRCP = 1; illegal otherwise) runs rtl/core/qrcp.sv in every
// lane: the unit latches its rs1 in EX, QR steps it through qr_ph 0..3 and
// latches its result into res at qr_ph 4, and WB writes rd in active lanes
// (rd = x0 never). Fixed latency: 8 cycles per QRCP (ID, EX, 5 x QR, WB; the fetch overlaps WB).
//
// Errors (Tai_2 sections 1 and 2). An illegal instruction or a fetch outside
// instruction memory (caught in ID), a taken branch, JAL, JALR, or a DIF or
// DELSE that jumps, whose target is not 4-aligned (EX, D16: the target is
// never fetched, the link is not written), or a misaligned load/store in any
// active lane (EX, before any bus access) pulses err_set[ERR_ILLEGAL]. A
// divergent branch or JALR pulses err_set[ERR_DIVERGE], a DIF on a full or a
// DELSE/DENDIF on an empty divergence stack err_set[ERR_DIVSTACK]. One bit
// per halt (DIVERGE, then DIVSTACK, then ILLEGAL), err_pc = the
// instruction's own PC, and the FSM parks in HALT until reset: nothing is
// fetched or written, the mask and stack stay as they were. Every PC the
// core reaches is 4-aligned, so fetch never needs an alignment check.
//
// Buses. imem_rdata is the instruction BRAM output for imem_addr one cycle
// earlier. On the data bus, bus_req is held until bus_ready; the access
// happens once in the bus_req && bus_ready cycle and load data is valid in
// bus_rdata the cycle after.
module core
    import kc_isa_pkg::*;
#(
    parameter int          LANES         = 1,
    parameter logic [31:0] RESET_VEC     = IMEM_BASE,
    parameter bit          HAS_DIV_STACK = 1'b0,
    parameter bit          HAS_RAY_UNIT  = 1'b0,
    parameter bit          HAS_QRCP      = 1'b0
) (
    input  logic                 clk,
    input  logic                 rst,

    output logic [31:0]          imem_addr,
    input  logic [31:0]          imem_rdata,

    output logic                 bus_req,
    output logic                 bus_we,
    output logic [31:0]          bus_addr,
    output logic [3:0]           bus_be,
    output logic [31:0]          bus_wdata,
    input  logic                 bus_ready,
    input  logic [31:0]          bus_rdata,

    output logic [ERR_WIDTH-1:0] err_set,
    output logic [31:0]          err_pc,
    output logic                 halted,
    output logic [31:0]          pc,

    // Ray unit (HAS_RAY_UNIT = 1); outputs 0 and inputs ignored otherwise
    output logic                 vt_cfg_we,
    output logic [2:0]           vt_cfg_field,
    output logic [LANES-1:0]     vt_lane_mask,
    output logic [31:0]          vt_cfg_data [LANES],
    output logic                 vt_start,
    output logic [7:0]           vt_max_steps,
    input  logic                 vt_done,
    input  logic [15:0]          vt_result   [LANES],
    input  logic [16:0]          vt_hit_cell [LANES],
    input  logic [31:0]          vt_t_hit    [LANES]
);

    if (LANES < 1 || LANES > 8) begin : g_lanes_check
        $error("core: LANES must be 1 to 8");
    end

    localparam int LW = (LANES > 1) ? $clog2(LANES) : 1;

    typedef enum logic [3:0] {
        S_IF, S_ID, S_EX, S_MUL, S_MEM, S_WB, S_HALT, S_VT, S_QR
    } state_t;

    state_t state;

    // ------------------------------------------------------------------
    // ID: decode the BRAM output once
    // ------------------------------------------------------------------
    logic [3:0]  d_alu_ctrl;
    logic        d_alu_a_pc, d_r_we, d_res_imm, d_is_load, d_is_store, d_is_mul;
    logic        d_is_branch, d_is_jal, d_is_jalr, d_is_lane, d_is_div, d_is_ray, d_is_qrcp;
    logic        d_uniform_wr, d_illegal;
    logic [1:0]  d_alu_b_sel;
    logic [2:0]  d_imm_sig;
    logic [31:0] d_imm;
    logic        id_fault;

    inst_decoder #(.HAS_DIV_STACK(HAS_DIV_STACK), .HAS_RAY_UNIT(HAS_RAY_UNIT),
                   .HAS_QRCP(HAS_QRCP)) u_dec (
        .inst(imem_rdata), .alu_ctrl(d_alu_ctrl), .alu_a_pc(d_alu_a_pc),
        .alu_b_sel(d_alu_b_sel), .imm_sig(d_imm_sig), .r_we(d_r_we),
        .res_imm(d_res_imm), .is_load(d_is_load), .is_store(d_is_store),
        .is_mul(d_is_mul), .is_branch(d_is_branch), .is_jal(d_is_jal),
        .is_jalr(d_is_jalr), .is_lane(d_is_lane), .is_div(d_is_div), .is_ray(d_is_ray),
        .is_qrcp(d_is_qrcp), .uniform_wr(d_uniform_wr), .illegal(d_illegal)
    );

    immgen u_imm (.inst(imem_rdata[31:7]), .imm_sig(d_imm_sig), .imm(d_imm));

    assign id_fault = d_illegal || is_fetch_outside_imem(pc);

    // ID -> EX registers (shared decode; the operands live in the lanes)
    logic [3:0]  ex_alu_ctrl;
    logic        ex_alu_a_pc, ex_r_we, ex_res_imm, ex_is_load, ex_is_store, ex_is_mul;
    logic        ex_is_branch, ex_is_jal, ex_is_jalr, ex_is_lane, ex_is_div, ex_is_ray, ex_uniform_wr;
    logic        ex_is_qrcp;
    logic [1:0]  ex_alu_b_sel;
    logic [2:0]  ex_funct3;
    logic [2:0]  ex_field;    // funct7[2:0]: VSET / VGET field
    logic [4:0]  ex_rd;
    logic [31:0] ex_imm;
    logic [31:0] ir;          // instruction in flight, for the retire wires

    always_ff @(posedge clk) begin
        if (state == S_ID) begin
            ir            <= imem_rdata;
            ex_alu_ctrl   <= d_alu_ctrl;
            ex_alu_a_pc   <= d_alu_a_pc;
            ex_alu_b_sel  <= d_alu_b_sel;
            ex_r_we       <= d_r_we && (instr_rd(imem_rdata) != 5'd0);
            ex_res_imm    <= d_res_imm;
            ex_is_load    <= d_is_load;
            ex_is_store   <= d_is_store;
            ex_is_mul     <= d_is_mul;
            ex_is_branch  <= d_is_branch;
            ex_is_jal     <= d_is_jal;
            ex_is_jalr    <= d_is_jalr;
            ex_is_lane    <= d_is_lane;
            ex_is_div     <= d_is_div;
            ex_is_ray     <= d_is_ray;
            ex_is_qrcp    <= d_is_qrcp;
            ex_uniform_wr <= d_uniform_wr;
            ex_funct3     <= instr_funct3(imem_rdata);
            ex_field      <= imem_rdata[27:25];   // funct7[2:0]
            ex_rd         <= instr_rd(imem_rdata);
            ex_imm        <= d_imm;
        end
    end

    // ------------------------------------------------------------------
    // Lanes
    // ------------------------------------------------------------------
    logic [LANES-1:0] mask, mask_d;
    logic             mask_we;
    logic [31:0]      lane_y;                 // BALLOT / MASKGET result
    logic [31:0]      l_rs1 [LANES];
    logic [31:0]      l_rs2 [LANES];
    logic [31:0]      l_alu [LANES];
    logic [31:0]      l_res [LANES];
    logic [31:0]      l_wdata [LANES];
    logic [1:0]       l_addr_lo [LANES];
    logic [LANES-1:0] l_cmp, l_we, nonzero, ld_cap, mis;
    logic [31:0]      ld_value;
    logic             wb_en;
    logic             ray_en;                 // latch ray_y into res (VGET, VTRACE)
    logic [31:0]      ray_y [LANES];
    logic             vt_nowr;                // VTRACE with a cap of 0: no write
    logic [2:0]       qr_ph;                  // QRCP step in QR
    logic             qr_wr;                  // QR, last step: latch the result

    assign qr_wr = HAS_QRCP && (state == S_QR) && (qr_ph == 3'd4);

    assign wb_en = (state == S_WB) && ex_r_we && !vt_nowr;

    for (genvar i = 0; i < LANES; i++) begin : g_lane
        lane #(.ID(i), .HAS_QRCP(HAS_QRCP)) u_lane (
            .clk,
            .id_en(state == S_ID),
            .rs1_addr(instr_rs1(imem_rdata)), .rs2_addr(instr_rs2(imem_rdata)),
            .ex_en(state == S_EX), .mul_en(state == S_MUL),
            .alu_ctrl(ex_alu_ctrl), .alu_a_pc(ex_alu_a_pc), .alu_b_sel(ex_alu_b_sel),
            .funct3(ex_funct3), .pc, .imm(ex_imm), .res_imm(ex_res_imm),
            .is_lane(ex_is_lane), .lane_y, .ray_en, .ray_y(ray_y[i]),
            .qr_load((state == S_EX) && ex_is_qrcp), .qr_en(state == S_QR), .qr_ph, .qr_wr,
            .rs1(l_rs1[i]), .rs2(l_rs2[i]), .alu_y(l_alu[i]), .cmp(l_cmp[i]), .res(l_res[i]),
            .ld_cap(ld_cap[i]), .ld_value,
            .we(l_we[i]), .rd(ex_rd), .is_load(ex_is_load), .wdata(l_wdata[i])
        );

        assign nonzero[i]   = (l_rs1[i] != 32'd0);
        assign l_we[i]      = wb_en && (mask[i] || ex_uniform_wr);
        assign l_addr_lo[i] = l_alu[i][1:0];
    end

    // ------------------------------------------------------------------
    // EX: scalar lane, branch, uniform checks, divergence stack, mask
    // ------------------------------------------------------------------
    logic [LW-1:0]    sl;           // scalar lane: lowest active, 0 when M = 0
    logic [31:0]      target;
    logic             take;
    logic [LANES-1:0] l_taken;      // per-lane branch outcome
    logic [31:0]      l_jt [LANES]; // per-lane JALR target
    logic [LANES-1:0] jt_diff;      // lane's JALR target differs from the scalar lane's
    logic             ex_diverge, ex_divstack, ex_halt;
    logic             ds_err, ds_jump;
    logic [LANES-1:0] ds_mask;

    always_comb begin
        sl = '0;
        for (int i = LANES - 1; i >= 0; i--)
            if (mask[i]) sl = LW'(i);
    end

    for (genvar i = 0; i < LANES; i++) begin : g_uniform
        // funct3[0] inverts the comparator (BNE, BGE, BGEU).
        assign l_taken[i] = l_cmp[i] ^ ex_funct3[0];
        assign l_jt[i]    = (l_rs1[i] + ex_imm) & ~32'd1;
        assign jt_diff[i] = (l_jt[i] != l_jt[sl]);
    end

    // Uniform checks over the active lanes only; with M = 0 nothing is
    // compared and lane 0 (the scalar lane) decides.
    assign ex_diverge = (ex_is_branch && ((l_taken & mask) != '0) && ((l_taken & mask) != mask)) ||
                        (ex_is_jalr && ((jt_diff & mask) != '0));

    // custom-2 (Tai_2 section 2, Divergence stack): DIF and DELSE jump to the
    // B-type label when divstack says so; DENDIF never jumps.
    if (HAS_DIV_STACK) begin : g_divstack
        divstack #(.LANES(LANES)) u_divstack (
            .clk, .rst,
            .op(ex_funct3), .mask, .cond(nonzero),
            .step((state == S_EX) && ex_is_div && !ex_halt),
            .err(ds_err), .mask_d(ds_mask), .jump(ds_jump)
        );
    end else begin : g_no_divstack
        assign ds_err  = 1'b0;
        assign ds_mask = '0;
        assign ds_jump = 1'b0;
    end

    assign ex_divstack = ex_is_div && ds_err;

    assign take   = ex_is_jal || ex_is_jalr || (ex_is_branch && l_taken[sl]) ||
                    (ex_is_div && ds_jump);
    assign target = ex_is_jalr ? l_jt[sl] : (pc + ex_imm);

    always_comb begin
        lane_y  = 32'd0;
        mask_d  = mask;
        mask_we = 1'b0;
        if (ex_is_div) begin
            mask_d  = ds_mask;
            mask_we = 1'b1;
        end else if (ex_is_lane) begin
            unique case (ex_funct3)
                F3_LMASK: begin
                    mask_d  = mask & nonzero;
                    mask_we = 1'b1;
                end
                F3_BALLOT:  lane_y = {{(32-LANES){1'b0}}, mask & nonzero};
                F3_MASKGET: lane_y = {{(32-LANES){1'b0}}, mask};
                F3_MASKSET: begin
                    mask_d  = l_rs1[sl][LANES-1:0];
                    mask_we = 1'b1;
                end
                F3_MASKALL: begin
                    mask_d  = '1;
                    mask_we = 1'b1;
                end
                default: ;   // LANEID: each lane returns its own index
            endcase
        end
    end

    always_ff @(posedge clk) begin
        if (rst)                                     mask <= LANES'(1);
        else if (state == S_EX && mask_we && !ex_halt) mask <= mask_d;
    end

    logic [31:0] next_pc;

    always_ff @(posedge clk) begin
        if (state == S_EX) next_pc <= take ? target : pc + 32'd4;
    end

    // QRCP step counter: 0 from EX, counts through QR (constant 0 without QRCP)
    if (HAS_QRCP) begin : g_qr_ph
        always_ff @(posedge clk) begin
            if (state == S_EX)      qr_ph <= 3'd0;
            else if (state == S_QR) qr_ph <= qr_ph + 3'd1;
        end
    end else begin : g_no_qr_ph
        assign qr_ph = 3'd0;
    end

    // ------------------------------------------------------------------
    // Ray unit (custom-1)
    // ------------------------------------------------------------------
    logic ex_vtrace;

    if (HAS_RAY_UNIT) begin : g_ray
        logic [7:0] cap;

        assign ex_vtrace = ex_is_ray && (ex_funct3 == F3_VTRACE);
        assign cap       = l_rs1[sl][7:0];

        assign vt_lane_mask = mask;
        assign vt_cfg_we    = (state == S_EX) && ex_is_ray && (ex_funct3 == F3_VSET);
        assign vt_cfg_field = ex_field;
        assign vt_cfg_data  = l_rs1;
        assign vt_start     = (state == S_EX) && ex_vtrace;
        assign vt_max_steps = cap;
        assign ray_en       = ex_is_ray && (((state == S_EX) && (ex_funct3 == F3_VGET)) ||
                                            ((state == S_VT) && vt_done));

        for (genvar i = 0; i < LANES; i++) begin : g_y
            assign ray_y[i] = (ex_funct3 != F3_VGET) ? {16'd0, vt_result[i]} :
                              ex_field[0]            ? vt_t_hit[i] : {15'd0, vt_hit_cell[i]};
        end

        // A cap of 0 finishes with no writes (M = 0 writes nothing anyway).
        always_ff @(posedge clk) begin
            if (state == S_EX) vt_nowr <= ex_vtrace && (cap == 8'd0);
        end
    end else begin : g_no_ray
        assign ex_vtrace    = 1'b0;
        assign vt_lane_mask = '0;
        assign vt_cfg_we    = 1'b0;
        assign vt_cfg_field = '0;
        assign vt_start     = 1'b0;
        assign vt_max_steps = '0;
        assign ray_en       = 1'b0;
        assign vt_nowr      = 1'b0;
        for (genvar i = 0; i < LANES; i++) begin : g_y
            assign vt_cfg_data[i] = '0;
            assign ray_y[i]       = '0;
        end

        /* verilator lint_off UNUSEDSIGNAL */
        logic unused_ray;
        /* verilator lint_on UNUSEDSIGNAL */
        always_comb begin
            unused_ray = vt_done ^ ex_is_ray ^ (^ex_field);
            for (int i = 0; i < LANES; i++)
                unused_ray ^= (^vt_result[i]) ^ (^vt_hit_cell[i]) ^ (^vt_t_hit[i]);
        end
    end

    // ------------------------------------------------------------------
    // MEM: lane-serial LSU
    // ------------------------------------------------------------------
    logic ex_mem, ex_illegal, lsu_done;

    assign ex_mem     = (ex_is_load || ex_is_store) && (mask != '0);
    // A taken jump to a target with bit 1 set halts at the jump (D16): a
    // branch, JAL, JALR, or a DIF/DELSE that jumps. Bit 0 is 0 for B/J
    // immediates and cleared for JALR. Lanes that disagree on a branch or
    // JALR halt with ERR.DIVERGE only, a DIVSTACK halt wins over the rest.
    assign ex_illegal = !ex_diverge && !ex_divstack &&
                        ((ex_mem && ((mis & mask) != '0)) || (take && target[1]));
    assign ex_halt    = ex_diverge || ex_divstack || ex_illegal;

    lsu #(.LANES(LANES)) u_lsu (
        .clk, .rst,
        .start((state == S_EX) && ex_mem && !ex_halt), .active(mask),
        .go(state == S_MEM), .is_store(ex_is_store), .funct3(ex_funct3),
        .addr(l_res), .rs2(l_rs2), .done(lsu_done),
        .ex_addr_lo(l_addr_lo), .misaligned(mis),
        .bus_req, .bus_we, .bus_addr, .bus_be, .bus_wdata, .bus_ready, .bus_rdata,
        .ld_cap, .ld_value
    );

    // ------------------------------------------------------------------
    // FSM, PC and errors
    // ------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            state <= S_IF;
            pc    <= RESET_VEC;
        end else begin
            unique case (state)
                S_IF:  state <= S_ID;
                S_ID:  state <= id_fault ? S_HALT : S_EX;
                S_EX: begin
                    if (ex_halt)                         state <= S_HALT;
                    else if (ex_is_mul)                  state <= S_MUL;
                    else if (ex_mem)                     state <= S_MEM;
                    else if (ex_vtrace)                  state <= S_VT;
                    else if (HAS_QRCP && ex_is_qrcp)     state <= S_QR;
                    else                                 state <= S_WB;
                end
                S_MUL: state <= S_WB;
                S_VT:  if (vt_done) state <= S_WB;
                S_QR:  if (qr_wr) state <= S_WB;
                S_MEM: begin
                    if (lsu_done) begin
                        if (ex_is_store) begin
                            state <= S_ID;      // next_pc was fetched this cycle
                            pc    <= next_pc;
                        end else begin
                            state <= S_WB;
                        end
                    end
                end
                S_WB: begin
                    state <= S_ID;              // next_pc was fetched this cycle
                    pc    <= next_pc;
                end
                default: state <= S_HALT;
            endcase
        end
    end

    // The retiring cycle fetches next_pc (fetch overlap, above); every other
    // state keeps the current instruction at the BRAM output.
    assign imem_addr = ((state == S_WB) || ((state == S_MEM) && lsu_done && ex_is_store))
                     ? next_pc : pc;
    assign halted    = (state == S_HALT);
    assign err_pc    = pc;

    always_comb begin
        err_set = '0;
        err_set[ERR_ILLEGAL] = ((state == S_ID) && id_fault) || ((state == S_EX) && ex_illegal);
        err_set[ERR_DIVERGE]  = (state == S_EX) && ex_diverge;
        err_set[ERR_DIVSTACK] = (state == S_EX) && ex_divstack;
    end

    // ------------------------------------------------------------------
    // Retire observation wires for Verilator harnesses and traces (not used
    // by any logic). One retire per instruction: stores in the cycle the bus
    // accepts the last lane, everything else in WB. A halting instruction
    // never retires. retire_mask is the active mask at retirement. The
    // retire_rd_* wires are lane 0's write; retire_lane_we/wdata carry every
    // lane's (rd is retire_rd_addr in all of them).
    // ------------------------------------------------------------------
    /* verilator lint_off UNUSEDSIGNAL */
    logic             retire_valid;
    logic [31:0]      retire_pc;
    logic [31:0]      retire_inst;
    logic             retire_rd_we;
    logic [4:0]       retire_rd_addr;
    logic [31:0]      retire_rd_wdata;
    logic [LANES-1:0] retire_mask;
    logic [LANES-1:0] retire_lane_we;
    logic [31:0]      retire_lane_wdata [LANES];
    /* verilator lint_on UNUSEDSIGNAL */

    assign retire_valid      = (state == S_WB) || ((state == S_MEM) && lsu_done && ex_is_store);
    assign retire_pc         = pc;
    assign retire_inst       = ir;
    assign retire_rd_we      = l_we[0];
    assign retire_rd_addr    = ex_rd;
    assign retire_rd_wdata   = l_wdata[0];
    assign retire_mask       = mask;
    assign retire_lane_we    = l_we;
    assign retire_lane_wdata = l_wdata;

endmodule
