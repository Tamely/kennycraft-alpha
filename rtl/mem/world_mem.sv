// World window memory (D17, Tai page, Memory & Display): two copies of
// 131,072 x 4 bits, physical addresses {y[4:0], z[5:0], x[5:0]} after
// world_remap. Each copy is one true dual-port BRAM array on the core clock.
//
//   port A  VTRACE while vt_busy, else the core (core_req), else the chunk
//           engine. Writes (core stores, engine LOADs) go to both copies at the
//           same address; core and engine reads come from copy 0 (a_rdata,
//           one cycle after the address).
//   port B  VTRACE only.
//
// VTRACE (WORLD_LATENCY = 2). Copy n serves lanes 4n..4n+3. vt_paddr is held
// for two cycles; vt_phase (from vtrace: the index of its WAIT cycle) is 0
// in the first and 1 in the second, and 0 in the CHECK cycle. In phase 0
// ports A and B of copy n read lanes 4n and 4n+1, in phase 1 lanes 4n+2 and
// 4n+3. The phase-0 data is captured on the phase-1 edge, so every lane's
// vt_rdata is valid after the second edge (lanes 4n+2, 4n+3 straight from the
// BRAM outputs, which hold until the next VTRACE read).
//
// The chunk engine only drives eng_we / eng_re while vt_busy and core_req are
// both 0 (chunk_engine grant), so it never loses port A mid-access.
module world_mem #(
    parameter int LANES = 8
) (
    input  logic        clk,

    // VTRACE (physical addresses from world_remap)
    input  logic        vt_busy,
    input  logic        vt_phase,
    input  logic [16:0] vt_paddr [LANES],
    output logic [3:0]  vt_rdata [LANES],

    // core (physical address from world_remap)
    input  logic        core_req,
    input  logic        core_we,
    input  logic [16:0] core_paddr,
    input  logic [3:0]  core_wdata,

    // chunk engine (physical addresses)
    input  logic        eng_we,
    input  logic [16:0] eng_addr,
    input  logic [3:0]  eng_wdata,

    // copy 0 port A read data (core and engine)
    output logic [3:0]  a_rdata
);
    localparam int COPIES = LANES / 4;

    logic [16:0] a_addr_w;
    logic        a_we;
    logic [3:0]  a_wdata;

    always_comb begin
        a_addr_w = core_req ? core_paddr : eng_addr;
        a_we     = !vt_busy && (core_req ? core_we : eng_we);
        a_wdata  = core_req ? core_wdata : eng_wdata;
    end

    logic [3:0] dout_a [COPIES];

    for (genvar c = 0; c < COPIES; c++) begin : g_copy
        (* ram_style = "block" *) logic [3:0] mem [131072];

        logic [16:0] addr_a, addr_b;
        logic [3:0]  dout_b, hold_a, hold_b;

        assign addr_a = vt_busy ? vt_paddr[4*c + (vt_phase ? 2 : 0)] : a_addr_w;
        assign addr_b = vt_paddr[4*c + (vt_phase ? 3 : 1)];

        always_ff @(posedge clk) begin
            if (a_we) mem[addr_a] <= a_wdata;
            dout_a[c] <= mem[addr_a];
        end

        always_ff @(posedge clk) dout_b <= mem[addr_b];

        always_ff @(posedge clk) begin
            if (vt_busy && vt_phase) begin
                hold_a <= dout_a[c];
                hold_b <= dout_b;
            end
        end

        assign vt_rdata[4*c]     = hold_a;
        assign vt_rdata[4*c + 1] = hold_b;
        assign vt_rdata[4*c + 2] = dout_a[c];
        assign vt_rdata[4*c + 3] = dout_b;
    end

    assign a_rdata = dout_a[0];

endmodule
