// vtrace_tb: the VTRACE ray unit against golden vectors from Kennedy's C
// reference (tb/vtrace/golden.c), run with +data=<dir>.
//
// Per group of 8 lanes: VSET the 8 fields (lanes outside lane_mask get random
// cfg_data that must be ignored), optionally probe start with max_steps = 0
// and with an empty lane_mask (done the next cycle, busy never high, no output
// changes), then start the walk and check
//   - busy stays high and done pulses exactly once, N * (WORLD_LATENCY + 1) + 1
//     cycles after start, N = the most steps any active lane takes;
//   - active lanes on a hit: result, hit_cell and t_hit equal the golden
//     value bit for bit; on a miss only hit and steps (result[0] and
//     result[15:8]) must match, the other fields being undefined (Shared,
//     VTRACE result; Tai_2 P6 item 5);
//   - lanes outside lane_mask: outputs unchanged.
// World model: WORLD_LATENCY = 1 is a registered read of every lane's address.
// WORLD_LATENCY = 2 is the world_mem two-phase read (D17) on the DUT's
// world_phase (its WAIT cycle index): lanes 4n, 4n+1 sample in phase 0 and
// 4n+2, 4n+3 in phase 1, and all data is valid after the second edge. Data is
// random while busy is low.
module vtrace_tb;
  localparam int LANES = 8;
  parameter int WORLD_LATENCY = 2;
  localparam int GW = 3 + LANES * 11;   // words per group

  logic             clk = 1'b0;
  logic             rst = 1'b1;
  logic             cfg_we = 1'b0;
  logic [2:0]       cfg_field = '0;
  logic [LANES-1:0] lane_mask = '0;
  logic [31:0]      cfg_data   [LANES];
  logic             start = 1'b0;
  logic [7:0]       max_steps = '0;
  logic             busy, done;
  logic [15:0]      result     [LANES];
  logic [16:0]      hit_cell   [LANES];
  logic [31:0]      t_hit      [LANES];
  logic [16:0]      world_addr [LANES];
  logic             world_phase;
  logic [3:0]       world_data [LANES];

  vtrace #(.LANES(LANES), .WORLD_LATENCY(WORLD_LATENCY)) dut (.*);

  always #5 clk = ~clk;

  // ---------------------------------------------------------------- world ----
  logic [3:0] wmem [131072];

  always_ff @(posedge clk) begin
    for (int l = 0; l < LANES; l++) begin
      if (WORLD_LATENCY == 1) world_data[l] <= wmem[world_addr[l]];
      else if (!busy) world_data[l] <= 4'($urandom);
      else if (((l % 4) / 2) == int'(world_phase)) world_data[l] <= wmem[world_addr[l]];
    end
  end

  // done must be a one-cycle pulse and never coincide with busy
  int done_pulses = 0;
  always @(posedge clk) begin
    if (!rst && done) begin
      done_pulses++;
      if (busy) $fatal(1, "done and busy high together");
    end
  end

  // ---------------------------------------------------------------- data ----
  logic [31:0] grp [];
  string       dir;
  int          errors = 0, checked = 0, hits = 0, cycles_total = 0, steps_total = 0;

  task automatic load(input string set, output int n);
    int fd, c;
    logic [31:0] w;
    $readmemh({dir, "/", set, "_world.hex"}, wmem);
    fd = $fopen({dir, "/", set, "_groups.hex"}, "r");
    if (fd == 0) $fatal(1, "cannot open %s/%s_groups.hex", dir, set);
    c = $fscanf(fd, "%h", w);
    n = int'(w);
    grp = new[n * GW];
    for (int i = 0; i < n * GW; i++) begin
      c = $fscanf(fd, "%h", w);
      if (c != 1) $fatal(1, "%s_groups.hex short at word %0d", set, i);
      grp[i] = w;
    end
    $fclose(fd);
  endtask

  function automatic logic [31:0] gw(input int g, input int k); return grp[g * GW + k]; endfunction
  function automatic logic [31:0] lw(input int g, input int l, input int k); return gw(g, 3 + l * 11 + k); endfunction

  // ---------------------------------------------------------------- driving ----
  task automatic vset(input int g, input logic [LANES-1:0] mask);
    for (int f = 0; f < 8; f++) begin
      @(negedge clk);
      cfg_we    = 1'b1;
      cfg_field = 3'(f);
      lane_mask = mask;
      for (int l = 0; l < LANES; l++) cfg_data[l] = lw(g, l, f);
    end
    @(negedge clk);
    cfg_we    = 1'b0;
    lane_mask = 8'($urandom);
    for (int l = 0; l < LANES; l++) cfg_data[l] = $urandom;
  endtask

  logic [15:0] p_res  [LANES];
  logic [16:0] p_cell [LANES];
  logic [31:0] p_t    [LANES];

  task automatic snapshot();
    for (int l = 0; l < LANES; l++) begin
      p_res[l] = result[l]; p_cell[l] = hit_cell[l]; p_t[l] = t_hit[l];
    end
  endtask

  task automatic expect_unchanged(input string what, input logic [LANES-1:0] lanes);
    for (int l = 0; l < LANES; l++)
      if (lanes[l] && (result[l] !== p_res[l] || hit_cell[l] !== p_cell[l] || t_hit[l] !== p_t[l])) begin
        $display("ERROR %s: lane %0d changed: %h %h %h -> %h %h %h", what, l,
                 p_res[l], p_cell[l], p_t[l], result[l], hit_cell[l], t_hit[l]);
        errors++;
      end
  endtask

  // start that must finish the next cycle without touching anything
  task automatic probe_nop(input string what, input logic [LANES-1:0] mask, input logic [7:0] ms);
    int d0;
    snapshot();
    d0 = done_pulses;
    @(negedge clk);
    start = 1'b1; lane_mask = mask; max_steps = ms;
    @(negedge clk);
    start = 1'b0; max_steps = 8'($urandom);
    if (!done || busy) begin
      $display("ERROR %s: done %b busy %b the cycle after start", what, done, busy); errors++;
    end
    @(negedge clk);
    if (done || busy || done_pulses != d0 + 1) begin
      $display("ERROR %s: done %b busy %b, %0d done pulses", what, done, busy, done_pulses - d0); errors++;
    end
    expect_unchanged(what, '1);
  endtask

  task automatic run_group(input string set, input int g);
    logic [LANES-1:0] mask;
    logic [7:0]       ms;
    int               nmax, cyc, d0;
    mask = LANES'(gw(g, 0));
    ms   = 8'(gw(g, 1));

    snapshot();
    vset(g, mask);
    // inactive lanes' cell is untouched by VSET
    expect_unchanged($sformatf("%s group %0d VSET", set, g), ~mask);
    if ((gw(g, 2) & 32'd1) != 0) begin
      probe_nop($sformatf("%s group %0d max_steps = 0", set, g), mask, 8'd0);
      probe_nop($sformatf("%s group %0d empty lane_mask", set, g), '0, ms);
    end
    snapshot();

    nmax = 0;
    for (int l = 0; l < LANES; l++)
      if (mask[l] && int'(lw(g, l, 8) >> 8) > nmax) nmax = int'(lw(g, l, 8) >> 8);

    d0 = done_pulses;
    @(negedge clk);
    start = 1'b1; lane_mask = mask; max_steps = ms;
    @(negedge clk);
    start = 1'b0; lane_mask = 8'($urandom); max_steps = 8'($urandom);
    cyc = 1;
    while (!done) begin
      if (!busy) begin $display("ERROR %s group %0d: busy low before done", set, g); errors++; end
      @(negedge clk);
      cyc++;
      if (cyc > 300 * 4) $fatal(1, "%s group %0d: no done", set, g);
    end
    if (cyc != nmax * (WORLD_LATENCY + 1) + 1) begin
      $display("ERROR %s group %0d: done after %0d cycles, want %0d steps x %0d + 1", set, g, cyc, nmax,
               WORLD_LATENCY + 1);
      errors++;
    end
    cycles_total += cyc;
    steps_total  += nmax;

    for (int l = 0; l < LANES; l++) begin
      logic [15:0] er;
      logic [16:0] ec;
      logic [31:0] et;
      if (!mask[l]) continue;
      er = 16'(lw(g, l, 8));
      ec = 17'(lw(g, l, 9));
      et = lw(g, l, 10);
      checked++;
      if (er[0]) begin
        hits++;
        if (result[l] === er && hit_cell[l] === ec && t_hit[l] === et) continue;
      end else if (result[l][0] === 1'b0 && result[l][15:8] === er[15:8]) begin
        continue;
      end
      $display("ERROR %s group %0d lane %0d ms %0d %s: got result %h cell %h t %h, want %h %h %h",
               set, g, l, ms, er[0] ? "hit" : "miss (hit and steps only)",
               result[l], hit_cell[l], t_hit[l], er, ec, et);
      $display("  fields %h %h %h %h %h %h %h %h", lw(g, l, 0), lw(g, l, 1), lw(g, l, 2), lw(g, l, 3),
               lw(g, l, 4), lw(g, l, 5), lw(g, l, 6), lw(g, l, 7));
      errors++;
    end
    expect_unchanged($sformatf("%s group %0d inactive lanes", set, g), ~mask);

    @(negedge clk);
    if (done || done_pulses != d0 + 1) begin
      $display("ERROR %s group %0d: done not a one-cycle pulse", set, g); errors++;
    end
    // results hold after done
    snapshot();
    repeat (3) @(negedge clk);
    expect_unchanged($sformatf("%s group %0d after done", set, g), '1);
    if (errors > 20) $fatal(1, "too many errors");
  endtask

  task automatic run_set(input string set);
    int n, c0, s0, k0, h0;
    load(set, n);
    c0 = cycles_total; s0 = steps_total; k0 = checked; h0 = hits;
    for (int g = 0; g < n; g++) run_group(set, g);
    $display("vtrace_tb: %s: %0d groups, %0d rays (%0d hits), %0d walk cycles for %0d steps (%0.2f cycles/step)",
             set, n, checked - k0, hits - h0, cycles_total - c0, steps_total - s0,
             real'(cycles_total - c0) / real'(steps_total - s0));
  endtask

  initial begin
    if (!$value$plusargs("data=%s", dir)) $fatal(1, "need +data=<dir>");
    for (int l = 0; l < LANES; l++) cfg_data[l] = '0;
    repeat (3) @(negedge clk);
    rst = 1'b0;
    if (busy || done) $fatal(1, "busy/done after reset");

    // reset state: nothing walks; max_steps = 0 / empty mask before any VSET
    probe_nop("after reset, max_steps = 0", '1, 8'd0);
    probe_nop("after reset, empty lane_mask", '0, 8'd200);

    run_set("edge");
    run_set("rand");

    $display("vtrace_tb: WORLD_LATENCY = %0d, %0d rays checked: %0d hits bit for bit, %0d misses on hit and steps",
             WORLD_LATENCY, checked, hits, checked - hits);
    if (errors != 0) $fatal(1, "vtrace_tb: FAIL, %0d errors", errors);
    $display("vtrace_tb: PASS");
    $finish;
  end
endmodule
