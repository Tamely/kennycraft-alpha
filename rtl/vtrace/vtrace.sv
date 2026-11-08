// VTRACE ray unit (Tai page, VTRACE Ray Unit; Shared page, Interfaces, Ray unit).
//
// Hardware Amanatides & Woo voxel traversal for LANES rays at once, v1
// multi-cycle. VSET (cfg_we) loads one field of each lane in lane_mask:
//   0 start cell {z[5:0], y[4:0], x[5:0]} in bits 16:0
//   1-3 tMax x, y, z (Q16.16)    4-6 tDelta x, y, z (Q16.16)
//   7 step signs, bit 0 x, 1 y, 2 z, 1 = stepping negative
// The walk updates cell and tMax in place.
//
// One step (wiki "One step", ties as written there: x only on a strict
// minimum, then y if tMaxY < tMaxZ, else z; tMax compares are signed):
//   step   (an edge, not a state) move the cell one block on the chosen
//          axis, tMax += tDelta (saturating at 0x7FFF_FFFF), record axis,
//          sign, t = old tMax and the step count. world_addr follows the
//          cell, so it is held until the next step.
//   WAIT   WORLD_LATENCY cycles with world_addr held. world_phase is the
//          index of the WAIT cycle (0, 1): world_mem reads lanes 4n, 4n + 1
//          in phase 0 and 4n + 2, 4n + 3 in phase 1.
//   CHECK  world_data is valid: outside 0..63 / 0..31 / 0..63 finishes the
//          lane with hit = 0, a non-air block finishes it with hit = 1 and
//          the block type. The walk ends when every lane is finished or
//          max_steps steps have been taken; otherwise the lanes still live
//          take their next step on the CHECK edge itself.
// That is WORLD_LATENCY + 1 = 3 cycles per step with two world copies
// (D17), the world memory busy in both WAIT cycles. The first step happens
// on the start edge, so the first cycle with busy high is the first
// world_addr cycle.
//
// The start cell is never tested. start with max_steps = 0 or an empty
// lane_mask raises done the next cycle and changes nothing. Lanes outside
// lane_mask keep their state and results. On a miss (left the world or step
// cap) the result keeps the last step's axis, sign, t and cell, block type 0;
// hit_cell packs the low bits of the cell, so x = 64 reads as 0 and -1 as 63.
// done is a one-cycle pulse in the first cycle after the walk with busy low;
// the results are stable from then until the next start.
module vtrace #(parameter int LANES = 8, parameter int WORLD_LATENCY = 2) (
  input  logic             clk,
  input  logic             rst,
  // VSET: write one field for all active lanes
  input  logic             cfg_we,
  input  logic [2:0]       cfg_field,
  input  logic [LANES-1:0] lane_mask,
  input  logic [31:0]      cfg_data   [LANES],
  // VTRACE
  input  logic             start,       // one-cycle pulse
  input  logic [7:0]       max_steps,
  output logic             busy,
  output logic             done,        // one-cycle pulse
  // results, valid after done, read by VTRACE rd and VGET
  output logic [15:0]      result     [LANES],
  output logic [16:0]      hit_cell   [LANES],
  output logic [31:0]      t_hit      [LANES],
  // world read ports, one per lane. world_addr is held for WORLD_LATENCY
  // cycles and world_data is valid WORLD_LATENCY cycles after it (2 with two
  // world copies, D17; 1 with four)
  output logic [16:0]      world_addr [LANES],
  output logic             world_phase,  // WAIT cycle index (world_mem two-phase read)
  input  logic [3:0]       world_data [LANES]
);
  localparam int WL_W = (WORLD_LATENCY > 1) ? $clog2(WORLD_LATENCY) : 1;

  typedef enum logic [1:0] { S_IDLE, S_WAIT, S_CHECK } state_t;

  state_t           state;
  logic [WL_W-1:0]  wait_cnt;
  logic [7:0]       step_cnt;   // steps taken so far in this walk
  logic [7:0]       cap;        // max_steps latched at start
  logic [LANES-1:0] live;       // lane still walking
  logic [LANES-1:0] live_next;  // live after this CHECK

  wire launch   = (state == S_IDLE) && start && (max_steps != 8'd0) && (lane_mask != '0);
  wire do_check = (state == S_CHECK);
  wire finish   = do_check && ((live_next == '0) || (step_cnt == cap));
  // lanes that take a step on this edge, and the step number they record:
  // the start edge, or a CHECK that does not finish the walk
  wire [LANES-1:0] stepping = launch ? lane_mask : (do_check && !finish) ? live_next : '0;
  wire [7:0]       step_num = launch ? 8'd1 : step_cnt + 8'd1;

  assign busy        = (state != S_IDLE);
  assign world_phase = (state == S_WAIT) && (wait_cnt != '0);

  always_ff @(posedge clk) begin
    if (rst) begin
      state <= S_IDLE;
      done  <= 1'b0;
      live  <= '0;
    end else begin
      done <= 1'b0;
      unique case (state)
        S_IDLE: begin
          if (start) begin
            if (launch) begin
              live     <= lane_mask;
              cap      <= max_steps;
              step_cnt <= 8'd1;
              wait_cnt <= '0;
              state    <= S_WAIT;
            end else begin
              done <= 1'b1;
            end
          end
        end
        S_WAIT: begin
          if (wait_cnt == WL_W'(WORLD_LATENCY - 1)) state <= S_CHECK;
          else wait_cnt <= wait_cnt + 1'b1;
        end
        S_CHECK: begin
          live <= live_next;
          if (finish) begin
            done  <= 1'b1;
            state <= S_IDLE;
          end else begin
            step_cnt <= step_num;     // the live lanes step on this edge
            wait_cnt <= '0;
            state    <= S_WAIT;
          end
        end
        default: state <= S_IDLE;
      endcase
    end
  end

  for (genvar l = 0; l < LANES; l++) begin : g_lane
    logic signed [6:0]  cx, cy, cz;        // cell, signed so leaving is visible
    logic signed [31:0] tmx, tmy, tmz;     // tMax, Q16.16
    logic signed [31:0] tdx, tdy, tdz;     // tDelta, Q16.16
    logic [2:0]         neg;               // step signs, 1 = negative
    logic               r_hit, r_sign;
    logic [1:0]         r_axis;
    logic [3:0]         r_type;
    logic [7:0]         r_steps;
    logic [31:0]        r_t;

    // ---- STEP: pick the axis with the smallest tMax ----
    wire                sel_x = (tmx < tmy) && (tmx < tmz);
    wire                sel_y = !sel_x && (tmy < tmz);
    wire                sel_z = !sel_x && !sel_y;
    wire signed [31:0]  tm    = sel_x ? tmx : sel_y ? tmy : tmz;
    wire signed [31:0]  td    = sel_x ? tdx : sel_y ? tdy : tdz;
    wire signed [32:0]  sum   = 33'(tm) + 33'(td);
    // saturate at 0x7FFF_FFFF on positive overflow
    wire signed [31:0]  tm_n  = (sum[32:31] == 2'b01) ? 32'sh7FFF_FFFF : sum[31:0];
    wire [1:0]          axis  = sel_x ? 2'd0 : sel_y ? 2'd1 : 2'd2;
    wire                sgn   = neg[axis];
    wire signed [6:0]   delta = sgn ? -7'sd1 : 7'sd1;

    // ---- CHECK ----
    wire oob  = cx[6] | cy[6] | cy[5] | cz[6];
    wire solid = (world_data[l] != 4'd0);
    assign live_next[l] = live[l] && !oob && !solid;

    always_ff @(posedge clk) begin
      if (cfg_we && lane_mask[l]) begin
        unique case (cfg_field)
          3'd0: begin
            cx <= {1'b0, cfg_data[l][5:0]};
            cy <= {2'b0, cfg_data[l][10:6]};
            cz <= {1'b0, cfg_data[l][16:11]};
          end
          3'd1: tmx <= cfg_data[l];
          3'd2: tmy <= cfg_data[l];
          3'd3: tmz <= cfg_data[l];
          3'd4: tdx <= cfg_data[l];
          3'd5: tdy <= cfg_data[l];
          3'd6: tdz <= cfg_data[l];
          3'd7: neg <= cfg_data[l][2:0];
        endcase
      end
      if (stepping[l]) begin
        if (sel_x) begin cx <= cx + delta; tmx <= tm_n; end
        if (sel_y) begin cy <= cy + delta; tmy <= tm_n; end
        if (sel_z) begin cz <= cz + delta; tmz <= tm_n; end
        r_axis  <= axis;
        r_sign  <= sgn;
        r_t     <= tm;
        r_steps <= step_num;
      end
      if (do_check && live[l]) begin
        r_hit  <= !oob && solid;
        r_type <= oob ? 4'd0 : world_data[l];
      end
    end

    assign result[l]     = {r_steps, r_type, r_sign, r_axis, r_hit};
    assign hit_cell[l]   = {cz[5:0], cy[4:0], cx[5:0]};
    assign t_hit[l]      = r_t;
    assign world_addr[l] = {cy[4:0], cz[5:0], cx[5:0]};

    // cfg_data bits above the field widths are ignored
    /* verilator lint_off UNUSEDSIGNAL */
    wire unused_cfg = ^cfg_data[l][31:17];
    /* verilator lint_on UNUSEDSIGNAL */
  end
endmodule
