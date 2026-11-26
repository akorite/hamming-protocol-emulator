/*
 * Copyright (c) 2026 akorite
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Pipelined round-robin arbiter for one 2 KiB SRAM bank.
//
// Client handshake: req is a level and grant is combinational. A transfer is
// accepted at the rising clock edge for which both are true, so a client that
// observes grant may retire or replace its request after that edge without a
// duplicate acceptance. One request is accepted per cycle (II=1). Read data
// returns exactly three accepting edges later in uncontended operation.
//
// Macro inputs and the returning data are registered. write_commit pulses
// just after the edge at which the macro actually samples a write, which is
// the point at which SRAM 0 loop-cache contents become stale.
module hamming_sram_arb #(
    parameter NCLIENTS = 6
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire [NCLIENTS-1:0]     req,
    input  wire [NCLIENTS-1:0]     we,
    input  wire [NCLIENTS*9-1:0]   addr_flat,
    input  wire [NCLIENTS*32-1:0]  wdata_flat,
    input  wire [NCLIENTS*32-1:0]  wmask_flat,
    output wire [NCLIENTS-1:0]     grant,
    output reg  [NCLIENTS-1:0]     rvalid,
    output reg  [31:0]             rdata,
    output reg                     write_commit
);

  localparam CLIENT_W = (NCLIENTS <= 2) ? 1 : $clog2(NCLIENTS);

  reg [CLIENT_W-1:0] rr_base;
  reg [CLIENT_W-1:0] sel;
  reg                found;
  reg [NCLIENTS-1:0] grant_comb;

  integer scan_i;
  integer scan_idx;

  // Strict round-robin scan. rr_base is one past the most recently granted
  // client, so a continuously requesting client set rotates in index order.
  always @* begin
    grant_comb = {NCLIENTS{1'b0}};
    sel = {CLIENT_W{1'b0}};
    found = 1'b0;
    scan_idx = rr_base;
    for (scan_i = 0; scan_i < NCLIENTS; scan_i = scan_i + 1) begin
      if (!found && req[scan_idx]) begin
        sel = scan_idx;
        found = 1'b1;
      end
      scan_idx = (scan_idx == NCLIENTS-1) ? 0 : scan_idx + 1;
    end
    if (found)
      grant_comb[sel] = 1'b1;
  end

  assign grant = grant_comb;

  // Registered macro inputs.
  reg        men_r;
  reg        wen_r;
  reg        ren_r;
  reg [8:0]  addr_r;
  reg [31:0] din_r;
  reg [3:0]  bm4_r;   // byte-lane mask; clients are byte-aligned
  wire [31:0] dout;

  // Return-tag pipeline. A read accepted at edge T has its macro data before
  // edge T+2, is captured then, and is presented to the client at edge T+3.
  reg                v0;
  reg                v1;
  reg                v2;
  reg [CLIENT_W-1:0] id0;
  reg [CLIENT_W-1:0] id1;
  reg [CLIENT_W-1:0] id2;
  reg [31:0]         cap_data;

  always @(posedge clk) begin
    if (!rst_n) begin
      rr_base      <= {CLIENT_W{1'b0}};
      men_r        <= 1'b0;
      wen_r        <= 1'b0;
      ren_r        <= 1'b0;
      addr_r       <= 9'd0;
      din_r        <= 32'd0;
      bm4_r        <= 4'd0;
      write_commit <= 1'b0;
      v0           <= 1'b0;
      v1           <= 1'b0;
      v2           <= 1'b0;
      id0          <= {CLIENT_W{1'b0}};
      id1          <= {CLIENT_W{1'b0}};
      id2          <= {CLIENT_W{1'b0}};
      cap_data     <= 32'd0;
      rvalid       <= {NCLIENTS{1'b0}};
      rdata        <= 32'd0;
    end else begin
      if (found) begin
        men_r  <= 1'b1;
        wen_r  <= we[sel];
        ren_r  <= !we[sel];
        addr_r <= addr_flat[sel*9 +: 9];
        din_r  <= wdata_flat[sel*32 +: 32];
        bm4_r  <= {|wmask_flat[sel*32+24 +: 8], |wmask_flat[sel*32+16 +: 8],
                     |wmask_flat[sel*32+8 +: 8],  |wmask_flat[sel*32 +: 8]};
        rr_base <= (sel == NCLIENTS-1) ? {CLIENT_W{1'b0}} : sel + 1'b1;
      end else begin
        men_r <= 1'b0;
        wen_r <= 1'b0;
        ren_r <= 1'b0;
      end

      // The macro samples the previous cycle's registered inputs at this
      // edge. That is the commit point exposed to loop-cache invalidation.
      write_commit <= men_r && wen_r;

      v0  <= found && !we[sel];
      id0 <= sel;
      v1  <= v0;
      id1 <= id0;
      v2  <= v1;
      id2 <= id1;

      if (v1)
        cap_data <= dout;

      rvalid <= {NCLIENTS{1'b0}};
      if (v2) begin
        rvalid[id2] <= 1'b1;
        rdata <= cap_data;
      end
    end
  end

  hamming_sram_2kb i_sram (
      .clk (clk),
      .men (men_r),
      .wen (wen_r),
      .ren (ren_r),
      .addr(addr_r),
      .din (din_r),
      .bm  ({{8{bm4_r[3]}}, {8{bm4_r[2]}}, {8{bm4_r[1]}}, {8{bm4_r[0]}}}),
      .dout(dout)
  );

`ifdef FORMAL
  // ------------------------------------------------------------
  // Modular formal contract (SymbiYosys only; no silicon logic).
  // ------------------------------------------------------------
  reg f_past_valid;
  initial f_past_valid = 1'b0;

  // Exact response monitor.  A read present on the request/grant interface
  // at the accepting edge appears as rvalid three accepting edges later.
  // In the state-transition encoding used by yosys-smtbmc that is the
  // fourth state after the input state, hence the four-stage witness.
  wire [NCLIENTS-1:0] f_read_grant = grant & ~we;
  reg  [NCLIENTS-1:0] f_read_m1;
  reg  [NCLIENTS-1:0] f_read_m2;
  reg  [NCLIENTS-1:0] f_read_m3;
  reg  [NCLIENTS-1:0] f_read_m4;
  wire [NCLIENTS-1:0] f_delayed_grant;

  always @(posedge clk) begin
    f_past_valid <= 1'b1;
    if (!f_past_valid)
      assume (!rst_n);

    if (!rst_n) begin
      f_read_m1 <= {NCLIENTS{1'b0}};
      f_read_m2 <= {NCLIENTS{1'b0}};
      f_read_m3 <= {NCLIENTS{1'b0}};
      f_read_m4 <= {NCLIENTS{1'b0}};
    end else begin
      f_read_m1 <= f_read_grant;
      f_read_m2 <= f_read_m1;
      f_read_m3 <= f_read_m2;
      f_read_m4 <= f_read_m3;
    end

    if (f_past_valid) begin
      // Structural grant contract: at most one client, never a grant
      // without a same-cycle request, and a grant for exactly one client
      // whenever any request is present (initiation interval one).
      assert (rr_base < NCLIENTS);
      assert (found == (|req));
      assert ((grant & (grant - {{(NCLIENTS-1){1'b0}}, 1'b1})) == {NCLIENTS{1'b0}});
      if (found) begin
        assert (req[sel]);
        assert (grant[sel]);
      end else begin
        assert (grant == {NCLIENTS{1'b0}});
      end

      // The macro port is a single registered owner: read and write are
      // never simultaneously requested, and write_commit is precisely the
      // delayed macro write edge.
      assert (!(men_r && wen_r && ren_r));
      if ($past(rst_n)) begin
        assert (write_commit == ($past(men_r) && $past(wen_r)));
        assert (v1 == $past(v0));
        assert (v2 == $past(v1));
        assert (id1 == $past(id0));
        assert (id2 == $past(id1));
      end

      // Exact, non-stale read response: rvalid is the acceptance vector
      // delayed by the documented three accepting edges and is otherwise
      // empty.  Because the acceptance vector is one-hot, so is rvalid.
      assert (rvalid == f_read_m4);
      assert ((rvalid & (rvalid - {{(NCLIENTS-1){1'b0}}, 1'b1})) == {NCLIENTS{1'b0}});

      // Non-vacuity and reset-space covers.  The re-reset covers require a
      // prior high reset sample, so the cover task cannot satisfy them with
      // only the mandatory initial reset cycle.
      cover (rst_n && (&req));
      cover (|rvalid);
      cover (write_commit);
      cover ($past(rst_n) && !rst_n && (v0 || v1 || v2));
      cover ($past(rst_n) && !rst_n && !(v0 || v1 || v2) && !write_commit);
      cover (|f_delayed_grant);
    end
  end

  // Per-client bounded-response proof.  f_dist is the cyclic distance from
  // rr_base to this client.  The key inductive invariant is
  //     consecutive_denials + cyclic_distance <= NCLIENTS-1.
  // Every cycle in which this client is skipped grants a client ahead of it,
  // moving rr_base beyond that grant and strictly decreasing the distance.
  // When the distance reaches zero the combinational scan necessarily grants
  // this client.  Thus a continuously asserted request is accepted after at
  // most NCLIENTS-1 intervening grants (within NCLIENTS accepting edges).
  genvar fg;
  generate
    for (fg = 0; fg < NCLIENTS; fg = fg + 1) begin : gen_formal_client
      wire [31:0] f_dist = (fg >= rr_base) ?
                           (32'd0 + fg - rr_base) :
                           (32'd0 + fg + NCLIENTS - rr_base);
      reg [3:0] f_wait_cnt;

      assign f_delayed_grant[fg] = req[fg] && grant[fg] && (f_wait_cnt != 4'd0);

      always @(posedge clk) begin
        if (!rst_n)
          f_wait_cnt <= 4'd0;
        else if (!req[fg] || grant[fg])
          f_wait_cnt <= 4'd0;
        else if (f_wait_cnt != 4'hf)
          f_wait_cnt <= f_wait_cnt + 4'd1;

        if (f_past_valid && rst_n) begin
          assert (f_wait_cnt <= (NCLIENTS-1));
          assert (({28'd0, f_wait_cnt} + f_dist) <= (NCLIENTS-1));
          if (req[fg] && (f_dist == 32'd0))
            assert (grant[fg]);
          if (grant[fg])
            assert (req[fg]);
        end
      end
    end
  endgenerate
`endif

endmodule
