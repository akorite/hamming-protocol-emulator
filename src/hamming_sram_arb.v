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
  reg [31:0] bm_r;
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
      bm_r         <= 32'd0;
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
        bm_r   <= wmask_flat[sel*32 +: 32];
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
      .bm  (bm_r),
      .dout(dout)
  );

endmodule
