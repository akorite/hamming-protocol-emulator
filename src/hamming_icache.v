/*
 * Copyright (c) 2026 akorite
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Eight-line direct-mapped instruction loop cache. Each line stores one
// 32-bit SRAM 0 word (two 16-bit instruction slots). Lookup is combinational
// because the core executes from a hit in the same cycle; fills and global
// invalidation are synchronous. Invalidation has priority over a coincident
// fill so a stale pre-commit response can never become valid.
module hamming_icache (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        inv_all,

    input  wire [8:0]  rd_waddr,
    output wire        hit,
    output wire [31:0] rd_data,

    input  wire        fill_en,
    input  wire [8:0]  fill_waddr,
    input  wire [31:0] fill_data
);

  reg [7:0]  valid;
  reg [5:0]  tag  [0:7];
  reg [31:0] line [0:7];

  wire [2:0] rd_index = rd_waddr[2:0];
  wire [2:0] fill_index = fill_waddr[2:0];

  assign hit = valid[rd_index] && (tag[rd_index] == rd_waddr[8:3]) && !inv_all;
  assign rd_data = line[rd_index];

  always @(posedge clk) begin
    if (!rst_n) begin
      // Only validity is architecturally significant. Tag/data storage is
      // never read while invalid, so spending reset muxes on it is unnecessary.
      valid <= 8'h00;
    end else if (inv_all) begin
      valid <= 8'h00;
    end else if (fill_en) begin
      valid[fill_index] <= 1'b1;
      tag[fill_index] <= fill_waddr[8:3];
      line[fill_index] <= fill_data;
    end
  end

endmodule

