/*
 * Copyright (c) 2026 akorite
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Trivial formal smoke: a word written into the SRAM must come back on the
// read port. Drives the bounded 16-word FORMAL stub through a fixed
// write-then-read sequence.
module sram_smoke (
    input wire clk
);

  (* anyconst *) reg [3:0]  f_addr;
  (* anyconst *) reg [31:0] f_data;

  reg  [1:0]  phase = 2'd0;
  reg         men   = 1'b0;
  reg         wen   = 1'b0;
  reg         ren   = 1'b0;
  reg  [8:0]  addr  = 9'd0;
  reg  [31:0] din   = 32'd0;
  reg  [31:0] bm    = 32'd0;
  wire [31:0] dout;

  hamming_sram_2kb sram (
      .clk (clk),
      .men (men),
      .wen (wen),
      .ren (ren),
      .addr(addr),
      .din (din),
      .bm  (bm),
      .dout(dout)
  );

  always @(posedge clk) begin
    phase <= (phase == 2'd3) ? phase : phase + 2'd1;
    men  <= (phase <= 2'd1);
    wen  <= (phase == 2'd0);
    ren  <= (phase == 2'd1);
    addr <= {5'b0, f_addr};
    din  <= f_data;
    bm   <= 32'hFFFF_FFFF;
    if (phase == 2'd3)
      assert (dout == f_data);
  end

endmodule
