/*
 * Copyright (c) 2026 akorite
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// 2 KiB single-port SRAM bank wrapper. The default build instantiates the
// IHP foundry macro; -DTARGET_FPGA infers portable synchronous block RAM for
// the FPGA port; -DFORMAL substitutes a 16-word bounded stub for SymbiYosys.
//
// Semantics (matching the foundry model):
//   - signals are sampled on the rising edge of clk when men = 1;
//   - wen = 1 writes din into addr under the active-high bit mask bm;
//   - ren = 1 drives dout with the addressed word one cycle later;
//   - wen and ren are mutually exclusive by design contract.
module hamming_sram_2kb (
    input  wire        clk,
    input  wire        men,
    input  wire        wen,
    input  wire        ren,
    input  wire [8:0]  addr,
    input  wire [31:0] din,
    input  wire [31:0] bm,
    output wire [31:0] dout
);

`ifdef TARGET_FPGA

  reg  [31:0] mem [0:511];
  reg  [31:0] dout_r;

  always @(posedge clk) begin
    if (men && wen)
      mem[addr] <= (mem[addr] & ~bm) | (din & bm);
    if (men && ren)
      dout_r <= wen ? ((mem[addr] & ~bm) | (din & bm)) : mem[addr];
  end

  assign dout = dout_r;

`elsif FORMAL

  // Bounded 16-word memory for formal proofs: same port contract, depth 16.
  reg  [31:0] mem [0:15];
  reg  [31:0] dout_r;
  wire [3:0]  faddr = addr[3:0];

  always @(posedge clk) begin
    if (men && wen)
      mem[faddr] <= (mem[faddr] & ~bm) | (din & bm);
    if (men && ren)
      dout_r <= wen ? ((mem[faddr] & ~bm) | (din & bm)) : mem[faddr];
  end

  assign dout = dout_r;

`else

  // The instance name ("i_sram") is referenced by MACROS and
  // PDN_MACRO_CONNECTIONS in src/config.json through the parent instance.
  RM_IHPSG13_1P_512x32_c2_bm_bist i_sram (
      .A_CLK      (clk),
      .A_MEN      (men),
      .A_WEN      (wen),
      .A_REN      (ren),
      .A_ADDR     (addr),
      .A_DIN      (din),
      .A_DLY      (1'b1),               // must be tied high (macro datasheet)
      .A_DOUT     (dout),
      .A_BM       (bm),
      .A_BIST_CLK (1'b0),
      .A_BIST_EN  (1'b0),
      .A_BIST_MEN (1'b0),
      .A_BIST_WEN (1'b0),
      .A_BIST_REN (1'b0),
      .A_BIST_ADDR(9'd0),
      .A_BIST_DIN (32'd0),
      .A_BIST_BM  (32'd0)
  );

`endif

`ifdef FORMAL
  // Read enable and write enable are mutually exclusive by contract.
  always @(posedge clk) begin
    if (men && wen) assume (!ren);
  end
`endif

endmodule
