/*
 * Copyright (c) 2026 akorite
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Minimal dual-SRAM smoke top for the Hamming protocol emulator.
//
// Exercises both 2 KiB foundry SRAM banks from the pins, one command per
// clock, sampled on the rising edge of clk:
//   ui_in[7:5] command: 3'b000 idle
//                       3'b001 ADDR   - address <= {ui_in[0], uio_in}
//                       3'b010 WRITE  - write byte to the selected bank/lane
//                       3'b011 WRBOTH - write byte to both banks, same lane
//                       3'b100 READ   - read addressed word in BOTH banks
//   ui_in[4]   bank select (0 = SRAM 0, 1 = SRAM 1)
//   ui_in[3:2] byte lane (0..3) for the A_BM write mask and read byte select
//   ui_in[1:0] extended address (bit 0 = address[8])
//   uio_in     command data byte (address low / write data)
//   uo_out     selected byte of the last word read, byte = {bank, lane}
//
// A command registers at the sampled edge and reaches the SRAM inputs one
// edge later; a READ result lands in the read registers two edges after the
// command edge and stays visible until the next read completes. During and
// after reset all outputs are strictly 0 and both banks are disabled.
module tt_um_akorite_hamming (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  localparam CMD_IDLE   = 3'b000;
  localparam CMD_ADDR   = 3'b001;
  localparam CMD_WRITE  = 3'b010;
  localparam CMD_WRBOTH = 3'b011;
  localparam CMD_READ   = 3'b100;

  reg  [2:0]  cmd_q;
  reg         bank_q;
  reg  [1:0]  lane_q;
  reg         ahi_q;
  reg  [7:0]  data_q;
  reg  [8:0]  addr;

  reg  [31:0] rd_data0;
  reg  [31:0] rd_data1;
  reg         cap_q;

  // The registered command occupies the SRAM input for exactly one cycle.
  wire        cmd_read  = (cmd_q == CMD_READ);
  wire        men0 = cmd_read || (cmd_q == CMD_WRBOTH) || (cmd_q == CMD_WRITE && !bank_q);
  wire        men1 = cmd_read || (cmd_q == CMD_WRBOTH) || (cmd_q == CMD_WRITE &&  bank_q);
  wire        wen  = (cmd_q == CMD_WRITE) || (cmd_q == CMD_WRBOTH);
  wire [31:0] sram_din = {data_q, data_q, data_q, data_q};
  wire [31:0] sram_bm  = 32'hFF << (lane_q * 8);
  wire [31:0] dout0;
  wire [31:0] dout1;

  always @(posedge clk) begin
    if (!rst_n) begin
      cmd_q  <= CMD_IDLE;
      bank_q <= 1'b0;
      lane_q <= 2'b00;
      ahi_q  <= 1'b0;
      data_q <= 8'h00;
      addr   <= 9'd0;
      cap_q  <= 1'b0;
      rd_data0 <= 32'h0;
      rd_data1 <= 32'h0;
    end else begin
      cap_q  <= cmd_read;
      cmd_q  <= ui_in[7:5];
      bank_q <= ui_in[4];
      lane_q <= ui_in[3:2];
      ahi_q  <= ui_in[0];
      data_q <= uio_in;
      if (cmd_q == CMD_ADDR)
        addr <= {ahi_q, data_q};
      if (cap_q) begin
        rd_data0 <= dout0;
        rd_data1 <= dout1;
      end
    end
  end

  hamming_sram_2kb i_sram0 (
      .clk (clk),
      .men (men0),
      .wen (wen),
      .ren (cmd_read),
      .addr(addr),
      .din (sram_din),
      .bm  (sram_bm),
      .dout(dout0)
  );

  hamming_sram_2kb i_sram1 (
      .clk (clk),
      .men (men1),
      .wen (wen),
      .ren (cmd_read),
      .addr(addr),
      .din (sram_din),
      .bm  (sram_bm),
      .dout(dout1)
  );

  wire [7:0] rd_byte0 = (lane_q == 2'd0) ? rd_data0[7:0]   :
                        (lane_q == 2'd1) ? rd_data0[15:8]  :
                        (lane_q == 2'd2) ? rd_data0[23:16] : rd_data0[31:24];
  wire [7:0] rd_byte1 = (lane_q == 2'd0) ? rd_data1[7:0]   :
                        (lane_q == 2'd1) ? rd_data1[15:8]  :
                        (lane_q == 2'd2) ? rd_data1[23:16] : rd_data1[31:24];

  assign uo_out  = bank_q ? rd_byte1 : rd_byte0;
  assign uio_out = 8'h00;
  assign uio_oe  = 8'h00;

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, ui_in[1], 1'b0};

endmodule
