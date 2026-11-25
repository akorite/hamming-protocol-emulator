/*
 * Copyright (c) 2026 akorite
 * SPDX-License-Identifier: Apache-2.0
 */

`timescale 1ns/1ps
`default_nettype none

// Parameterized bit-serial CRC/LFSR engine (KTD4).  One data bit is shifted
// per step_en cycle.  LSB-first mode is the reflected right-shift form used
// by USB CRC5/CRC16 and Ethernet CRC-32; MSB-first mode is the left-shift
// form used by CAN CRC-15.  The state register always stays masked to the
// configured width so bits above the width never influence feedback.
module hamming_bcu_crc (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [4:0]  cfg_width_m1,  // crc width minus one (0..31 -> 1..32)
    input  wire        cfg_msb,       // 0 = LSB-first reflected, 1 = MSB-first
    input  wire [31:0] cfg_poly,

    input  wire        load_en,       // synchronous seed load
    input  wire [31:0] load_state,

    // Half-word seed writes from the CSR bank (the CSR holds no duplicate
    // state; this register is the single storage).  A write to
    // BCU_CRC_STATE_LO/HI pulses the matching enable with the new half
    // presented on load_state.  Full load_en takes priority.
    input  wire        lo_we,
    input  wire        hi_we,

    input  wire        step_en,
    input  wire        step_bit,

    output wire [31:0] state,
    output wire [4:0]  width_m1_out
);

  reg [31:0] state_r;
  assign state = state_r;
  assign width_m1_out = cfg_width_m1;

  wire [5:0]  width = {1'b0, cfg_width_m1} + 6'd1;
  wire [31:0] mask = (width >= 6'd32) ? 32'hFFFFFFFF :
                     ((32'h00000001 << width) - 32'h00000001);

  wire        msb_top = (state_r >> cfg_width_m1) & 32'h1;
  wire        fb_msb = msb_top ^ step_bit;
  wire        fb_lsb = state_r[0] ^ step_bit;

  wire [31:0] step_msb = ((state_r << 1) & mask) ^
                         (fb_msb ? (cfg_poly & mask) : 32'h00000000);
  wire [31:0] step_lsb = (({1'b0, state_r[31:1]}) & mask) ^
                         (fb_lsb ? (cfg_poly & mask) : 32'h00000000);

  wire [31:0] half_lo  = lo_we ? {state_r[31:16], load_state[15:0]} : state_r;
  wire [31:0] half_next = hi_we ? {load_state[15:0], half_lo[15:0]} : half_lo;

  always @(posedge clk) begin
    if (!rst_n) begin
      state_r <= 32'h00000000;
    end else if (load_en) begin
      state_r <= load_state & mask;
    end else if (lo_we | hi_we) begin
      state_r <= half_next & mask;
    end else if (step_en) begin
      state_r <= cfg_msb ? step_msb : step_lsb;
    end
  end

endmodule
