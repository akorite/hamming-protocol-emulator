/*
 * Copyright (c) 2026 akorite
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Reduced-environment formal top for the BCU TX bit-stuffer proof.  RX, the
// pattern matcher, external-SCLK mode inputs, and the unrelated CRC seed
// interface are tied off so the induction problem concentrates on the TX
// serializer.  The protocol-relevant mode, bit-length, TX period, data and
// control inputs remain fully formal (and may therefore choose short periods
// in cover traces).
module bcu_stuffer_top (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        tx_push,
    input  wire [15:0] tx_data,
    input  wire        ctl_tx_arm,
    input  wire        ctl_eop_req,
    input  wire        ctl_stat_clr,
    input  wire        ctl_soft_rst,

    input  wire [2:0]  cfg_codec,
    input  wire [1:0]  cfg_stuff_mode,
    input  wire        cfg_rx_diff,
    input  wire [4:0]  cfg_bit_len_low,
    input  wire [7:0]  cfg_tx_period
);

    wire [15:0] cfg_mode = {cfg_rx_diff, 10'b0000000000,
                            cfg_stuff_mode, cfg_codec};
    wire [15:0] cfg_bit_len = {11'b00000000000, cfg_bit_len_low};
    wire [15:0] cfg_dpll = {cfg_tx_period, 8'h00};
    wire [7:0]  ctrl = {2'b00, ctl_soft_rst, ctl_stat_clr,
                        ctl_eop_req, 2'b00, ctl_tx_arm};

    wire [15:0] rx_data;
    wire [15:0] stat;
    wire [31:0] crc_state;
    wire        tx_p;
    wire        tx_n;
    wire        tx_oe;

    hamming_bcu dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .tx_push       (tx_push),
        .tx_data       (tx_data),
        .rx_pop        (1'b0),
        .rx_data       (rx_data),
        .stat          (stat),
        .crc_state     (crc_state),
        .ctrl          (ctrl),
        .cfg_mode      (cfg_mode),
        .cfg_crc_poly  (32'h00000000),
        .cfg_crc_state (32'h00000000),
        .crc_seed_we   (1'b0),
        .crc_lo_we     (1'b0),
        .crc_hi_we     (1'b0),
        .crc_seed16    (16'h0000),
        .cfg_bit_len   (cfg_bit_len),
        .cfg_pat_match (16'h0000),
        .cfg_pat_len   (16'h0000),
        .cfg_dpll      (cfg_dpll),
        .rx_p          (1'b0),
        .rx_n          (1'b0),
        .tx_p          (tx_p),
        .tx_n          (tx_n),
        .tx_oe         (tx_oe),
        .ext_sclk      (1'b0)
    );

    wire _unused = &{1'b0, rx_data, stat, crc_state, tx_p, tx_n, tx_oe};

endmodule

`default_nettype wire
