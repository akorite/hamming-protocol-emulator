/*
 * Copyright (c) 2026 akorite
 * SPDX-License-Identifier: Apache-2.0
 */

`timescale 1ns/1ps
`default_nettype none

// Fixed 2-tier (8 + 8) feedforward 16-LUT pin logic matrix with input
// synchronizers, per-uio-pin 4-tap glitch filters, and the 23-channel
// 16:1 output pin router (KTD5).  The structure is strictly feedforward:
// tier-2 LUTs consume tier-1 outputs, never the reverse, so no
// combinational loop can exist regardless of the CSR configuration.
//
// Router channel order:  0..7 uio_out data, 8..15 uio_oe, 16..22 uo_out.
// Router sources: 0 const0, 1 const1, 2 replay_data[ch[3:0]], 3..10
// LUT8..15, 11 paired core signal, 12/13/14 paired BCU tx_p/tx_n/tx_oe,
// 15 const0.  Core/BCU pairing uses channel index bits [1:0].  The source-11
// core signal depends on the channel group so every core can drive four
// independent output bits and two independent oe bits:
//   uio data ch 0..7    core out bit ch/4  (uio pins k and k+4 = bits 0/1)
//   uio oe   ch 8..15   core oe  bit (ch-8)/4 (mirrors the data position)
//   uo_out   ch 16..22  core out bit 2 + (ch >= 20)
// so one core can bit-bang I2C (two open-drain pins, distinct oe) and
// three-wire SPI (SCLK/MOSI on uio, CS_N on uo_out bit 2).
module hamming_lut_matrix (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  uio_in,
    input  wire [4:0]  ui_in,
    input  wire        ena,          // chip selected; low clamps outputs

    input  wire [7:0]  pin_filt_en,
    input  wire [255:0] lut_tt_flat,
    input  wire [255:0] lut_sel_flat,
    input  wire [15:0] lut_oreg,
    input  wire [95:0] pin_router_flat,
    input  wire [7:0]  open_drain_en,

    input  wire [7:0]  ev_flags,
    input  wire [15:0] replay_data,   // per-channel replay vector
    input  wire        replay_active,
    input  wire        host_cs_active,
    input  wire [15:0] core_out_flat,   // 4 cores x out[3:0]
    input  wire [15:0] core_oe_flat,    // 4 cores x oe[3:0]
    input  wire [3:0]  bcu_tx_p,
    input  wire [3:0]  bcu_tx_n,
    input  wire [3:0]  bcu_tx_oe,

    output wire [15:0] pin_inputs,      // conditioned tier-1 source bus
    output wire [15:0] lut_outputs,     // post-oreg LUT outputs (to cores)
    output reg  [7:0]  uio_out,
    output reg  [7:0]  uio_oe,
    output reg  [6:0]  uo_out
);

`ifdef FORMAL
  // Shared formal timebase.  The first clock is constrained to be a reset
  // clock; later reset activity is unconstrained so both reset and
  // deselection behavior are part of the proof.
  reg f_past_valid;
  initial f_past_valid = 1'b0;
`endif

  // ------------------------------------------------------------------------
  // Input synchronizers and per-uio-pin 4-tap glitch filters.  A filtered
  // output only changes when four consecutive samples agree, so spikes of
  // three cycles (~50 ns at 60 MHz) or shorter are rejected while pulses
  // reaching the 66.7 ns tap pass through.
  // ------------------------------------------------------------------------

  reg [12:0] sync1;
  reg [12:0] sync2;
  reg [2:0]  filt_hist [0:7];   // newest tap is sync2 itself
  reg [7:0]  filt_out;

  wire [12:0] pins_raw = {ui_in, uio_in};

  integer fi;
  always @(posedge clk) begin
    if (!rst_n) begin
      sync1 <= 13'h0000;
      sync2 <= 13'h0000;
      filt_out <= 8'h00;
      for (fi = 0; fi < 8; fi = fi + 1)
        filt_hist[fi] <= 3'h0;
    end else begin
      sync1 <= pins_raw;
      sync2 <= sync1;
      for (fi = 0; fi < 8; fi = fi + 1) begin
        filt_hist[fi] <= {filt_hist[fi][1:0], sync2[fi]};
        if ({filt_hist[fi], sync2[fi]} == 4'b0000)
          filt_out[fi] <= 1'b0;
        else if ({filt_hist[fi], sync2[fi]} == 4'b1111)
          filt_out[fi] <= 1'b1;
      end
    end
  end

  // Conditioned pin bus: uio pins use the filter output when enabled, the
  // synchronizer output otherwise; ui_in pins are synchronizer-only.
  wire [12:0] pins_cond;
  genvar gp;
  generate
    for (gp = 0; gp < 8; gp = gp + 1) begin : gen_pin_cond
      assign pins_cond[gp] = pin_filt_en[gp] ? filt_out[gp] : sync2[gp];
    end
    for (gp = 8; gp < 13; gp = gp + 1) begin : gen_pin_cond_ui
      assign pins_cond[gp] = sync2[gp];
    end
  endgenerate

  wire replay_gated = replay_data[0] & replay_active;

  // ------------------------------------------------------------------------
  // Tier source buses.
  // ------------------------------------------------------------------------

  wire [15:0] tier1_bus = {1'b0, host_cs_active, replay_gated, pins_cond};
  assign pin_inputs = tier1_bus;

  // ------------------------------------------------------------------------
  // LUT evaluation.  lut_sel packs four 4-bit input selectors per LUT as
  // {sel3, sel2, sel1, sel0} in bits [15:0] of each 16-bit lane; the truth
  // table index is {in3, in2, in1, in0}.
  // ------------------------------------------------------------------------

  wire [15:0] lut_comb;
  wire [15:0] tier2_bus = {1'b0, 1'b1, host_cs_active, replay_gated,
                           ev_flags[3:0], lut_outputs[7:0]};

  genvar gl;
  generate
    for (gl = 0; gl < 16; gl = gl + 1) begin : gen_lut
      wire [15:0] tt  = lut_tt_flat[gl*16 +: 16];
      wire [15:0] sel = lut_sel_flat[gl*16 +: 16];
      wire [15:0] bus = (gl < 8) ? tier1_bus : tier2_bus;
      wire        in0 = bus[sel[3:0]];
      wire        in1 = bus[sel[7:4]];
      wire        in2 = bus[sel[11:8]];
      wire        in3 = bus[sel[15:12]];
      assign lut_comb[gl] = tt[{in3, in2, in1, in0}];
      reg out_r;
      always @(posedge clk) begin
        if (!rst_n)
          out_r <= 1'b0;
        else
          out_r <= lut_comb[gl];
      end
      assign lut_outputs[gl] = lut_oreg[gl] ? out_r : lut_comb[gl];

`ifdef FORMAL
      // The optional output register is a true one-cycle sample of the LUT
      // combinational value.  When the register is not selected the leaf is
      // combinational; when selected it can only carry the registered sample.
      always @(posedge clk) begin
        if (f_past_valid && rst_n) begin
          if ($past(rst_n))
            assert (out_r == $past(lut_comb[gl]));
          if (!lut_oreg[gl])
            assert (lut_outputs[gl] == lut_comb[gl]);
          else
            assert (lut_outputs[gl] == out_r);
        end
      end
`endif
    end
  endgenerate

  // ------------------------------------------------------------------------
  // 23-channel 16:1 output pin router.  All outputs are single-stage
  // registered; the open-drain clamp forces the uio data channel low after
  // source selection so open-drain pins only ever pull down.
  // ------------------------------------------------------------------------

  // Per-pair 16-entry router source buses.  Every signal is referenced
  // directly in these continuous assignments so the mux evaluation tracks
  // all of them in simulation (a function reading globals can hide them
  // from the sensitivity).  Index map: 0 const0, 1 const1, 2 replay,
  // 3..10 LUT8..15, 11 paired core out[0], 12/13/14 paired BCU tx_p/n/oe,
  // 15 const0.
  wire [7:0] lut_hi = lut_outputs[15:8];
  wire [15:0] src_bus0 = {1'b0, bcu_tx_oe[0], bcu_tx_n[0], bcu_tx_p[0],
                          core_out_flat[0], lut_hi[7], lut_hi[6], lut_hi[5],
                          lut_hi[4], lut_hi[3], lut_hi[2], lut_hi[1], lut_hi[0],
                          replay_gated, 1'b1, 1'b0};
  wire [15:0] src_bus1 = {1'b0, bcu_tx_oe[1], bcu_tx_n[1], bcu_tx_p[1],
                          core_out_flat[4], lut_hi[7], lut_hi[6], lut_hi[5],
                          lut_hi[4], lut_hi[3], lut_hi[2], lut_hi[1], lut_hi[0],
                          replay_gated, 1'b1, 1'b0};
  wire [15:0] src_bus2 = {1'b0, bcu_tx_oe[2], bcu_tx_n[2], bcu_tx_p[2],
                          core_out_flat[8], lut_hi[7], lut_hi[6], lut_hi[5],
                          lut_hi[4], lut_hi[3], lut_hi[2], lut_hi[1], lut_hi[0],
                          replay_gated, 1'b1, 1'b0};
  wire [15:0] src_bus3 = {1'b0, bcu_tx_oe[3], bcu_tx_n[3], bcu_tx_p[3],
                          core_out_flat[12], lut_hi[7], lut_hi[6], lut_hi[5],
                          lut_hi[4], lut_hi[3], lut_hi[2], lut_hi[1], lut_hi[0],
                          replay_gated, 1'b1, 1'b0};

  integer ch;
  reg [3:0] ch_sel;
  reg [1:0] cbit;
  reg       use_oe;
  reg       core_sig;
  reg [15:0] ch_src;
  reg [22:0] ch_out;
  always @* begin
    for (ch = 0; ch < 23; ch = ch + 1) begin
      ch_sel = pin_router_flat[ch*4 +: 4];
      case (ch[1:0])
        2'd0:    ch_src = src_bus0;
        2'd1:    ch_src = src_bus1;
        2'd2:    ch_src = src_bus2;
        default: ch_src = src_bus3;
      endcase
      // Source 2: per-channel replay bit (index wraps mod 16).
      ch_src[2] = replay_data[ch[3:0]] & replay_active;
      // Source 11: channel-group-dependent core bit pairing.
      if (ch < 8) begin
        use_oe = 1'b0;
        cbit = {1'b0, ch[2]};
      end else if (ch < 16) begin
        use_oe = 1'b1;
        cbit = {1'b0, ch[2]};
      end else begin
        use_oe = 1'b0;
        cbit = 2'd2 + ((ch >= 20) ? 2'd1 : 2'd0);
      end
      if (use_oe)
        core_sig = core_oe_flat[ch[1:0]*4 + cbit];
      else
        core_sig = core_out_flat[ch[1:0]*4 + cbit];
      ch_src[11] = core_sig;
      ch_out[ch] = ch_src[ch_sel];
    end
  end

  integer ob;
  always @(posedge clk) begin
    if (!rst_n) begin
      uio_out <= 8'h00;
      uio_oe  <= 8'h00;
      uo_out  <= 7'h00;
    end else if (!ena) begin
      // R3: deselection releases every pin inside the single leaf stage.
      uio_out <= 8'h00;
      uio_oe  <= 8'h00;
      uo_out  <= 7'h00;
    end else begin
      for (ob = 0; ob < 8; ob = ob + 1) begin
        uio_out[ob] <= open_drain_en[ob] ? 1'b0 : ch_out[ob];
        uio_oe[ob]  <= ch_out[8 + ob];
      end
      for (ob = 0; ob < 7; ob = ob + 1)
        uo_out[ob] <= ch_out[16 + ob];
    end
  end

`ifdef FORMAL
  // One-cycle leaf-output witness.  ch_out and the configuration inputs are
  // captured at the same edge as the output leaf registers.  On the next
  // clock the visible outputs must exactly equal that sampled image.  This
  // simultaneously proves the open-drain clamp, the ena deselection clamp,
  // reset behavior, and safe replay exit (no replay value can leak beyond
  // the single registered leaf stage).
  reg [22:0] f_exp_ch_out;
  reg [7:0]  f_exp_open_drain;
  reg        f_exp_ena;
  reg        f_exp_rst_n;
  reg        f_exp_replay_active;
  reg        f_exp_valid;

  integer f_ob;
  initial f_exp_valid = 1'b0;

  always @(posedge clk) begin
    f_past_valid <= 1'b1;
    if (!f_past_valid)
      assume (!rst_n);

    f_exp_valid         <= 1'b1;
    f_exp_ch_out        <= ch_out;
    f_exp_open_drain    <= open_drain_en;
    f_exp_ena           <= ena;
    f_exp_rst_n         <= rst_n;
    f_exp_replay_active <= replay_active;

    if (f_exp_valid) begin
      if (!f_exp_rst_n || !f_exp_ena) begin
        assert (uio_out == 8'h00);
        assert (uio_oe  == 8'h00);
        assert (uo_out  == 7'h00);
      end else begin
        for (f_ob = 0; f_ob < 8; f_ob = f_ob + 1) begin
          assert (uio_out[f_ob] == (f_exp_open_drain[f_ob] ? 1'b0 : f_exp_ch_out[f_ob]));
          assert (uio_oe[f_ob]  == f_exp_ch_out[8 + f_ob]);
          if (f_exp_open_drain[f_ob])
            assert (uio_out[f_ob] == 1'b0);
        end
        for (f_ob = 0; f_ob < 7; f_ob = f_ob + 1)
          assert (uo_out[f_ob] == f_exp_ch_out[16 + f_ob]);
      end
    end

    if (f_past_valid) begin
      // Activity and safety covers: registered pin activity, a logical-one
      // router result clamped low by open-drain, replay deassertion after an
      // enabled cycle, disabled-pin release, and registered LUT activity.
      cover (f_exp_valid && f_exp_rst_n && f_exp_ena &&
             (uio_out != 8'h00) && (uio_oe != 8'h00) && (uo_out != 7'h00));
      cover (f_exp_valid && f_exp_rst_n && f_exp_ena &&
             f_exp_open_drain[0] && f_exp_ch_out[0] && !uio_out[0]);
      cover (f_exp_valid && f_exp_rst_n && f_exp_ena &&
             f_exp_replay_active && !replay_active);
      cover (f_exp_valid && f_exp_rst_n && !f_exp_ena &&
             (uio_out == 8'h00) && (uio_oe == 8'h00) && (uo_out == 7'h00));
      cover (rst_n && (lut_oreg != 16'h0000) && (lut_outputs != 16'h0000));
    end
  end
`endif

endmodule
