/*
 * Copyright (c) 2026 akorite
 * SPDX-License-Identifier: Apache-2.0
 */

`timescale 1ns/1ps
`default_nettype none

// tt_um_akorite_hamming - full-chip integration (U6).
//
// Four protocol cores with loop caches share SRAM 0 (firmware) through a
// 5-client round-robin arbiter with the Host SPI, and share SRAM 1
// (packet/RLE data) through a 6-client arbiter with the Host SPI and the
// RLE logic analyzer / waveform replay engine.  Each core pairs with one
// Bitstream Conditioning Unit; the 2-tier 16-LUT pin matrix conditions
// inputs, routes core/BCU/replay signals onto the pins, and clamps every
// output inside its single leaf register stage while the chip is
// deselected (KTD7 single-stage rule, R3 isolation).
//
// Pin map:
//   ui_in[7]   HOST_CS_N    ui_in[6] HOST_SCLK    ui_in[5] HOST_MOSI
//   ui_in[4:0] protocol inputs (into the LUT synchronizers)
//   uo_out[7]  HOST_MISO (directly from hamming_host_spi)
//   uo_out[6:0] router channels 16..22
//   uio[7:0]   bidirectional protocol pins
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

  // ==================================================================
  // Host SPI target
  // ==================================================================
  wire        spi_sram0_req, spi_sram0_we, spi_sram0_grant, spi_sram0_rvalid;
  wire [8:0]  spi_sram0_addr;
  wire [31:0] spi_sram0_wdata, spi_sram0_wmask, spi_sram0_rdata;
  wire        spi_sram1_req, spi_sram1_we, spi_sram1_grant, spi_sram1_rvalid;
  wire [8:0]  spi_sram1_addr;
  wire [31:0] spi_sram1_wdata, spi_sram1_wmask, spi_sram1_rdata;
  wire        spi_csr_we;
  wire [7:0]  spi_csr_addr;
  wire [15:0] spi_csr_wdata, spi_csr_rdata;
  wire        chip_ena, spi_active, host_miso;

  hamming_host_spi u_host_spi (
      .clk(clk), .rst_n(rst_n),
      .ena(ena),
      .host_cs_n(ui_in[7]),
      .host_sclk(ui_in[6]),
      .host_mosi(ui_in[5]),
      .host_miso(host_miso),
      .sram0_req(spi_sram0_req), .sram0_we(spi_sram0_we),
      .sram0_addr(spi_sram0_addr), .sram0_wdata(spi_sram0_wdata),
      .sram0_wmask(spi_sram0_wmask),
      .sram0_grant(spi_sram0_grant), .sram0_rvalid(spi_sram0_rvalid),
      .sram0_rdata(spi_sram0_rdata),
      .sram1_req(spi_sram1_req), .sram1_we(spi_sram1_we),
      .sram1_addr(spi_sram1_addr), .sram1_wdata(spi_sram1_wdata),
      .sram1_wmask(spi_sram1_wmask),
      .sram1_grant(spi_sram1_grant), .sram1_rvalid(spi_sram1_rvalid),
      .sram1_rdata(spi_sram1_rdata),
      .csr_we(spi_csr_we), .csr_addr(spi_csr_addr),
      .csr_wdata(spi_csr_wdata), .csr_rdata(spi_csr_rdata),
      .chip_ena(chip_ena), .spi_active(spi_active)
  );

  // MISO reaches the pin directly (KTD7); clamped low while deselected.
  assign uo_out[7] = host_miso & chip_ena;

  // ==================================================================
  // CSR bank
  // ==================================================================
  wire [3:0]  ctrl_run;
  wire [7:0]  ev_flags;
  wire [19:0] timer;
  wire [15:0] rle_base16, rle_mask16, rle_ctrl, rle_probe_mask;
  wire        rle_sticky_clr;
  wire [7:0]  pin_filt_en, open_drain_en;
  wire [95:0] pin_router_flat;
  wire [255:0] lut_tt_flat, lut_sel_flat;
  wire [15:0] lut_oreg;
  wire [63:0] bcu_mode_flat, bcu_bit_len_flat, bcu_pat_match_flat;
  wire [63:0] bcu_pat_len_flat, bcu_dpll_cfg_flat, bcu_tx_data_flat;
  wire [127:0] bcu_crc_poly_flat, bcu_crc_state_cfg_flat;
  wire [3:0]  crc_lo_we, crc_hi_we;
  wire [63:0] crc_seed_flat;

  // per-core CSR access wiring
  wire [3:0]  core_csr_we;
  wire [31:0] core_csr_waddr_flat;
  wire [63:0] core_csr_wdata_flat, core_csr_rdata_flat;
  wire [31:0] core_csr_raddr_flat;
  wire [63:0] core_status_flat;
  wire [39:0] core_pc_flat;
  wire [15:0] core_out_flat, core_oe_flat;
  wire [63:0] bcu_rx_data_flat, bcu_stat_flat;
  wire [127:0] bcu_crc_live_flat;
  wire [8:0]  rle_wr_ptr9;
  wire        rle_overflow, rle_underflow, rle_capturing;

  hamming_csr u_csr (
      .clk(clk), .rst_n(rst_n),
      .host_we(spi_csr_we), .host_addr(spi_csr_addr),
      .host_wdata(spi_csr_wdata), .host_rdata(spi_csr_rdata),
      .core_we(core_csr_we),
      .core_waddr_flat(core_csr_waddr_flat),
      .core_wdata_flat(core_csr_wdata_flat),
      .core_raddr_flat(core_csr_raddr_flat),
      .core_rdata_flat(core_csr_rdata_flat),
      .core_status_flat(core_status_flat),
      .core_pc_flat(core_pc_flat),
      .core_out_flat(core_out_flat),
      .core_oe_flat(core_oe_flat),
      .bcu_rx_data_flat(bcu_rx_data_flat),
      .bcu_stat_flat(bcu_stat_flat),
      .rle_wr_ptr({7'h00, rle_wr_ptr9}),
      .rle_status({13'h0, rle_capturing, rle_underflow, rle_overflow}),
      .ctrl_run(ctrl_run),
      .ev_flags(ev_flags),
      .timer(timer),
      .rle_base(rle_base16), .rle_mask(rle_mask16),
      .pin_filt_en(pin_filt_en), .open_drain_en(open_drain_en),
      .pin_router_flat(pin_router_flat),
      .lut_tt_flat(lut_tt_flat), .lut_sel_flat(lut_sel_flat),
      .lut_oreg(lut_oreg),
      .bcu_mode_flat(bcu_mode_flat),
      .bcu_crc_poly_flat(bcu_crc_poly_flat),
      .bcu_crc_state_flat(bcu_crc_state_cfg_flat),
      .bcu_bit_len_flat(bcu_bit_len_flat),
      .bcu_pat_match_flat(bcu_pat_match_flat),
      .bcu_pat_len_flat(bcu_pat_len_flat),
      .bcu_dpll_cfg_flat(bcu_dpll_cfg_flat),
      .bcu_tx_data_flat(bcu_tx_data_flat),
      .bcu_crc_live_flat(bcu_crc_live_flat),
      .crc_lo_we(crc_lo_we), .crc_hi_we(crc_hi_we),
      .crc_seed_flat(crc_seed_flat),
      .rle_ctrl(rle_ctrl), .rle_probe_mask(rle_probe_mask),
      .rle_sticky_clr(rle_sticky_clr)
  );

  // ==================================================================
  // LUT pin matrix (input conditioning, 2-tier logic, output router)
  // ==================================================================
  wire [15:0] pin_inputs, lut_outputs;
  wire [3:0]  bcu_tx_p, bcu_tx_n, bcu_tx_oe;
  wire [15:0] rle_replay_data;
  wire        rle_replay_active;

  hamming_lut_matrix u_lut (
      .clk(clk), .rst_n(rst_n),
      .uio_in(uio_in),
      .ui_in(ui_in[4:0]),
      .ena(chip_ena),
      .pin_filt_en(pin_filt_en),
      .lut_tt_flat(lut_tt_flat),
      .lut_sel_flat(lut_sel_flat),
      .lut_oreg(lut_oreg),
      .pin_router_flat(pin_router_flat),
      .open_drain_en(open_drain_en),
      .ev_flags(ev_flags),
      .replay_data(rle_replay_data),
      .replay_active(rle_replay_active),
      .host_cs_active(spi_active),
      .core_out_flat(core_out_flat),
      .core_oe_flat(core_oe_flat),
      .bcu_tx_p(bcu_tx_p), .bcu_tx_n(bcu_tx_n), .bcu_tx_oe(bcu_tx_oe),
      .pin_inputs(pin_inputs),
      .lut_outputs(lut_outputs),
      .uio_out(uio_out),
      .uio_oe(uio_oe),
      .uo_out(uo_out[6:0])
  );

  // ==================================================================
  // SRAM arbiters and macros
  //
  // SRAM 0 clients: 0..3 core instruction fetch, 4 host SPI.
  // SRAM 1 clients: 0..3 core data memory, 4 host SPI, 5 RLE engine.
  // ==================================================================
  wire [3:0]  core_imem_req;
  wire [35:0] core_imem_addr_flat;
  wire [3:0]  core_imem_grant, core_imem_rvalid;
  wire [31:0] sram0_rdata;
  wire        sram0_write_commit;

  wire [4:0]  arb0_req, arb0_we, arb0_grant, arb0_rvalid;
  wire [44:0] arb0_addr_flat;
  wire [159:0] arb0_wdata_flat, arb0_wmask_flat;

  assign arb0_req = {spi_sram0_req, core_imem_req};
  assign arb0_we  = {spi_sram0_we, 4'b0000};
  assign arb0_addr_flat = {spi_sram0_addr, core_imem_addr_flat};
  assign arb0_wdata_flat = {spi_sram0_wdata, 128'h0};
  assign arb0_wmask_flat = {spi_sram0_wmask, 128'h0};
  assign spi_sram0_grant  = arb0_grant[4];
  assign spi_sram0_rvalid = arb0_rvalid[4];
  assign spi_sram0_rdata  = sram0_rdata;
  assign core_imem_grant  = arb0_grant[3:0];
  assign core_imem_rvalid = arb0_rvalid[3:0];

  hamming_sram_arb #(.NCLIENTS(5)) u_arb0 (
      .clk(clk), .rst_n(rst_n),
      .req(arb0_req), .we(arb0_we),
      .addr_flat(arb0_addr_flat),
      .wdata_flat(arb0_wdata_flat),
      .wmask_flat(arb0_wmask_flat),
      .grant(arb0_grant), .rvalid(arb0_rvalid),
      .rdata(sram0_rdata),
      .write_commit(sram0_write_commit)
  );

  wire [3:0]  core_dmem_req, core_dmem_we, core_dmem_grant, core_dmem_rvalid;
  wire [35:0] core_dmem_addr_flat;
  wire [127:0] core_dmem_wdata_flat, core_dmem_wmask_flat;
  wire        rle_mem_req, rle_mem_we, rle_mem_grant, rle_mem_rvalid;
  wire [8:0]  rle_mem_addr;
  wire [31:0] rle_mem_wdata, sram1_rdata;

  wire [5:0]  arb1_req, arb1_we, arb1_grant, arb1_rvalid;
  wire [53:0] arb1_addr_flat;
  wire [191:0] arb1_wdata_flat, arb1_wmask_flat;

  assign arb1_req = {rle_mem_req, spi_sram1_req, core_dmem_req};
  assign arb1_we  = {rle_mem_we,  spi_sram1_we,  core_dmem_we};
  assign arb1_addr_flat  = {rle_mem_addr,  spi_sram1_addr,  core_dmem_addr_flat};
  assign arb1_wdata_flat = {rle_mem_wdata, spi_sram1_wdata, core_dmem_wdata_flat};
  assign arb1_wmask_flat = {32'hFFFFFFFF,  spi_sram1_wmask, core_dmem_wmask_flat};
  assign spi_sram1_grant  = arb1_grant[4];
  assign spi_sram1_rvalid = arb1_rvalid[4];
  assign spi_sram1_rdata  = sram1_rdata;
  assign core_dmem_grant  = arb1_grant[3:0];
  assign core_dmem_rvalid = arb1_rvalid[3:0];
  assign rle_mem_grant    = arb1_grant[5];
  assign rle_mem_rvalid   = arb1_rvalid[5];
  // Both arbiters share a single read-data bus per macro; each client
  // samples it when its own rvalid pulses.
  wire [31:0] rle_mem_rdata = sram1_rdata;

  hamming_sram_arb #(.NCLIENTS(6)) u_arb1 (
      .clk(clk), .rst_n(rst_n),
      .req(arb1_req), .we(arb1_we),
      .addr_flat(arb1_addr_flat),
      .wdata_flat(arb1_wdata_flat),
      .wmask_flat(arb1_wmask_flat),
      .grant(arb1_grant), .rvalid(arb1_rvalid),
      .rdata(sram1_rdata),
      .write_commit()
  );

  // ==================================================================
  // RLE logic analyzer / waveform replay engine
  // ==================================================================
  // Probe vector: the conditioned 13-pin bus plus the tier-1 control bits
  // (pin_inputs layout), giving capture uniform synchronization for free.
  wire [2:0] rle_trig_sel = rle_ctrl[6:4];
  wire       rle_trig_in  = lut_outputs[8 + rle_trig_sel];
  wire [1:0] rle_mode = chip_ena ? rle_ctrl[1:0] : 2'b00;

  hamming_rle_engine u_rle (
      .clk(clk), .rst_n(rst_n),
      .probe(pin_inputs),
      .probe_mask(rle_probe_mask),
      .mode(rle_mode),
      .trig_start_en(rle_ctrl[2]),
      .trig_stop_en(rle_ctrl[3]),
      .trig_in(rle_trig_in),
      .sticky_clear(rle_sticky_clr),
      .rle_base(rle_base16[8:0]),
      .rle_mask(rle_mask16[8:0]),
      .mem_req(rle_mem_req), .mem_we(rle_mem_we),
      .mem_addr(rle_mem_addr), .mem_wdata(rle_mem_wdata),
      .mem_grant(rle_mem_grant), .mem_rvalid(rle_mem_rvalid),
      .mem_rdata(rle_mem_rdata),
      .replay_data(rle_replay_data),
      .replay_active(rle_replay_active),
      .overflow(rle_overflow),
      .underflow(rle_underflow),
      .capturing(rle_capturing),
      .wr_ptr(rle_wr_ptr9)
  );

  // ==================================================================
  // Four protocol cores + BCUs
  // ==================================================================
  genvar gc;
  generate
    for (gc = 0; gc < 4; gc = gc + 1) begin : gen_core
      wire [7:0]  core_bcu_ctrl;
      wire        bcu_tx_push_w, bcu_rx_pop_w;
      wire [15:0] bcu_tx_data_w;

      hamming_core u_core (
          .clk(clk), .rst_n(rst_n),
          .run(ctrl_run[gc] & chip_ena),
          .timer(timer),
          .ev_flags(ev_flags),
          .pin_inputs(pin_inputs),
          .lut_outputs(lut_outputs),
          .core_out(core_out_flat[gc*4 +: 4]),
          .core_oe(core_oe_flat[gc*4 +: 4]),
          .core_status(core_status_flat[gc*16 +: 16]),
          .core_pc_out(core_pc_flat[gc*10 +: 10]),
          .dbg_regs_flat(),
          .csr_we(core_csr_we[gc]),
          .csr_addr(core_csr_waddr_flat[gc*8 +: 8]),
          .csr_wdata(core_csr_wdata_flat[gc*16 +: 16]),
          .csr_rdata(core_csr_rdata_flat[gc*16 +: 16]),
          .bcu_tx_push(bcu_tx_push_w),
          .bcu_rx_pop(bcu_rx_pop_w),
          .bcu_tx_data(bcu_tx_data_w),
          .bcu_rx_data(bcu_rx_data_flat[gc*16 +: 16]),
          .bcu_stat(bcu_stat_flat[gc*16 +: 16]),
          .bcu_crc_state(bcu_crc_live_flat[gc*32 +: 32]),
          .bcu_ctrl(core_bcu_ctrl),
          .icache_inv_all(sram0_write_commit),
          .imem_req(core_imem_req[gc]),
          .imem_addr(core_imem_addr_flat[gc*9 +: 9]),
          .imem_grant(core_imem_grant[gc]),
          .imem_rvalid(core_imem_rvalid[gc]),
          .imem_rdata(sram0_rdata),
          .dmem_req(core_dmem_req[gc]),
          .dmem_we(core_dmem_we[gc]),
          .dmem_addr(core_dmem_addr_flat[gc*9 +: 9]),
          .dmem_wdata(core_dmem_wdata_flat[gc*32 +: 32]),
          .dmem_wmask(core_dmem_wmask_flat[gc*32 +: 32]),
          .dmem_grant(core_dmem_grant[gc]),
          .dmem_rvalid(core_dmem_rvalid[gc]),
          .dmem_rdata(sram1_rdata)
      );

      // RX inputs come through tier-1 LUTs so any pin can reach any BCU:
      // rx_p = lut_outputs[2k], rx_n/ext_sclk share lut_outputs[2k+1]
      // (differential RX and external-clock mode are mutually exclusive).
      hamming_bcu u_bcu (
          .clk(clk), .rst_n(rst_n),
          .tx_push(bcu_tx_push_w),
          .tx_data(bcu_tx_data_w),
          .rx_pop(bcu_rx_pop_w),
          .rx_data(bcu_rx_data_flat[gc*16 +: 16]),
          .stat(bcu_stat_flat[gc*16 +: 16]),
          .crc_state(bcu_crc_live_flat[gc*32 +: 32]),
          .ctrl(core_bcu_ctrl),
          .cfg_mode(bcu_mode_flat[gc*16 +: 16]),
          .cfg_crc_poly(bcu_crc_poly_flat[gc*32 +: 32]),
          .cfg_crc_state(bcu_crc_state_cfg_flat[gc*32 +: 32]),
          .crc_seed_we(1'b0),
          .crc_lo_we(crc_lo_we[gc]),
          .crc_hi_we(crc_hi_we[gc]),
          .crc_seed16(crc_seed_flat[gc*16 +: 16]),
          .cfg_bit_len(bcu_bit_len_flat[gc*16 +: 16]),
          .cfg_pat_match(bcu_pat_match_flat[gc*16 +: 16]),
          .cfg_pat_len(bcu_pat_len_flat[gc*16 +: 16]),
          .cfg_dpll(bcu_dpll_cfg_flat[gc*16 +: 16]),
          .rx_p(lut_outputs[gc*2]),
          .rx_n(lut_outputs[gc*2 + 1]),
          .tx_p(bcu_tx_p[gc]),
          .tx_n(bcu_tx_n[gc]),
          .tx_oe(bcu_tx_oe[gc]),
          .ext_sclk(lut_outputs[gc*2 + 1])
      );
    end
  endgenerate

  // Core CSR read addresses: the core presents one read port that the CSR
  // bank decodes combinationally.
  // (wired inside gen_core via core_csr_raddr_flat below)
  assign core_csr_raddr_flat = core_csr_waddr_flat;

endmodule
