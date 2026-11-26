/*
 * Copyright (c) 2026 akorite
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none
`include "hamming_defs.vh"

// Uniform 16-bit CSR bank for Hamming. Host writes are synchronous; reads are
// combinational so both Host SPI prefetch and zero-stall core CSR access use
// the same decode. Core write ownership is restricted to that core's private
// bank plus the atomic event set/clear registers. Atomic event updates are
// bitwise; when set and clear arrive together, set wins so an event is not
// silently lost.
module hamming_csr (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        host_we,
    input  wire [7:0]  host_addr,
    input  wire [15:0] host_wdata,
    output reg  [15:0] host_rdata,

    input  wire [3:0]  core_we,
    input  wire [31:0] core_waddr_flat,  // 4 x 8-bit write addresses
    input  wire [63:0] core_wdata_flat,  // 4 x 16-bit write data
    input  wire [31:0] core_raddr_flat,  // 4 x 8-bit read addresses
    output wire [63:0] core_rdata_flat,  // 4 x 16-bit read data

    input  wire [63:0] core_status_flat,
    input  wire [39:0] core_pc_flat,
    input  wire [15:0] core_out_flat,
    input  wire [15:0] core_oe_flat,
    input  wire [63:0] bcu_rx_data_flat,
    input  wire [63:0] bcu_stat_flat,
    input  wire [15:0] rle_wr_ptr,
    input  wire [15:0] rle_status,

    output wire [3:0]  ctrl_run,
    output wire [7:0]  ev_flags,
    output wire [19:0] timer,
    output wire [15:0] rle_base,
    output wire [15:0] rle_mask,
    output wire [7:0]  pin_filt_en,
    output wire [7:0]  open_drain_en,
    output wire [95:0]  pin_router_flat,
    output wire [255:0] lut_tt_flat,
    output wire [255:0] lut_sel_flat,
    output wire [15:0] lut_oreg,
    output wire [63:0]  bcu_mode_flat,
    output wire [127:0] bcu_crc_poly_flat,
    output wire [127:0] bcu_crc_state_flat,
    output wire [63:0]  bcu_bit_len_flat,
    output wire [63:0]  bcu_pat_match_flat,
    output wire [63:0]  bcu_pat_len_flat,
    output wire [63:0]  bcu_dpll_cfg_flat,
    output wire [63:0]  bcu_tx_data_flat,
    input  wire [127:0] bcu_crc_live_flat,
    output reg  [3:0]  crc_lo_we,
    output reg  [3:0]  crc_hi_we,
    output reg  [63:0] crc_seed_flat,
    output wire [15:0] rle_ctrl,
    output wire [15:0] rle_probe_mask,
    output reg         rle_sticky_clr
);

  localparam CSR_CTRL_RUN      = `HAM_CSR_CTRL_RUN;
  localparam CSR_EV_FLAGS      = `HAM_CSR_EV_FLAGS;
  localparam CSR_EV_SET        = `HAM_CSR_EV_SET;
  localparam CSR_EV_CLR        = `HAM_CSR_EV_CLR;
  localparam CSR_TIMER_LO      = `HAM_CSR_TIMER_LO;
  localparam CSR_TIMER_HI      = `HAM_CSR_TIMER_HI;
  localparam CSR_RLE_BASE      = `HAM_CSR_RLE_BASE;
  localparam CSR_RLE_CTRL      = `HAM_CSR_RLE_CTRL;
  localparam CSR_RLE_PROBE_MASK = `HAM_CSR_RLE_PROBE_MASK;
  localparam CSR_RLE_MASK      = `HAM_CSR_RLE_MASK;
  localparam CSR_RLE_WR_PTR    = `HAM_CSR_RLE_WR_PTR;
  localparam CSR_RLE_STATUS    = `HAM_CSR_RLE_STATUS;
  localparam CSR_PIN_FILT_EN   = `HAM_CSR_PIN_FILT_EN;
  localparam CSR_OPEN_DRAIN_EN = `HAM_CSR_OPEN_DRAIN_EN;
  localparam CSR_PIN_ROUTER0   = `HAM_CSR_PIN_ROUTER0;
  localparam CSR_CORE_BANK     = `HAM_CSR_CORE_BANK;
  localparam CSR_LUT_TT_BASE   = `HAM_CSR_LUT_TT_BASE;
  localparam CSR_LUT_SEL_BASE  = `HAM_CSR_LUT_SEL_BASE;
  localparam CSR_LUT_OREG      = `HAM_CSR_LUT_OREG;

  reg [3:0]  ctrl_run_r;
  reg [7:0]  ev_flags_r;
  reg [19:0] timer_r;
  reg [6:0]  rle_ctrl_r;
  reg [15:0] rle_probe_mask_r;
  reg [8:0]  rle_base_r;
  reg [8:0]  rle_mask_r;
  reg [7:0]  pin_filt_en_r;
  reg [7:0]  open_drain_en_r;
  reg [15:0] pin_router_r [0:5];
  reg [15:0] lut_tt_r  [0:15];
  reg [15:0] lut_sel_r [0:15];
  reg [15:0] lut_oreg_r;

  reg [15:0] bcu_mode_r      [0:3];
  reg [31:0] bcu_crc_poly_r  [0:3];
  reg [4:0]  bcu_bit_len_r  [0:3];
  reg [15:0] bcu_pat_match_r [0:3];
  reg [3:0]  bcu_pat_len_r  [0:3];
  reg [13:0] bcu_dpll_cfg_r  [0:3];

  assign ctrl_run     = ctrl_run_r;
  assign ev_flags     = ev_flags_r;
  assign timer        = timer_r;
  assign rle_base     = {7'b0, rle_base_r};
  assign rle_mask     = {7'b0, rle_mask_r};
  assign pin_filt_en  = pin_filt_en_r;
  assign open_drain_en = open_drain_en_r;
  assign lut_oreg     = lut_oreg_r;

  genvar go;
  generate
    for (go = 0; go < 6; go = go + 1) begin : gen_pin_router_out
      assign pin_router_flat[go*16 +: 16] = pin_router_r[go];
    end
    for (go = 0; go < 16; go = go + 1) begin : gen_lut_out
      assign lut_tt_flat[go*16 +: 16]  = lut_tt_r[go];
      assign lut_sel_flat[go*16 +: 16] = lut_sel_r[go];
    end
    for (go = 0; go < 4; go = go + 1) begin : gen_core_cfg_out
      assign bcu_mode_flat[go*16 +: 16]       = bcu_mode_r[go];
      assign bcu_crc_poly_flat[go*32 +: 32]   = bcu_crc_poly_r[go];
      assign bcu_crc_state_flat[go*32 +: 32] = bcu_crc_live_flat[go*32 +: 32];
      assign bcu_bit_len_flat[go*16 +: 16]    = {11'b0, bcu_bit_len_r[go]};
      assign bcu_pat_match_flat[go*16 +: 16]  = bcu_pat_match_r[go];
      assign bcu_pat_len_flat[go*16 +: 16]    = {12'b0, bcu_pat_len_r[go]};
      assign bcu_dpll_cfg_flat[go*16 +: 16]   = {bcu_dpll_cfg_r[go][13:2], 2'b00, bcu_dpll_cfg_r[go][1:0]};
      assign bcu_tx_data_flat[go*16 +: 16] = 16'h0000;
    end

    // Core read ports are deliberately narrower than the host port.  Core k
    // sees its own private bank, the shared event flags, and the shared timer;
    // foreign-bank data, host configuration, RLE/pin-router state, and the LUT
    // programming page read as zero.  Offset CORE_ID is the one cross-bank
    // exception: reading offset D in any per-core bank returns the reading
    // core's own index, which is the architectural role-dispatch mechanism.
    // Because the bank number is the generate constant go, all private-bank
    // references below elaborate to direct wires rather than cross-bank muxes.
    for (go = 0; go < 4; go = go + 1) begin : gen_core_read_decode
      localparam [1:0] CORE_ID_BITS = (go == 0) ? 2'd0 :
                                      (go == 1) ? 2'd1 :
                                      (go == 2) ? 2'd2 : 2'd3;
      localparam [3:0] OWN_BANK_NIBBLE = {2'b01, CORE_ID_BITS};

      wire [7:0]  core_read_addr;
      wire        core_bank_page;
      wire        core_own_bank;
      reg  [15:0] core_read_value;

      assign core_read_addr = core_raddr_flat[go*8 +: 8];
      assign core_bank_page = (core_read_addr[7:6] == 2'b01);
      assign core_own_bank  = (core_read_addr[7:4] == OWN_BANK_NIBBLE);

      always @* begin
        core_read_value = 16'h0000;
        if (core_bank_page) begin
          if (core_read_addr[3:0] == 4'hD) begin
            core_read_value = {14'h000, CORE_ID_BITS};
          end else if (core_own_bank) begin
            case (core_read_addr[3:0])
              4'h0: core_read_value = core_status_flat[go*16 +: 16];
              4'h1: core_read_value = {6'b000000, core_pc_flat[go*10 +: 10]};
              4'h2: core_read_value = {12'b0, core_out_flat[go*4 +: 4]};
              4'h3: core_read_value = {12'b0, core_oe_flat[go*4 +: 4]};
              4'h4: core_read_value = bcu_mode_r[go];
              4'h5: core_read_value = bcu_crc_poly_r[go][15:0];
              4'h6: core_read_value = bcu_crc_poly_r[go][31:16];
              4'h7: core_read_value = bcu_crc_live_flat[go*32 +: 16];
              4'h8: core_read_value = bcu_crc_live_flat[go*32+16 +: 16];
              4'h9: core_read_value = {11'b0, bcu_bit_len_r[go]};
              4'hA: core_read_value = bcu_pat_match_r[go];
              4'hB: core_read_value = {12'b0, bcu_pat_len_r[go]};
              4'hC: core_read_value = {bcu_dpll_cfg_r[go][13:2], 2'b00, bcu_dpll_cfg_r[go][1:0]};
              4'hE: core_read_value = bcu_rx_data_flat[go*16 +: 16];
              4'hF: core_read_value = bcu_stat_flat[go*16 +: 16];
              default: core_read_value = 16'h0000;
            endcase
          end
        end else begin
          case (core_read_addr)
            CSR_EV_FLAGS:      core_read_value = {8'b0, ev_flags_r};
            CSR_TIMER_LO:      core_read_value = timer_r[15:0];
            CSR_TIMER_HI:      core_read_value = {12'b0, timer_r[19:16]};
            default:           core_read_value = 16'h0000;
          endcase
        end
      end

      assign core_rdata_flat[go*16 +: 16] = core_read_value;
    end
  endgenerate

  integer host_read_bank;

  // The host retains the full cross-bank debug/readback view.  Keeping this
  // decode as a single port is intentional: the four restricted core ports
  // above no longer replicate these variable bank and LUT index muxes.
  always @* begin
    host_rdata = 16'h0000;
    host_read_bank = 0;
    if (host_addr >= CSR_CORE_BANK && host_addr < 8'h80) begin
      host_read_bank = (host_addr - CSR_CORE_BANK) >> 4;
      case (host_addr[3:0])
        4'h0: host_rdata = core_status_flat[host_read_bank*16 +: 16];
        4'h1: host_rdata = {6'b000000, core_pc_flat[host_read_bank*10 +: 10]};
        4'h2: host_rdata = {12'b0, core_out_flat[host_read_bank*4 +: 4]};
        4'h3: host_rdata = {12'b0, core_oe_flat[host_read_bank*4 +: 4]};
        4'h4: host_rdata = bcu_mode_r[host_read_bank];
        4'h5: host_rdata = bcu_crc_poly_r[host_read_bank][15:0];
        4'h6: host_rdata = bcu_crc_poly_r[host_read_bank][31:16];
        4'h7: host_rdata = bcu_crc_live_flat[host_read_bank*32 +: 16];
        4'h8: host_rdata = bcu_crc_live_flat[host_read_bank*32+16 +: 16];
        4'h9: host_rdata = {11'b0, bcu_bit_len_r[host_read_bank]};
        4'hA: host_rdata = bcu_pat_match_r[host_read_bank];
        4'hB: host_rdata = {12'b0, bcu_pat_len_r[host_read_bank]};
        4'hC: host_rdata = {bcu_dpll_cfg_r[host_read_bank][13:2], 2'b00, bcu_dpll_cfg_r[host_read_bank][1:0]};
        4'hD: host_rdata = {14'h000, host_read_bank[1:0]};
        4'hE: host_rdata = bcu_rx_data_flat[host_read_bank*16 +: 16];
        4'hF: host_rdata = bcu_stat_flat[host_read_bank*16 +: 16];
        default: host_rdata = 16'h0000;
      endcase
    end else if (host_addr >= CSR_LUT_TT_BASE && host_addr < CSR_LUT_SEL_BASE) begin
      host_rdata = lut_tt_r[host_addr - CSR_LUT_TT_BASE];
    end else if (host_addr >= CSR_LUT_SEL_BASE && host_addr < CSR_LUT_OREG) begin
      host_rdata = lut_sel_r[host_addr - CSR_LUT_SEL_BASE];
    end else begin
      case (host_addr)
        CSR_CTRL_RUN:       host_rdata = {12'b0, ctrl_run_r};
        CSR_EV_FLAGS:       host_rdata = {8'b0, ev_flags_r};
        CSR_TIMER_LO:       host_rdata = timer_r[15:0];
        CSR_TIMER_HI:       host_rdata = {12'b0, timer_r[19:16]};
        CSR_RLE_BASE:       host_rdata = {7'b0, rle_base_r};
        CSR_RLE_CTRL:       host_rdata = {9'b0, rle_ctrl_r};
        CSR_RLE_PROBE_MASK: host_rdata = rle_probe_mask_r;
        CSR_RLE_MASK:       host_rdata = {7'b0, rle_mask_r};
        CSR_RLE_WR_PTR:     host_rdata = rle_wr_ptr;
        CSR_RLE_STATUS:     host_rdata = rle_status;
        CSR_PIN_FILT_EN:    host_rdata = {8'b0, pin_filt_en_r};
        CSR_OPEN_DRAIN_EN:  host_rdata = {8'b0, open_drain_en_r};
        CSR_PIN_ROUTER0:    host_rdata = pin_router_r[0];
        CSR_PIN_ROUTER0+1:  host_rdata = pin_router_r[1];
        CSR_PIN_ROUTER0+2:  host_rdata = pin_router_r[2];
        CSR_PIN_ROUTER0+3:  host_rdata = pin_router_r[3];
        CSR_PIN_ROUTER0+4:  host_rdata = pin_router_r[4];
        CSR_PIN_ROUTER0+5:  host_rdata = pin_router_r[5];
        CSR_LUT_OREG:       host_rdata = lut_oreg_r;
        default:            host_rdata = 16'h0000;
      endcase
    end
  end

  function private_writable;
    input [3:0] off;
    begin
      // Offsets 0..3 are core-owned status, E is BCU RX data, and F is the
      // BCU status snapshot. Offsets 4..D are configuration/TX state.
      private_writable = (off >= 4'h4 && off <= 4'hD);
    end
  endfunction

  task write_private;
    input integer bank;
    input [3:0] off;
    input [15:0] value;
    begin
      case (off)
        4'h4: bcu_mode_r[bank] <= value;
        4'h5: bcu_crc_poly_r[bank][15:0] <= value;
        4'h6: bcu_crc_poly_r[bank][31:16] <= value;
        4'h9: bcu_bit_len_r[bank] <= value[4:0];
        4'hA: bcu_pat_match_r[bank] <= value;
        4'hB: bcu_pat_len_r[bank] <= value[3:0];
        4'hC: bcu_dpll_cfg_r[bank] <= {value[15:4], value[1:0]};
        default: begin end
      endcase
    end
  endtask

  reg [7:0] ev_set_req;
  reg [7:0] ev_clr_req;
  integer ev_core;
  reg [7:0] ev_core_addr;
  reg [15:0] ev_core_data;

  always @* begin
    ev_set_req = 8'h00;
    ev_clr_req = 8'h00;
    if (host_we) begin
      if (host_addr == CSR_EV_SET)
        ev_set_req = ev_set_req | host_wdata[7:0];
      if (host_addr == CSR_EV_CLR)
        ev_clr_req = ev_clr_req | host_wdata[7:0];
    end
    for (ev_core = 0; ev_core < 4; ev_core = ev_core + 1) begin
      ev_core_addr = core_waddr_flat[ev_core*8 +: 8];
      ev_core_data = core_wdata_flat[ev_core*16 +: 16];
      if (core_we[ev_core] && ev_core_addr == CSR_EV_SET)
        ev_set_req = ev_set_req | ev_core_data[7:0];
      if (core_we[ev_core] && ev_core_addr == CSR_EV_CLR)
        ev_clr_req = ev_clr_req | ev_core_data[7:0];
    end
  end

  integer ri;
  integer wc;
  reg [7:0] waddr;
  reg [15:0] wdata;
  integer wbank;

  always @(posedge clk) begin
    if (!rst_n) begin
      ctrl_run_r <= 4'h0;
      ev_flags_r <= 8'h00;
      timer_r <= 20'd0;
      rle_ctrl_r <= 16'h0000;
      rle_probe_mask_r <= 16'h0000;
      rle_base_r <= 16'h0000;
      rle_mask_r <= 16'h0000;
      pin_filt_en_r <= 8'h00;
      open_drain_en_r <= 8'h00;
      lut_oreg_r <= 16'h0000;
      for (ri = 0; ri < 6; ri = ri + 1)
        pin_router_r[ri] <= 16'h0000;
      for (ri = 0; ri < 16; ri = ri + 1) begin
        lut_tt_r[ri] <= 16'h0000;
        lut_sel_r[ri] <= 16'h0000;
      end
      for (ri = 0; ri < 4; ri = ri + 1) begin
        bcu_mode_r[ri] <= 16'h0000;
        bcu_crc_poly_r[ri] <= 32'h00000000;
        bcu_bit_len_r[ri] <= 16'h0000;
        bcu_pat_match_r[ri] <= 16'h0000;
        bcu_pat_len_r[ri] <= 16'h0000;
        bcu_dpll_cfg_r[ri] <= 16'h0000;
      end
    end else begin
      timer_r <= timer_r + 1'b1;

      // Atomic event update. Clear is applied first and set second, so a
      // simultaneous set wins bitwise and cannot be lost.
      ev_flags_r <= (ev_flags_r & ~ev_clr_req) | ev_set_req;

      // Core writes. A core may write only its own private writable offsets;
      // event set/clear writes are handled above and all other global writes
      // are dropped. Distinct cores can update distinct banks in one cycle.
      for (wc = 0; wc < 4; wc = wc + 1) begin
        if (core_we[wc]) begin
          waddr = core_waddr_flat[wc*8 +: 8];
          wdata = core_wdata_flat[wc*16 +: 16];
          if (waddr >= CSR_CORE_BANK + (wc * 16) &&
              waddr <  CSR_CORE_BANK + (wc * 16) + 16 &&
              private_writable(waddr[3:0]))
            write_private(wc, waddr[3:0], wdata);
        end
      end

      // Host writes are decoded after core writes so host control wins any
      // same-cycle configuration conflict.
      if (host_we) begin
        if (host_addr >= CSR_CORE_BANK && host_addr < 8'h80) begin
          wbank = (host_addr - CSR_CORE_BANK) >> 4;
          if (private_writable(host_addr[3:0]))
            write_private(wbank, host_addr[3:0], host_wdata);
        end else if (host_addr >= CSR_LUT_TT_BASE && host_addr < CSR_LUT_SEL_BASE) begin
          lut_tt_r[host_addr - CSR_LUT_TT_BASE] <= host_wdata;
        end else if (host_addr >= CSR_LUT_SEL_BASE && host_addr < CSR_LUT_OREG) begin
          lut_sel_r[host_addr - CSR_LUT_SEL_BASE] <= host_wdata;
        end else begin
          case (host_addr)
            CSR_CTRL_RUN:      ctrl_run_r <= host_wdata[3:0];
            CSR_EV_FLAGS:      ev_flags_r <= host_wdata[7:0];
            CSR_RLE_BASE:      rle_base_r <= host_wdata[8:0];
            CSR_RLE_MASK:      rle_mask_r <= host_wdata[8:0];
            CSR_RLE_CTRL:      rle_ctrl_r <= host_wdata[6:0];
            CSR_RLE_PROBE_MASK: rle_probe_mask_r <= host_wdata;
            CSR_PIN_FILT_EN:   pin_filt_en_r <= host_wdata[7:0];
            CSR_OPEN_DRAIN_EN: open_drain_en_r <= host_wdata[7:0];
            CSR_PIN_ROUTER0:   pin_router_r[0] <= host_wdata;
            CSR_PIN_ROUTER0+1: pin_router_r[1] <= host_wdata;
            CSR_PIN_ROUTER0+2: pin_router_r[2] <= host_wdata;
            CSR_PIN_ROUTER0+3: pin_router_r[3] <= host_wdata;
            CSR_PIN_ROUTER0+4: pin_router_r[4] <= host_wdata;
            CSR_PIN_ROUTER0+5: pin_router_r[5] <= {4'b0000, host_wdata[11:0]};
            CSR_LUT_OREG:      lut_oreg_r <= host_wdata;
            default: begin end
          endcase
        end
      end
    end
  end


  // CRC seed half-write strobes: a write to BCU_CRC_STATE_LO (offset 7) or
  // BCU_CRC_STATE_HI (offset 8) in bank k pulses the matching write-enable
  // for one cycle and presents the written data on that bank's seed bus.
  // The CRC engine inside each BCU is the sole state storage.
  integer swc;
  reg [7:0] swaddr;
  always @* begin
    swaddr = 8'h0;
    crc_lo_we = 4'h0;
    crc_hi_we = 4'h0;
    crc_seed_flat = 64'h0;
    rle_sticky_clr = 1'b0;
    if (host_we) begin
      if (host_addr == CSR_RLE_CTRL && host_wdata[8])
        rle_sticky_clr = 1'b1;
      if (host_addr >= CSR_CORE_BANK && host_addr < 8'h80) begin
        swaddr = (host_addr - CSR_CORE_BANK) >> 4;
        if (host_addr[3:0] == 4'h7) begin
          crc_lo_we[swaddr[1:0]] = 1'b1;
          crc_seed_flat[swaddr[1:0]*16 +: 16] = host_wdata;
        end
        if (host_addr[3:0] == 4'h8) begin
          crc_hi_we[swaddr[1:0]] = 1'b1;
          crc_seed_flat[swaddr[1:0]*16 +: 16] = host_wdata;
        end
      end
    end
    for (swc = 0; swc < 4; swc = swc + 1) begin
      if (core_we[swc]) begin
        swaddr = core_waddr_flat[swc*8 +: 8];
        if (swaddr == CSR_CORE_BANK + swc*16 + 7) begin
          crc_lo_we[swc] = 1'b1;
          crc_seed_flat[swc*16 +: 16] = core_wdata_flat[swc*16 +: 16];
        end
        if (swaddr == CSR_CORE_BANK + swc*16 + 8) begin
          crc_hi_we[swc] = 1'b1;
          crc_seed_flat[swc*16 +: 16] = core_wdata_flat[swc*16 +: 16];
        end
        if (swaddr == CSR_EV_SET || swaddr == CSR_EV_CLR) begin
        end
      end
    end
  end

  assign rle_ctrl = {9'b0, rle_ctrl_r};
  assign rle_probe_mask = rle_probe_mask_r;

endmodule
