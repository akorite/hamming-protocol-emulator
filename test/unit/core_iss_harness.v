// Test-only harness for one Hamming core. It provides single-client
// fixed-latency SRAM 0/1 models with backdoor load ports and wires the core
// into the production CSR bank so CSR timing/ownership are the real thing.
`timescale 1ns/1ps
`default_nettype none

module core_test_mem #(
    parameter RD_LATENCY = 3,
    parameter WR_LATENCY = 1
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        req,
    input  wire        we,
    input  wire [8:0]  addr,
    input  wire [31:0] wdata,
    input  wire [31:0] wmask,
    output wire        grant,
    output wire        rvalid,
    output wire [31:0] rdata,

    input  wire        bwe,
    input  wire [8:0]  baddr,
    input  wire [31:0] bdata,
    input  wire [31:0] bmask
);

  reg [31:0] mem [0:511];
  reg [7:0]  rd_valid_pipe;
  reg [7:0]  wr_valid_pipe;
  reg [8:0]  rd_addr_pipe [0:7];
  reg [8:0]  wr_addr_pipe [0:7];
  reg [31:0] wr_data_pipe [0:7];
  reg [31:0] wr_mask_pipe [0:7];

  assign grant  = req;
  assign rvalid = rd_valid_pipe[RD_LATENCY-1];
  assign rdata  = mem[rd_addr_pipe[RD_LATENCY-1]];

  integer pi;
  always @(posedge clk) begin
    if (!rst_n) begin
      rd_valid_pipe <= 8'h00;
      wr_valid_pipe <= 8'h00;
      for (pi = 0; pi < 8; pi = pi + 1) begin
        rd_addr_pipe[pi] <= 9'h000;
        wr_addr_pipe[pi] <= 9'h000;
        wr_data_pipe[pi] <= 32'h00000000;
        wr_mask_pipe[pi] <= 32'h00000000;
      end
    end else begin
      for (pi = 7; pi > 0; pi = pi - 1) begin
        rd_valid_pipe[pi] <= rd_valid_pipe[pi-1];
        wr_valid_pipe[pi] <= wr_valid_pipe[pi-1];
        rd_addr_pipe[pi] <= rd_addr_pipe[pi-1];
        wr_addr_pipe[pi] <= wr_addr_pipe[pi-1];
        wr_data_pipe[pi] <= wr_data_pipe[pi-1];
        wr_mask_pipe[pi] <= wr_mask_pipe[pi-1];
      end
      rd_valid_pipe[0] <= req && !we;
      wr_valid_pipe[0] <= req &&  we;
      rd_addr_pipe[0] <= addr;
      wr_addr_pipe[0] <= addr;
      wr_data_pipe[0] <= wdata;
      wr_mask_pipe[0] <= wmask;

      if (wr_valid_pipe[WR_LATENCY-1]) begin
        mem[wr_addr_pipe[WR_LATENCY-1]] <=
            (mem[wr_addr_pipe[WR_LATENCY-1]] & ~wr_mask_pipe[WR_LATENCY-1]) |
            (wr_data_pipe[WR_LATENCY-1] & wr_mask_pipe[WR_LATENCY-1]);
      end
      if (bwe)
        mem[baddr] <= (mem[baddr] & ~bmask) | (bdata & bmask);
    end
  end

endmodule

module core_iss_harness (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        run,

    input  wire [15:0] pin_inputs,
    input  wire [15:0] lut_outputs,
    input  wire        icache_inv_all,

    input  wire        host_csr_we,
    input  wire [7:0]  host_csr_addr,
    input  wire [15:0] host_csr_wdata,

    input  wire [15:0] bcu_rx_data,
    input  wire [15:0] bcu_stat,
    input  wire [31:0] bcu_crc_state,

    input  wire        imem_bwe,
    input  wire [8:0]  imem_baddr,
    input  wire [31:0] imem_bdata,
    input  wire        dmem_bwe,
    input  wire [8:0]  dmem_baddr,
    input  wire [31:0] dmem_bdata,

    output wire [15:0] core_status,
    output wire [9:0]  core_pc,
    output wire [3:0]  core_out,
    output wire [3:0]  core_oe,
    output wire [63:0] dbg_regs_flat,
    output wire [7:0]  ev_flags,
    output wire [19:0] timer,
    output wire        bcu_tx_push,
    output wire [15:0] bcu_tx_data,
    output wire [7:0]  bcu_ctrl,

    output wire        imem_req,
    output wire [8:0]  imem_addr,
    output wire        dmem_req,
    output wire        dmem_we,
    output wire [8:0]  dmem_addr,
    output wire [31:0] dmem_wdata,
    output wire [31:0] dmem_wmask
);

  wire        core_csr_we;
  wire [7:0]  core_csr_addr;
  wire [15:0] core_csr_wdata;
  wire [15:0] core_csr_rdata;

  wire        imem_grant;
  wire        imem_rvalid;
  wire [31:0] imem_rdata;
  wire        dmem_grant;
  wire        dmem_rvalid;
  wire [31:0] dmem_rdata;

  wire [63:0] csr_status_flat = {48'h0, core_status};
  wire [39:0] csr_pc_flat     = {30'h0, core_pc};
  wire [15:0] csr_out_flat    = {12'h0, core_out};
  wire [15:0] csr_oe_flat     = {12'h0, core_oe};
  wire [63:0] bcu_rx_flat     = {48'h0, bcu_rx_data};
  wire [63:0] bcu_stat_flat   = {48'h0, bcu_stat};
  wire [63:0] core_rdata_flat;

  hamming_core #(.CORE_ID(0)) i_core (
      .clk            (clk),
      .rst_n          (rst_n),
      .run            (run),
      .timer          (timer),
      .ev_flags       (ev_flags),
      .pin_inputs     (pin_inputs),
      .lut_outputs    (lut_outputs),
      .core_out       (core_out),
      .core_oe        (core_oe),
      .core_status    (core_status),
      .core_pc_out    (core_pc),
      .dbg_regs_flat  (dbg_regs_flat),
      .csr_we         (core_csr_we),
      .csr_addr       (core_csr_addr),
      .csr_wdata      (core_csr_wdata),
      .csr_rdata      (core_csr_rdata),
      .bcu_tx_push    (bcu_tx_push),
      .bcu_tx_data    (bcu_tx_data),
      .bcu_rx_data    (bcu_rx_data),
      .bcu_stat       (bcu_stat),
      .bcu_crc_state  (bcu_crc_state),
      .bcu_ctrl       (bcu_ctrl),
      .icache_inv_all (icache_inv_all),
      .imem_req       (imem_req),
      .imem_addr      (imem_addr),
      .imem_grant     (imem_grant),
      .imem_rvalid    (imem_rvalid),
      .imem_rdata     (imem_rdata),
      .dmem_req       (dmem_req),
      .dmem_we        (dmem_we),
      .dmem_addr      (dmem_addr),
      .dmem_wdata     (dmem_wdata),
      .dmem_wmask     (dmem_wmask),
      .dmem_grant     (dmem_grant),
      .dmem_rvalid    (dmem_rvalid),
      .dmem_rdata     (dmem_rdata)
  );

  hamming_csr i_csr (
      .clk                 (clk),
      .rst_n               (rst_n),
      .host_we             (host_csr_we),
      .host_addr           (host_csr_addr),
      .host_wdata          (host_csr_wdata),
      .host_rdata          (),
      .core_we             ({3'b000, core_csr_we}),
      .core_waddr_flat     ({24'h0, core_csr_addr}),
      .core_wdata_flat     ({48'h0, core_csr_wdata}),
      .core_raddr_flat     ({24'h0, core_csr_addr}),
      .core_rdata_flat     (core_rdata_flat),
      .core_status_flat    (csr_status_flat),
      .core_pc_flat        (csr_pc_flat),
      .core_out_flat       (csr_out_flat),
      .core_oe_flat        (csr_oe_flat),
      .bcu_rx_data_flat    (bcu_rx_flat),
      .bcu_stat_flat       (bcu_stat_flat),
      .rle_wr_ptr          (16'h0000),
      .rle_status          (16'h0000),
      .ctrl_run            (),
      .ev_flags            (ev_flags),
      .timer               (timer),
      .rle_base            (),
      .rle_mask            (),
      .pin_filt_en         (),
      .open_drain_en       (),
      .pin_router_flat     (),
      .lut_tt_flat         (),
      .lut_sel_flat        (),
      .lut_oreg            (),
      .bcu_mode_flat       (),
      .bcu_crc_poly_flat   (),
      .bcu_crc_state_flat  (),
      .bcu_bit_len_flat    (),
      .bcu_pat_match_flat  (),
      .bcu_pat_len_flat    (),
      .bcu_dpll_cfg_flat   (),
      .bcu_tx_data_flat    ()
  );

  assign core_csr_rdata = core_rdata_flat[15:0];

  core_test_mem #(.RD_LATENCY(3), .WR_LATENCY(1)) i_imem (
      .clk    (clk),
      .rst_n  (rst_n),
      .req    (imem_req),
      .we     (1'b0),
      .addr   (imem_addr),
      .wdata  (32'h00000000),
      .wmask  (32'h00000000),
      .grant  (imem_grant),
      .rvalid (imem_rvalid),
      .rdata  (imem_rdata),
      .bwe    (imem_bwe),
      .baddr  (imem_baddr),
      .bdata  (imem_bdata),
      .bmask  (32'hFFFFFFFF)
  );

  core_test_mem #(.RD_LATENCY(5), .WR_LATENCY(1)) i_dmem (
      .clk    (clk),
      .rst_n  (rst_n),
      .req    (dmem_req),
      .we     (dmem_we),
      .addr   (dmem_addr),
      .wdata  (dmem_wdata),
      .wmask  (dmem_wmask),
      .grant  (dmem_grant),
      .rvalid (dmem_rvalid),
      .rdata  (dmem_rdata),
      .bwe    (dmem_bwe),
      .baddr  (dmem_baddr),
      .bdata  (dmem_bdata),
      .bmask  (32'hFFFFFFFF)
  );

endmodule
