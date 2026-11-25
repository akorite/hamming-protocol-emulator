// Test-only integration harness for the Host SPI target, CSR bank, and both
// bounded SRAM arbiters. Client indices match the chip integration: SRAM 0
// clients 0..3 are cores and client 4 is Host SPI; SRAM 1 clients 0..3 are
// cores, client 4 is the RLE engine, and client 5 is Host SPI.
`default_nettype none

module host_spi_arb_harness (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        ena,

    input  wire        host_cs_n,
    input  wire        host_sclk,
    input  wire        host_mosi,
    output wire        host_miso,

    input  wire [3:0]  core_we,
    input  wire [31:0] core_waddr_flat,
    input  wire [63:0] core_wdata_flat,
    input  wire [31:0] core_raddr_flat,
    output wire [63:0] core_rdata_flat,

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
    output wire [95:0] pin_router_flat,
    output wire [255:0] lut_tt_flat,
    output wire [255:0] lut_sel_flat,
    output wire [15:0] lut_oreg,
    output wire [63:0] bcu_mode_flat,
    output wire [127:0] bcu_crc_poly_flat,
    output wire [127:0] bcu_crc_state_flat,
    output wire [63:0] bcu_bit_len_flat,
    output wire [63:0] bcu_pat_match_flat,
    output wire [63:0] bcu_pat_len_flat,
    output wire [63:0] bcu_dpll_cfg_flat,
    output wire [63:0] bcu_tx_data_flat,

    input  wire [3:0]   sram0_ext_req,
    input  wire [3:0]   sram0_ext_we,
    input  wire [35:0]  sram0_ext_addr_flat,
    input  wire [127:0] sram0_ext_wdata_flat,
    input  wire [127:0] sram0_ext_wmask_flat,
    output wire [3:0]   sram0_ext_grant,
    output wire [3:0]   sram0_ext_rvalid,
    output wire [31:0]  sram0_rdata,
    output wire         sram0_write_commit,

    input  wire [4:0]   sram1_ext_req,
    input  wire [4:0]   sram1_ext_we,
    input  wire [44:0]  sram1_ext_addr_flat,
    input  wire [159:0] sram1_ext_wdata_flat,
    input  wire [159:0] sram1_ext_wmask_flat,
    output wire [4:0]   sram1_ext_grant,
    output wire [4:0]   sram1_ext_rvalid,
    output wire [31:0]  sram1_rdata,
    output wire         sram1_write_commit
);

  wire        host_csr_we;
  wire [7:0]  host_csr_addr;
  wire [15:0] host_csr_wdata;
  wire [15:0] host_csr_rdata;

  wire        host0_req;
  wire        host0_we;
  wire [8:0]  host0_addr;
  wire [31:0] host0_wdata;
  wire [31:0] host0_wmask;
  wire        host0_grant;
  wire        host0_rvalid;

  wire        host1_req;
  wire        host1_we;
  wire [8:0]  host1_addr;
  wire [31:0] host1_wdata;
  wire [31:0] host1_wmask;
  wire        host1_grant;
  wire        host1_rvalid;

  hamming_host_spi i_host_spi (
      .clk          (clk),
      .rst_n        (rst_n),
      .ena          (ena),
      .host_cs_n    (host_cs_n),
      .host_sclk    (host_sclk),
      .host_mosi    (host_mosi),
      .host_miso    (host_miso),
      .csr_we       (host_csr_we),
      .csr_addr     (host_csr_addr),
      .csr_wdata    (host_csr_wdata),
      .csr_rdata    (host_csr_rdata),
      .sram0_req    (host0_req),
      .sram0_we     (host0_we),
      .sram0_addr   (host0_addr),
      .sram0_wdata  (host0_wdata),
      .sram0_wmask  (host0_wmask),
      .sram0_grant  (host0_grant),
      .sram0_rvalid (host0_rvalid),
      .sram0_rdata  (sram0_rdata),
      .sram1_req    (host1_req),
      .sram1_we     (host1_we),
      .sram1_addr   (host1_addr),
      .sram1_wdata  (host1_wdata),
      .sram1_wmask  (host1_wmask),
      .sram1_grant  (host1_grant),
      .sram1_rvalid (host1_rvalid),
      .sram1_rdata  (sram1_rdata)
  );

  hamming_csr i_csr (
      .clk                 (clk),
      .rst_n               (rst_n),
      .host_we             (host_csr_we),
      .host_addr           (host_csr_addr),
      .host_wdata          (host_csr_wdata),
      .host_rdata          (host_csr_rdata),
      .core_we             (core_we),
      .core_waddr_flat     (core_waddr_flat),
      .core_wdata_flat     (core_wdata_flat),
      .core_raddr_flat     (core_raddr_flat),
      .core_rdata_flat     (core_rdata_flat),
      .core_status_flat    (core_status_flat),
      .core_pc_flat        (core_pc_flat),
      .core_out_flat       (core_out_flat),
      .core_oe_flat        (core_oe_flat),
      .bcu_rx_data_flat    (bcu_rx_data_flat),
      .bcu_stat_flat       (bcu_stat_flat),
      .rle_wr_ptr          (rle_wr_ptr),
      .rle_status          (rle_status),
      .ctrl_run            (ctrl_run),
      .ev_flags            (ev_flags),
      .timer               (timer),
      .rle_base            (rle_base),
      .rle_mask            (rle_mask),
      .pin_filt_en         (pin_filt_en),
      .open_drain_en       (open_drain_en),
      .pin_router_flat     (pin_router_flat),
      .lut_tt_flat         (lut_tt_flat),
      .lut_sel_flat        (lut_sel_flat),
      .lut_oreg            (lut_oreg),
      .bcu_mode_flat       (bcu_mode_flat),
      .bcu_crc_poly_flat   (bcu_crc_poly_flat),
      .bcu_crc_state_flat  (bcu_crc_state_flat),
      .bcu_bit_len_flat    (bcu_bit_len_flat),
      .bcu_pat_match_flat  (bcu_pat_match_flat),
      .bcu_pat_len_flat    (bcu_pat_len_flat),
      .bcu_dpll_cfg_flat   (bcu_dpll_cfg_flat),
      .bcu_tx_data_flat    (bcu_tx_data_flat)
  );

  wire [4:0]   arb0_req;
  wire [4:0]   arb0_we;
  wire [44:0]  arb0_addr_flat;
  wire [159:0] arb0_wdata_flat;
  wire [159:0] arb0_wmask_flat;
  wire [4:0]   arb0_grant;
  wire [4:0]   arb0_rvalid;

  assign arb0_req = {host0_req, sram0_ext_req};
  assign arb0_we  = {host0_we, sram0_ext_we};
  assign arb0_addr_flat[35:0]  = sram0_ext_addr_flat;
  assign arb0_addr_flat[44:36] = host0_addr;
  assign arb0_wdata_flat[127:0] = sram0_ext_wdata_flat;
  assign arb0_wdata_flat[159:128] = host0_wdata;
  assign arb0_wmask_flat[127:0] = sram0_ext_wmask_flat;
  assign arb0_wmask_flat[159:128] = host0_wmask;
  assign sram0_ext_grant  = arb0_grant[3:0];
  assign sram0_ext_rvalid = arb0_rvalid[3:0];
  assign host0_grant  = arb0_grant[4];
  assign host0_rvalid = arb0_rvalid[4];

  hamming_sram_arb #(.NCLIENTS(5)) i_arb0 (
      .clk          (clk),
      .rst_n        (rst_n),
      .req          (arb0_req),
      .we           (arb0_we),
      .addr_flat    (arb0_addr_flat),
      .wdata_flat   (arb0_wdata_flat),
      .wmask_flat   (arb0_wmask_flat),
      .grant        (arb0_grant),
      .rvalid       (arb0_rvalid),
      .rdata        (sram0_rdata),
      .write_commit (sram0_write_commit)
  );

  wire [5:0]   arb1_req;
  wire [5:0]   arb1_we;
  wire [53:0]  arb1_addr_flat;
  wire [191:0] arb1_wdata_flat;
  wire [191:0] arb1_wmask_flat;
  wire [5:0]   arb1_grant;
  wire [5:0]   arb1_rvalid;

  assign arb1_req = {host1_req, sram1_ext_req};
  assign arb1_we  = {host1_we, sram1_ext_we};
  assign arb1_addr_flat[44:0]  = sram1_ext_addr_flat;
  assign arb1_addr_flat[53:45] = host1_addr;
  assign arb1_wdata_flat[159:0] = sram1_ext_wdata_flat;
  assign arb1_wdata_flat[191:160] = host1_wdata;
  assign arb1_wmask_flat[159:0] = sram1_ext_wmask_flat;
  assign arb1_wmask_flat[191:160] = host1_wmask;
  assign sram1_ext_grant  = arb1_grant[4:0];
  assign sram1_ext_rvalid = arb1_rvalid[4:0];
  assign host1_grant  = arb1_grant[5];
  assign host1_rvalid = arb1_rvalid[5];

  hamming_sram_arb #(.NCLIENTS(6)) i_arb1 (
      .clk          (clk),
      .rst_n        (rst_n),
      .req          (arb1_req),
      .we           (arb1_we),
      .addr_flat    (arb1_addr_flat),
      .wdata_flat   (arb1_wdata_flat),
      .wmask_flat   (arb1_wmask_flat),
      .grant        (arb1_grant),
      .rvalid       (arb1_rvalid),
      .rdata        (sram1_rdata),
      .write_commit (sram1_write_commit)
  );

endmodule
