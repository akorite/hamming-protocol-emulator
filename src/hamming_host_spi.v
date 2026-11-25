/*
 * Copyright (c) 2026 akorite
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Synchronous oversampled Host SPI Mode 0 target. All host pins are sampled
// only in the core clock domain; when ena is low the pins are synchronizers-
// only and cannot create a session or state change.
//
// Command format: CMD, 16-bit big-endian byte address, then payload. SRAM
// payloads are 32-bit words at aligned byte addresses; CSR payloads are
// 16-bit values. Reads use one mandatory dummy byte. MISO changes only on
// synchronized SCLK falling edges and is driven directly by rd_shift[31].
module hamming_host_spi (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        ena,
    input  wire        host_cs_n,
    input  wire        host_sclk,
    input  wire        host_mosi,
    output wire        host_miso,

    output wire        sram0_req,
    output wire        sram0_we,
    output reg  [8:0]  sram0_addr,
    output reg  [31:0] sram0_wdata,
    output reg  [31:0] sram0_wmask,
    input  wire        sram0_grant,
    input  wire        sram0_rvalid,
    input  wire [31:0] sram0_rdata,

    output wire        sram1_req,
    output wire        sram1_we,
    output reg  [8:0]  sram1_addr,
    output reg  [31:0] sram1_wdata,
    output reg  [31:0] sram1_wmask,
    input  wire        sram1_grant,
    input  wire        sram1_rvalid,
    input  wire [31:0] sram1_rdata,

    output reg         csr_we,
    output reg  [7:0]  csr_addr,
    output reg  [15:0] csr_wdata,
    input  wire [15:0] csr_rdata
);

  localparam ST_IDLE      = 3'd0;
  localparam ST_CMD       = 3'd1;
  localparam ST_ADDR      = 3'd2;
  localparam ST_DUMMY     = 3'd3;
  localparam ST_READ_DATA = 3'd4;
  localparam ST_WRITE     = 3'd5;

  localparam CMD_WR_SRAM0 = 8'h01;
  localparam CMD_WR_SRAM1 = 8'h02;
  localparam CMD_WR_CSR   = 8'h03;
  localparam CMD_RD_SRAM0 = 8'h81;
  localparam CMD_RD_SRAM1 = 8'h82;
  localparam CMD_RD_CSR   = 8'h83;

  // Two-flop input synchronizers.
  reg cs_meta, cs_sync;
  reg sck_meta, sck_sync;
  reg mosi_meta, mosi_sync;
  reg ena_meta, ena_sync;
  reg sck_prev;
  reg was_active;

  wire sync_active = ena_sync & ~cs_sync;
  wire active = sync_active & was_active;
  wire start = sync_active & ~was_active;
  wire done = ~sync_active & was_active;
  wire sck_rise = active & sck_sync & ~sck_prev;
  wire sck_fall = active & ~sck_sync & sck_prev;

  always @(posedge clk) begin
    if (!rst_n) begin
      cs_meta <= 1'b1;
      cs_sync <= 1'b1;
      sck_meta <= 1'b0;
      sck_sync <= 1'b0;
      mosi_meta <= 1'b0;
      mosi_sync <= 1'b0;
      ena_meta <= 1'b0;
      ena_sync <= 1'b0;
      sck_prev <= 1'b0;
      was_active <= 1'b0;
    end else begin
      cs_meta <= host_cs_n;
      cs_sync <= cs_meta;
      sck_meta <= host_sclk;
      sck_sync <= sck_meta;
      mosi_meta <= host_mosi;
      mosi_sync <= mosi_meta;
      ena_meta <= ena;
      ena_sync <= ena_meta;
      sck_prev <= sck_sync;
      was_active <= sync_active;
    end
  end

  reg [2:0]  state;
  reg [2:0]  dummy_cnt;
  reg [4:0]  bit_cnt;
  reg [4:0]  out_bit_cnt;
  reg [15:0] addr_shift;
  reg [7:0]  cmd_shift;
  reg [30:0] wr_shift;
  reg [31:0] rd_shift;
  reg [31:0] rd_word;
  reg        rd_word_valid;
  reg        preload_pending;
  reg        target_csr;
  reg        target_sram0;
  reg        target_sram1;
  reg        target_write;
  reg [8:0]  write_burst_addr;
  reg [8:0]  read_burst_addr;
  reg [7:0]  csr_burst_addr;
  reg        wr0_pending;
  reg        wr1_pending;
  reg        read_req_pending;
  reg        read_wait_pending;
  reg        read_bank;

  wire [7:0]  cmd_next  = {cmd_shift[6:0], mosi_sync};
  wire [15:0] addr_next = {addr_shift[14:0], mosi_sync};
  wire [31:0] write_word = {wr_shift[30:0], mosi_sync};
  wire [15:0] csr_write_word = {wr_shift[14:0], mosi_sync};

  wire read_grant = read_req_pending &&
                    (read_bank ? sram1_grant : sram0_grant);
  wire read_rvalid = read_wait_pending &&
                     (read_bank ? sram1_rvalid : sram0_rvalid);
  wire [31:0] read_rdata = read_bank ? sram1_rdata : sram0_rdata;

  // Requests are held until grant. At most one read prefetch is outstanding;
  // writes have many SPI bit periods of slack before the next final bit.
  assign sram0_req = wr0_pending || (read_req_pending && !read_bank);
  assign sram1_req = wr1_pending || (read_req_pending &&  read_bank);
  assign sram0_we  = wr0_pending;
  assign sram1_we  = wr1_pending;

  // MISO is exactly one flip-flop stage: it is the registered shift msb.
  assign host_miso = rd_shift[31];

  always @(posedge clk) begin
    if (!rst_n) begin
      state <= ST_IDLE;
      dummy_cnt <= 3'd0;
      bit_cnt <= 5'd0;
      out_bit_cnt <= 5'd0;
      addr_shift <= 16'h0000;
      cmd_shift <= 8'h00;
      wr_shift <= 31'h00000000;
      rd_shift <= 32'h00000000;
      rd_word <= 32'h00000000;
      rd_word_valid <= 1'b0;
      preload_pending <= 1'b0;
      target_csr <= 1'b0;
      target_sram0 <= 1'b0;
      target_sram1 <= 1'b0;
      target_write <= 1'b0;
      write_burst_addr <= 9'h000;
      read_burst_addr <= 9'h000;
      csr_burst_addr <= 8'h00;
      wr0_pending <= 1'b0;
      wr1_pending <= 1'b0;
      read_req_pending <= 1'b0;
      read_wait_pending <= 1'b0;
      read_bank <= 1'b0;
      sram0_addr <= 9'h000;
      sram0_wdata <= 32'h00000000;
      sram0_wmask <= 32'hFFFFFFFF;
      sram1_addr <= 9'h000;
      sram1_wdata <= 32'h00000000;
      sram1_wmask <= 32'hFFFFFFFF;
      csr_we <= 1'b0;
      csr_addr <= 8'h00;
      csr_wdata <= 16'h0000;
    end else begin
      csr_we <= 1'b0;

      // Held write requests retire at the accepting arbiter edge.
      if (wr0_pending && sram0_grant)
        wr0_pending <= 1'b0;
      if (wr1_pending && sram1_grant)
        wr1_pending <= 1'b0;

      // A held read request becomes an accepted, in-flight macro read at its
      // grant edge. The return is captured independently of SPI bit timing.
      if (read_grant) begin
        read_req_pending <= 1'b0;
        read_wait_pending <= 1'b1;
      end

      if (done) begin
        // Early CS_N/ena deassertion discards partial input. Already accepted
        // SRAM writes are complete transactions; no partial payload can reach
        // the arbiters because write pending is set only on the final bit.
        state <= ST_IDLE;
        bit_cnt <= 5'd0;
        out_bit_cnt <= 5'd0;
        dummy_cnt <= 3'd0;
        rd_shift <= 32'h00000000;
        rd_word_valid <= 1'b0;
        preload_pending <= 1'b0;
        read_req_pending <= 1'b0;
        read_wait_pending <= 1'b0;
      end else if (start) begin
        state <= ST_CMD;
        bit_cnt <= 5'd7;
        out_bit_cnt <= 5'd0;
        dummy_cnt <= 3'd7;
        cmd_shift <= 8'h00;
        addr_shift <= 16'h0000;
        wr_shift <= 31'h00000000;
        rd_shift <= 32'h00000000;
        rd_word_valid <= 1'b0;
        preload_pending <= 1'b0;
        target_csr <= 1'b0;
        target_sram0 <= 1'b0;
        target_sram1 <= 1'b0;
        target_write <= 1'b0;
        read_req_pending <= 1'b0;
        read_wait_pending <= 1'b0;
      end else begin
        // MOSI/transaction state is clocked by synchronized rising edges.
        if (sck_rise) begin
          case (state)
            ST_CMD: begin
              cmd_shift <= cmd_next;
              if (bit_cnt == 5'd0) begin
                target_csr <= (cmd_next == CMD_RD_CSR) ||
                              (cmd_next == CMD_WR_CSR);
                target_sram0 <= (cmd_next == CMD_RD_SRAM0) ||
                                (cmd_next == CMD_WR_SRAM0);
                target_sram1 <= (cmd_next == CMD_RD_SRAM1) ||
                                (cmd_next == CMD_WR_SRAM1);
                target_write <= (cmd_next == CMD_WR_CSR) ||
                                (cmd_next == CMD_WR_SRAM0) ||
                                (cmd_next == CMD_WR_SRAM1);
                state <= ST_ADDR;
                bit_cnt <= 5'd15;
              end else begin
                bit_cnt <= bit_cnt - 1'b1;
              end
            end

            ST_ADDR: begin
              addr_shift <= addr_next;
              if (bit_cnt == 5'd0) begin
                csr_addr <= addr_next[7:0];
                csr_burst_addr <= addr_next[7:0];
                if (cmd_shift == CMD_RD_SRAM0 || cmd_shift == CMD_RD_SRAM1 ||
                    cmd_shift == CMD_RD_CSR) begin
                  state <= ST_DUMMY;
                  dummy_cnt <= 3'd7;
                  if (cmd_shift != CMD_RD_CSR) begin
                    // Prefetch the first aligned word as soon as the byte
                    // address is complete. The dummy byte covers macro latency.
                    read_req_pending <= 1'b1;
                    read_wait_pending <= 1'b0;
                    read_bank <= (cmd_shift == CMD_RD_SRAM1);
                    read_burst_addr <= addr_next[10:2] + 9'd1;
                    if (cmd_shift == CMD_RD_SRAM1)
                      sram1_addr <= addr_next[10:2];
                    else
                      sram0_addr <= addr_next[10:2];
                  end
                end else if (cmd_shift == CMD_WR_SRAM0 || cmd_shift == CMD_WR_SRAM1 ||
                             cmd_shift == CMD_WR_CSR) begin
                  state <= ST_WRITE;
                  bit_cnt <= (cmd_shift == CMD_WR_CSR) ? 5'd15 : 5'd31;
                  wr_shift <= 31'h00000000;
                  write_burst_addr <= addr_next[10:2];
                end else begin
                  state <= ST_IDLE;
                end
              end else begin
                bit_cnt <= bit_cnt - 1'b1;
              end
            end

            ST_DUMMY: begin
              if (dummy_cnt == 3'd0) begin
                state <= ST_READ_DATA;
                preload_pending <= 1'b1;
                out_bit_cnt <= target_csr ? 5'd15 : 5'd31;
              end else begin
                dummy_cnt <= dummy_cnt - 1'b1;
              end
            end

            ST_READ_DATA: begin
              // Read data is advanced on falling edges; nothing is sampled
              // from MOSI here.
            end

            ST_WRITE: begin
              wr_shift <= {wr_shift[29:0], mosi_sync};
              if (bit_cnt != 5'd0) begin
                bit_cnt <= bit_cnt - 1'b1;
              end else begin
                if (target_sram0 || target_sram1) begin
                  if (target_sram0) begin
                    wr0_pending <= 1'b1;
                    sram0_wdata <= write_word;
                    sram0_wmask <= 32'hFFFFFFFF;
                    sram0_addr <= write_burst_addr;
                  end else begin
                    wr1_pending <= 1'b1;
                    sram1_wdata <= write_word;
                    sram1_wmask <= 32'hFFFFFFFF;
                    sram1_addr <= write_burst_addr;
                  end
                  write_burst_addr <= write_burst_addr + 9'd1;
                  bit_cnt <= 5'd31;
                end else if (target_csr) begin
                  csr_we <= 1'b1;
                  csr_wdata <= csr_write_word;
                  csr_addr <= csr_burst_addr;
                  csr_burst_addr <= csr_burst_addr + 8'd1;
                  bit_cnt <= 5'd15;
                end else begin
                  state <= ST_IDLE;
                end
                wr_shift <= 31'h00000000;
              end
            end

            default: begin
              state <= ST_IDLE;
            end
          endcase
        end

        // MISO is advanced only on synchronized falling edges. The first
        // falling edge after the dummy byte preloads bit 31 before the first
        // data rising edge, as required by Mode 0.
        if (sck_fall && state == ST_READ_DATA) begin
          if (preload_pending) begin
            preload_pending <= 1'b0;
            if (target_csr) begin
              rd_shift <= {csr_rdata, 16'h0000};
            end else if (rd_word_valid) begin
              rd_shift <= rd_word;
              rd_word_valid <= 1'b0;
              read_req_pending <= 1'b1;
              if (read_bank)
                sram1_addr <= read_burst_addr;
              else
                sram0_addr <= read_burst_addr;
              read_burst_addr <= read_burst_addr + 9'd1;
            end else begin
              rd_shift <= 32'h00000000;
            end
          end else if (out_bit_cnt == 5'd0) begin
            // The final bit was sampled on the preceding rising edge. Load
            // the next prefetched word (or repeat the CSR sample) for the
            // next bit period.
            if (target_csr) begin
              out_bit_cnt <= 5'd15;
              rd_shift <= {csr_rdata, 16'h0000};
            end else begin
              out_bit_cnt <= 5'd31;
              if (rd_word_valid) begin
                rd_shift <= rd_word;
                rd_word_valid <= 1'b0;
                read_req_pending <= 1'b1;
                if (read_bank)
                  sram1_addr <= read_burst_addr;
                else
                  sram0_addr <= read_burst_addr;
                read_burst_addr <= read_burst_addr + 9'd1;
              end else begin
                rd_shift <= 32'h00000000;
              end
            end
          end else begin
            out_bit_cnt <= out_bit_cnt - 1'b1;
            rd_shift <= {rd_shift[30:0], 1'b0};
          end
        end
      end

      // Capture a completed prefetch into the one-word buffer. The next
      // prefetch is issued only when that buffer is consumed into rd_shift,
      // which keeps the pipeline exactly one word ahead even though SPI bit
      // time is much longer than macro latency.
      if (read_rvalid) begin
        read_wait_pending <= 1'b0;
        rd_word <= read_rdata;
        rd_word_valid <= 1'b1;
      end
    end
  end

endmodule
