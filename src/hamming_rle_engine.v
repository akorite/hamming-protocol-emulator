/*
 * Copyright (c) 2026 akorite
 * SPDX-License-Identifier: Apache-2.0
 */

`timescale 1ns/1ps
`default_nettype none

// SRAM1-backed RLE logic analyzer and waveform replay engine (KTD6).
//
// Record format is one 32-bit SRAM word {delta[15:0], probe[15:0]}; delta
// counts capture-clock cycles since the previous record.  In CAPTURE mode
// an edge on any probe bit armed by probe_mask pushes a record into the
// 8-entry skid FIFO; when no edge arrives for 16'hFFFF cycles the engine
// pushes the heartbeat {16'hFFFF, prev_probe} so idle gaps of any length
// are lossless.  An edge exactly at delta == 16'hFFFF pushes
// {16'hFFFF, new_probe} instead - the two encodings are distinguished by
// whether the probe word changed.
//
// In REPLAY mode the engine reads the whole ring (base..base+mask) into
// the same FIFO, asserts replay_active only after the first record is
// buffered, then walks each record: dwell delta cycles, drive probe onto
// replay_data.  Replay stops cleanly - replay_active drops and
// replay_data holds - at ring end, on underflow, or when mode leaves
// REPLAY.
//
// The ring is aligned at rle_base with power-of-two-minus-one rle_mask
// (word count minus one).  overflow/underflow are sticky until the CSR
// sticky_clear pulse.
module hamming_rle_engine (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [15:0] probe,         // capture probe vector
    input  wire [15:0] probe_mask,    // edge-detect arm mask
    input  wire [1:0]  mode,          // 0=OFF 1=CAPTURE 2=REPLAY
    input  wire        trig_start_en,
    input  wire        trig_stop_en,
    input  wire        trig_in,       // selected LUT trigger output
    input  wire        sticky_clear,
    input  wire [8:0]  rle_base,
    input  wire [8:0]  rle_mask,

    // SRAM 1 arbitration client port (level req, comb grant, read data
    // returns three accepting edges after the accepting edge).
    output reg         mem_req,
    output reg         mem_we,
    output reg  [8:0]  mem_addr,
    output reg  [31:0] mem_wdata,
    input  wire        mem_grant,
    input  wire        mem_rvalid,
    input  wire [31:0] mem_rdata,

    output reg  [15:0] replay_data,
    output reg         replay_active,

    output reg         overflow,
    output reg         underflow,
    output wire        capturing,
    output reg  [8:0]  wr_ptr
);

  localparam MODE_OFF     = 2'd0;
  localparam MODE_CAPTURE = 2'd1;
  localparam MODE_REPLAY  = 2'd2;

  // ------------------------------------------------------------------
  // Shared 8x32 FIFO.  CAPTURE: records pushed by edge/heartbeat logic,
  // popped when an SRAM write is accepted.  REPLAY: records pushed when
  // read data returns, popped when the replay dwell finishes.
  // ------------------------------------------------------------------
  reg [31:0] fifo [0:7];
  reg [2:0]  fifo_rd;
  reg [2:0]  fifo_wr;
  reg [3:0]  fifo_cnt;
  wire fifo_full  = (fifo_cnt >= 4'd8);
  wire fifo_empty = (fifo_cnt == 4'd0);

  reg        push;
  reg        pop;
  reg [31:0] push_data;

  always @(posedge clk) begin
    if (!rst_n) begin
      fifo_rd  <= 3'd0;
      fifo_wr  <= 3'd0;
      fifo_cnt <= 4'd0;
    end else begin
      case ({push && !fifo_full, pop && !fifo_empty})
        2'b10: begin
          fifo[fifo_wr] <= push_data;
          fifo_wr <= fifo_wr + 3'd1;
          fifo_cnt <= fifo_cnt + 4'd1;
        end
        2'b01: begin
          fifo_rd <= fifo_rd + 3'd1;
          fifo_cnt <= fifo_cnt - 4'd1;
        end
        2'b11: begin
          fifo[fifo_wr] <= push_data;
          fifo_wr <= fifo_wr + 3'd1;
          fifo_rd <= fifo_rd + 3'd1;
        end
        default: begin end
      endcase
    end
  end

  // ------------------------------------------------------------------
  // Ring addressing.  Aligned base, power-of-two ring: the low mask bits
  // of the pointer are the ring offset.  ring_next wraps inside the ring.
  // ------------------------------------------------------------------
  reg [8:0] ring_wptr;   // capture: next write address
  wire [8:0] ring_wnext = ((ring_wptr & rle_mask) == rle_mask) ?
                          rle_base :
                          (rle_base | ((ring_wptr + 9'd1) & rle_mask));

  // ------------------------------------------------------------------
  // Capture edge detection.
  // ------------------------------------------------------------------
  reg [15:0] prev_probe;
  reg [15:0] delta_cnt;
  reg        cap_active;
  reg        trig_started;
  reg        trig_d;

  wire [15:0] probe_chg = (probe ^ prev_probe) & probe_mask;
  wire        edge_hit  = |probe_chg;
  wire        trig_rise =  trig_in & ~trig_d;

  wire cap_enabled = (mode == MODE_CAPTURE) && cap_active &&
                     (~trig_start_en | trig_started);

  wire cap_edge_push = cap_enabled & edge_hit;
  wire cap_hb_push   = cap_enabled & ~edge_hit & (delta_cnt == 16'hFFFF);
  wire cap_will_push = cap_edge_push | cap_hb_push;

  // ------------------------------------------------------------------
  // Replay sequencer.  Reads are issued in ring order with at most one
  // outstanding; a credit guard (fifo occupancy + in-flight) prevents
  // overflow of the 8-entry FIFO.  The sequencer pops a record, dwells
  // delta cycles, drives the probe, and repeats until every ring word
  // has been consumed.
  // ------------------------------------------------------------------
  localparam RP_IDLE = 2'd0;
  localparam RP_RUN  = 2'd1;
  localparam RP_DONE = 2'd2;
  reg [1:0]  rp_state;
  reg [8:0]  rp_rd_ptr;      // next read address inside the ring
  reg [9:0]  rp_issued;      // reads issued this replay pass
  reg [9:0]  rp_consumed;    // records popped by the sequencer
  reg [15:0] rp_wait;
  reg        rp_rec_valid;
  reg        rd_inflight;

  wire [9:0] ring_size = {1'b0, rle_mask} + 10'd1;
  wire       rp_all_issued   = (rp_issued >= ring_size);
  wire       rp_all_consumed = (rp_consumed >= ring_size);
  // Credit: never issue a read whose data could arrive to a full FIFO.
  wire [4:0] fifo_space = 5'd8 - {1'b0, fifo_cnt};
  wire       rp_can_issue = !rd_inflight && !rp_all_issued &&
                            (fifo_space >= 5'd2);

  assign capturing = cap_active;

  // ------------------------- push/pop arbitration ---------------------
  wire rp_rec_done = (rp_state == RP_RUN) && rp_rec_valid &&
                     (rp_wait == 16'd0);
  wire mem_accept = mem_req & mem_grant;

  always @* begin
    push = 1'b0;
    pop  = 1'b0;
    push_data = 32'h00000000;
    if (mode == MODE_CAPTURE) begin
      if (cap_will_push) begin
        push = 1'b1;
        push_data = {delta_cnt, probe};
      end
      if (mem_accept && mem_we)
        pop = 1'b1;
    end else if (mode == MODE_REPLAY) begin
      if (mem_rvalid) begin
        push = 1'b1;
        push_data = mem_rdata;
      end
      if (rp_rec_done)
        pop = 1'b1;
    end
  end

  integer dummy;
  always @(posedge clk) begin
    if (!rst_n) begin
      prev_probe    <= 16'h0;
      delta_cnt     <= 16'h0;
      cap_active    <= 1'b0;
      trig_started  <= 1'b0;
      trig_d        <= 1'b0;
      ring_wptr     <= 9'h0;
      wr_ptr        <= 9'h0;
      mem_req       <= 1'b0;
      mem_we        <= 1'b0;
      mem_addr      <= 9'h0;
      mem_wdata     <= 32'h0;
      overflow      <= 1'b0;
      underflow     <= 1'b0;
      replay_data   <= 16'h0;
      replay_active <= 1'b0;
      rp_state      <= RP_IDLE;
      rp_rd_ptr     <= 9'h0;
      rp_issued     <= 10'd0;
      rp_consumed   <= 10'd0;
      rp_wait       <= 16'h0;
      rp_rec_valid  <= 1'b0;
      rd_inflight   <= 1'b0;
    end else begin
      trig_d <= trig_in;

      if (sticky_clear) begin
        overflow  <= 1'b0;
        underflow <= 1'b0;
      end

      // ---------------------------------------------------- OFF -----
      if (mode == MODE_OFF) begin
        cap_active    <= 1'b0;
        trig_started  <= 1'b0;
        delta_cnt     <= 16'h0;
        mem_req       <= 1'b0;
        rd_inflight   <= 1'b0;
        replay_active <= 1'b0;
        rp_state      <= RP_IDLE;
        rp_rec_valid  <= 1'b0;
        rp_issued     <= 10'd0;
        rp_consumed   <= 10'd0;
      end

      // ----------------------------------------------- CAPTURE ------
      else if (mode == MODE_CAPTURE) begin
        if (!cap_active) begin
          // Mode entry: reset ring write pointer and drain state.
          cap_active   <= 1'b1;
          trig_started <= ~trig_start_en;
          ring_wptr    <= rle_base;
          wr_ptr       <= rle_base;
          prev_probe   <= probe;
          delta_cnt    <= 16'h0;
          mem_req      <= 1'b0;
          rd_inflight  <= 1'b0;
        end else begin
          if (trig_rise && trig_start_en && !trig_started)
            trig_started <= 1'b1;
          if (trig_rise && trig_stop_en && trig_started)
            cap_active <= 1'b0;

          if (cap_enabled) begin
            prev_probe <= probe;
            if (cap_will_push) begin
              if (fifo_full)
                overflow <= 1'b1;
              delta_cnt <= 16'h0;
            end else begin
              delta_cnt <= delta_cnt + 16'd1;
            end
          end

          // FIFO drain to SRAM: at most one outstanding write.
          if (mem_req && mem_accept) begin
            mem_req   <= 1'b0;
            ring_wptr <= ring_wnext;
            wr_ptr    <= ring_wnext;
          end else if (!mem_req && !fifo_empty) begin
            mem_req   <= 1'b1;
            mem_we    <= 1'b1;
            mem_addr  <= ring_wptr;
            mem_wdata <= fifo[fifo_rd];
          end
        end
      end

      // ------------------------------------------------ REPLAY ------
      else if (mode == MODE_REPLAY) begin
        case (rp_state)
          RP_IDLE: begin
            rp_state      <= RP_RUN;
            rp_rd_ptr     <= rle_base;
            rp_issued     <= 10'd0;
            rp_consumed   <= 10'd0;
            rp_rec_valid  <= 1'b0;
            rp_wait       <= 16'h0;
            rd_inflight   <= 1'b0;
            mem_req       <= 1'b0;
            replay_active <= 1'b0;
          end

          RP_RUN: begin
            // --- read issue --------------------------------------
            if (mem_req && mem_accept) begin
              mem_req     <= 1'b0;
              rd_inflight <= 1'b1;
            end else if (!mem_req && rp_can_issue) begin
              mem_req   <= 1'b1;
              mem_we    <= 1'b0;
              mem_addr  <= rp_rd_ptr;
              rp_issued <= rp_issued + 10'd1;
              rp_rd_ptr <= ((rp_rd_ptr & rle_mask) == rle_mask) ?
                           rle_base :
                           (rle_base | ((rp_rd_ptr + 9'd1) & rle_mask));
            end
            if (mem_rvalid)
              rd_inflight <= 1'b0;

            // --- replay walk -------------------------------------
            if (!rp_rec_valid) begin
              if (!fifo_empty) begin
                // Prefetch satisfied: outputs go live with the first
                // record pickup.
                replay_active <= 1'b1;
                rp_wait       <= fifo[fifo_rd][31:16];
                rp_rec_valid  <= 1'b1;
              end else if (rp_all_consumed ||
                           (rp_all_issued && !rd_inflight && fifo_empty)) begin
                rp_state      <= RP_DONE;
                replay_active <= 1'b0;
              end else if (replay_active && fifo_empty && !rd_inflight &&
                           rp_issued != 10'd0) begin
                // Starved mid-stream: sticky underflow, clean release.
                underflow     <= 1'b1;
                rp_state      <= RP_DONE;
                replay_active <= 1'b0;
              end
            end else begin
              if (rp_wait != 16'd0) begin
                rp_wait <= rp_wait - 16'd1;
              end else begin
                replay_data  <= fifo[fifo_rd][15:0];
                rp_consumed  <= rp_consumed + 10'd1;
                rp_rec_valid <= 1'b0;
                if (rp_consumed + 10'd1 >= ring_size) begin
                  rp_state      <= RP_DONE;
                  replay_active <= 1'b0;
                end
              end
            end
          end

          default: begin
            replay_active <= 1'b0;
            if (mode != MODE_REPLAY)
              rp_state <= RP_IDLE;
          end
        endcase
      end

      // Mode dropped out from under us: release everything cleanly.
      if (mode != MODE_REPLAY && rp_state != RP_IDLE) begin
        rp_state      <= RP_IDLE;
        replay_active <= 1'b0;
        rp_rec_valid  <= 1'b0;
        rd_inflight   <= 1'b0;
        if (mode == MODE_OFF)
          mem_req <= 1'b0;
      end
      if (mode != MODE_CAPTURE && cap_active)
        cap_active <= 1'b0;
    end
  end

endmodule
