/*
 * Copyright (c) 2026 akorite
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Deadline-scheduler formal top.  The harness instantiates one real core and
// the real loop cache, then executes the smallest useful timing program:
//
//   pc0  ldi r1.lo, delta[7:0]
//   pc1  ldi r1.hi, delta[15:8]
//   pc2  set_anchor 0
//   pc3  wait_phase r1
//   pc4+ nop
//
// delta and the reset value of the 20-bit timer are solver-chosen constants.
// The timer therefore obeys the architectural unit-increment modular contract
// while still allowing shallow cover witnesses for both maximum-delta and
// timer-wraparound scheduling scenarios.
module deadline_sched_top (
    input wire clk,
    input wire rst_n
);

    (* anyconst *) reg [15:0] f_wait_delta;
    (* anyconst *) reg [19:0] f_timer_initial;

    // This proof is about live deadline scheduling, not reset re-entry.  The
    // first clock is constrained into reset and reset then stays released;
    // reset clearing itself is already part of the RTL reset branch.
    reg f_h_past_valid;
    initial f_h_past_valid = 1'b0;
    always @(posedge clk) begin
        f_h_past_valid <= 1'b1;
        if (!f_h_past_valid)
            assume (!rst_n);
        else
            assume (rst_n);
    end

    // Architectural free-running timer abstraction: arbitrary modular phase
    // at reset, then exactly one increment per clock.
    reg [19:0] timer_r;
    always @(posedge clk) begin
        if (!rst_n)
            timer_r <= f_timer_initial;
        else
            timer_r <= timer_r + 20'd1;
    end

    // Instruction encodings, kept local and explicit so the formal harness is
    // auditable without depending on the Python assembler.
    wire [15:0] insn_ldi_lo = {4'h1, 2'b01, 2'b00, f_wait_delta[7:0]};
    wire [15:0] insn_ldi_hi = {4'h1, 2'b01, 2'b01, f_wait_delta[15:8]};
    wire [15:0] insn_anchor = 16'h3300;  // timing/set_anchor, imm8=0
    wire [15:0] insn_wait   = 16'h3004;  // timing/wait_phase rs=r1
    wire [15:0] insn_nop    = 16'hA200;  // misc/nop

    function [31:0] rom32;
        input [8:0] addr;
        begin
            case (addr)
                9'd0:    rom32 = {insn_ldi_hi, insn_ldi_lo};
                9'd1:    rom32 = {insn_wait, insn_anchor};
                default: rom32 = {insn_nop, insn_nop};
            endcase
        end
    endfunction

    wire        imem_req;
    wire [8:0]  imem_addr;
    wire        imem_grant;
    wire        imem_rvalid;
    wire [31:0] imem_rdata;

    // Single-client model of the production arbiter contract: immediate
    // grant, read data exactly three accepting edges later.  The proof does
    // not exercise SRAM contents beyond this fixed program; arbitership and
    // data return are the separate sram_arb proof obligation.
    reg [3:0]  imem_vpipe;
    reg [31:0] imem_dpipe0;
    reg [31:0] imem_dpipe1;
    reg [31:0] imem_dpipe2;
    reg [31:0] imem_dpipe3;

    assign imem_grant  = imem_req;
    assign imem_rvalid = imem_vpipe[3];
    assign imem_rdata  = imem_dpipe3;

    always @(posedge clk) begin
        if (!rst_n) begin
            imem_vpipe <= 4'b0000;
            imem_dpipe0 <= 32'h00000000;
            imem_dpipe1 <= 32'h00000000;
            imem_dpipe2 <= 32'h00000000;
            imem_dpipe3 <= 32'h00000000;
        end else begin
            imem_vpipe <= {imem_vpipe[2:0], (imem_req && imem_grant)};
            imem_dpipe0 <= rom32(imem_addr);
            imem_dpipe1 <= imem_dpipe0;
            imem_dpipe2 <= imem_dpipe1;
            imem_dpipe3 <= imem_dpipe2;
        end
    end

    wire [3:0]  core_out;
    wire [3:0]  core_oe;
    wire [15:0] core_status;
    wire [9:0]  core_pc;
    wire [63:0] dbg_regs;
    wire        csr_we;
    wire [7:0]  csr_addr;
    wire [15:0] csr_wdata;
    wire        bcu_tx_push;
    wire        bcu_rx_pop;
    wire [15:0] bcu_tx_data;
    wire [7:0]  bcu_ctrl;
    wire        dmem_req;
    wire        dmem_we;
    wire [8:0]  dmem_addr;
    wire [31:0] dmem_wdata;
    wire [31:0] dmem_wmask;

    hamming_core #(.CORE_ID(0)) dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .run              (1'b1),
        .timer            (timer_r),
        .ev_flags         (8'h00),
        .pin_inputs       (16'h0000),
        .lut_outputs      (16'h0000),
        .core_out         (core_out),
        .core_oe          (core_oe),
        .core_status      (core_status),
        .core_pc_out      (core_pc),
        .dbg_regs_flat    (dbg_regs),
        .csr_we           (csr_we),
        .csr_addr         (csr_addr),
        .csr_wdata        (csr_wdata),
        .csr_rdata        (16'h0000),
        .bcu_tx_push      (bcu_tx_push),
        .bcu_rx_pop       (bcu_rx_pop),
        .bcu_tx_data      (bcu_tx_data),
        .bcu_rx_data      (16'h0000),
        .bcu_stat         (16'h0000),
        .bcu_crc_state    (32'h00000000),
        .bcu_ctrl         (bcu_ctrl),
        .icache_inv_all   (1'b0),
        .imem_req         (imem_req),
        .imem_addr        (imem_addr),
        .imem_grant       (imem_grant),
        .imem_rvalid      (imem_rvalid),
        .imem_rdata       (imem_rdata),
        .dmem_req         (dmem_req),
        .dmem_we          (dmem_we),
        .dmem_addr        (dmem_addr),
        .dmem_wdata       (dmem_wdata),
        .dmem_wmask       (dmem_wmask),
        .dmem_grant       (1'b0),
        .dmem_rvalid      (1'b0),
        .dmem_rdata       (32'h00000000)
    );

    wire _unused = &{1'b0, core_out, core_oe, core_status, core_pc, dbg_regs,
                     csr_we, csr_addr, csr_wdata, bcu_tx_push, bcu_rx_pop,
                     bcu_tx_data, bcu_ctrl, dmem_req, dmem_we, dmem_addr,
                     dmem_wdata, dmem_wmask};

endmodule

`default_nettype wire
