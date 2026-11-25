/*
 * Copyright (c) 2026 akorite
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none
`include "hamming_defs.vh"

// One deterministic 16-bit Hamming protocol core with an attached 8x32-bit
// loop cache. Decode and external request control are combinational during
// the fetch-valid cycle; architectural state updates once at the clock edge.
// Memory instructions hold their request until grant and then retire only
// when the bounded read/write completion condition occurs.
module hamming_core #(
    parameter CORE_ID = 0
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        run,
    input  wire [19:0] timer,
    input  wire [7:0]  ev_flags,
    input  wire [15:0] pin_inputs,
    input  wire [15:0] lut_outputs,

    output reg  [3:0]  core_out,
    output reg  [3:0]  core_oe,
    output wire [15:0] core_status,
    output wire [9:0]  core_pc_out,
    output wire [63:0] dbg_regs_flat,

    output wire        csr_we,
    output wire [7:0]  csr_addr,
    output wire [15:0] csr_wdata,
    input  wire [15:0] csr_rdata,

    output wire        bcu_tx_push,
  output wire        bcu_rx_pop,
    output wire [15:0] bcu_tx_data,
    input  wire [15:0] bcu_rx_data,
    input  wire [15:0] bcu_stat,
    input  wire [31:0] bcu_crc_state,
    output reg  [7:0]  bcu_ctrl,

    input  wire        icache_inv_all,

    output wire        imem_req,
    output wire [8:0]  imem_addr,
    input  wire        imem_grant,
    input  wire        imem_rvalid,
    input  wire [31:0] imem_rdata,

    output wire        dmem_req,
    output wire        dmem_we,
    output wire [8:0]  dmem_addr,
    output wire [31:0] dmem_wdata,
    output wire [31:0] dmem_wmask,
    input  wire        dmem_grant,
    input  wire        dmem_rvalid,
    input  wire [31:0] dmem_rdata
);

  localparam MEM_NONE   = 2'd0;
  localparam MEM_DREAD  = 2'd1;
  localparam MEM_DWRITE = 2'd2;
  localparam MEM_IREAD  = 2'd3;

  reg [15:0] rf0;
  reg [15:0] rf1;
  reg [15:0] rf2;
  reg [15:0] rf3;
  reg [9:0]  pc;
  reg        z;
  reg        c;
  reg        eq;
  reg [19:0] anchor;
  reg        deadline_missed;
  reg        halted_r;

  reg [9:0] call0;
  reg [9:0] call1;
  reg [1:0] call_depth;

  reg        wait_active;
  reg [19:0] wait_deadline;

  reg        sync_active;
  reg [3:0]  sync_mask;
  reg [15:0] sync_left;

  reg [1:0]  mem_state;
  reg [1:0]  mem_rt_r;
  reg [1:0]  mem_lane;
  reg        mem_is_byte;
  reg        imem_commit_pending;
  reg [15:0] imem_commit_data;

  reg        fetch_outstanding;

  // Four-entry register file.  Manual read muxes and a single write decoder
  // keep Yosys from lowering the tiny architectural register file as a
  // generic memory with replicated memwr slicing logic.
  task rf_write;
    input [1:0] idx;
    input [15:0] value;
    begin
      case (idx)
        2'd0: rf0 <= value;
        2'd1: rf1 <= value;
        2'd2: rf2 <= value;
        default: rf3 <= value;
      endcase
    end
  endtask

  // ------------------------------------------------------------------------
  // Fetch and loop-cache lookup
  // ------------------------------------------------------------------------

  wire [8:0] pc_waddr = pc[9:1];
  wire       cache_hit_raw;
  wire [31:0] cache_data;
  wire       fetch_resp = fetch_outstanding && imem_rvalid;
  wire       cache_hit = cache_hit_raw && !icache_inv_all;
  wire       fetch_valid = (cache_hit || fetch_resp) && !icache_inv_all;
  wire [31:0] fetch_word = cache_hit ? cache_data : imem_rdata;
  wire [15:0] instr = pc[0] ? fetch_word[31:16] : fetch_word[15:0];

  wire fill_en = fetch_resp && !icache_inv_all;

  hamming_icache i_icache (
      .clk        (clk),
      .rst_n      (rst_n),
      .inv_all    (icache_inv_all),
      .rd_waddr   (pc_waddr),
      .hit        (cache_hit_raw),
      .rd_data    (cache_data),
      .fill_en    (fill_en),
      .fill_waddr (pc_waddr),
      .fill_data  (imem_rdata)
  );

  // ------------------------------------------------------------------------
  // Combinational decode/control
  // ------------------------------------------------------------------------

  wire [3:0] major = instr[15:12];
  wire [3:0] sub   = instr[11:8];
  wire [1:0] alu_rd = instr[7:6];
  wire [1:0] alu_rs = instr[5:4];
  wire [1:0] ldi_rd = instr[11:10];
  wire [1:0] ldi_half = instr[9:8];
  wire [3:0] pin_src = instr[7:4];
  wire [1:0] pin_rd  = instr[3:2];
  wire [1:0] tim_rs  = instr[3:2];
  wire [1:0] mem_ra  = instr[7:6];
  wire [1:0] mem_imm2 = instr[5:4];
  wire [1:0] mem_rt_sel  = instr[3:2];

  // Shared register-file read ports for the four operand positions used by
  // the instruction formats.  The muxes reference rf0-rf3 directly in the
  // continuous-assign expressions: a function call would hide the registers
  // from the assignment's implicit sensitivity and the wires could go stale
  // in simulation when a register is written back-to-back.
  wire [15:0] rf_76   = (instr[7:6]   == 2'd0) ? rf0 :
                        (instr[7:6]   == 2'd1) ? rf1 :
                        (instr[7:6]   == 2'd2) ? rf2 : rf3;
  wire [15:0] rf_54   = (instr[5:4]   == 2'd0) ? rf0 :
                        (instr[5:4]   == 2'd1) ? rf1 :
                        (instr[5:4]   == 2'd2) ? rf2 : rf3;
  wire [15:0] rf_32   = (instr[3:2]   == 2'd0) ? rf0 :
                        (instr[3:2]   == 2'd1) ? rf1 :
                        (instr[3:2]   == 2'd2) ? rf2 : rf3;
  wire [15:0] rf_1110 = (instr[11:10] == 2'd0) ? rf0 :
                        (instr[11:10] == 2'd1) ? rf1 :
                        (instr[11:10] == 2'd2) ? rf2 : rf3;

  function legal_instr;
    input [15:0] w;
    reg [3:0] mj;
    reg [3:0] sb;
    begin
      mj = w[15:12];
      sb = w[11:8];
      legal_instr = 1'b0;
      case (mj)
        `HAM_MAJOR_ALU: begin
          legal_instr = (sb <= 4'hB) && ((w & 16'h000F) == 16'h0) &&
                        ((sb < 4'h6 || sb == 4'h8 || sb == 4'h9) || w[5:4] == 2'b00);
        end
        `HAM_MAJOR_LDI8: legal_instr = (w[9:8] <= 2'b01);
        `HAM_MAJOR_PIN: begin
          if (sb == `HAM_PIN_OUT || sb == `HAM_PIN_OE)
            legal_instr = 1'b1;
          else if (sb == `HAM_PIN_IN || sb == `HAM_PIN_READ)
            legal_instr = (w[1:0] == 2'b00);
        end
        `HAM_MAJOR_TIMING: begin
          if (sb == `HAM_TIM_WAIT_PHASE_RS)
            legal_instr = ((w & 16'h00F3) == 16'h0);
          else if (sb == `HAM_TIM_WAIT_PHASE_IMM || sb == `HAM_TIM_SET_ANCHOR)
            legal_instr = 1'b1;
          else if (sb == `HAM_TIM_SYNC_EVENT)
            legal_instr = (w[1:0] == 2'b00);
          else if (sb == `HAM_TIM_READ_TIMER)
            legal_instr = (w[1:0] == 2'b00) && (w[7:4] <= 4'h1);
        end
        `HAM_MAJOR_BCU: begin
          if (sb == `HAM_BCU_TX || sb == `HAM_BCU_RX || sb == `HAM_BCU_STAT)
            legal_instr = ((w & 16'h00F3) == 16'h0);
          else if (sb == `HAM_BCU_CTRL)
            legal_instr = 1'b1;
          else if (sb == `HAM_BCU_CRC_RD)
            legal_instr = ((w & 16'h0033) == 16'h0) && (w[7:6] <= 2'b01);
        end
        `HAM_MAJOR_MEM:
          legal_instr = (sb <= `HAM_MEM_LD_IMEM) && (w[1:0] == 2'b00);
        `HAM_MAJOR_CSR_RD,
        `HAM_MAJOR_CSR_WR:
          legal_instr = ((w & 16'h0300) == 16'h0);
        `HAM_MAJOR_JMP:
          legal_instr = ((w & 16'h0C00) == 16'h0);
        `HAM_MAJOR_BR: begin
          if (w[11:10] == `HAM_BR_JEV)
            legal_instr = ((w & 16'h0030) == 16'h0);
          else
            legal_instr = 1'b1;
        end
        `HAM_MAJOR_MISC:
          legal_instr = (sb <= `HAM_MISC_NOP) && ((w & 16'h00FF) == 16'h0);
        default: legal_instr = 1'b0;
      endcase
    end
  endfunction

  wire busy = wait_active || sync_active || (mem_state != MEM_NONE);
  wire exec_valid = run && !halted_r && !busy && fetch_valid;
  wire instr_legal = legal_instr(instr);

  wire is_mem_op = exec_valid && instr_legal && major == `HAM_MAJOR_MEM &&
                   sub <= `HAM_MEM_ST_DMEM_B;
  wire is_ld_imem = exec_valid && instr_legal && major == `HAM_MAJOR_MEM &&
                    sub == `HAM_MEM_LD_IMEM;

  wire [15:0] mem_ea = rf_76 + {14'h0, mem_imm2};
  wire [1:0]  mem_byte_lane = mem_ea[1:0];
  wire [8:0]  dmem_waddr = (sub == `HAM_MEM_LD_DMEM_W || sub == `HAM_MEM_ST_DMEM_W) ?
                           mem_ea[8:0] : mem_ea[10:2];
  wire [31:0] mem_store32 = {16'h0000, rf_32};
  wire [31:0] mem_store8 = ({16'h0000, rf_32} & 32'h000000FF) << (8 * mem_byte_lane);

  wire fetch_need = run && !halted_r && !busy && !fetch_valid &&
                    !fetch_outstanding && !icache_inv_all;

  assign imem_req = fetch_need || is_ld_imem;
  assign imem_addr = is_ld_imem ? mem_ea[9:1] : pc_waddr;

  assign dmem_req = is_mem_op;
  assign dmem_we = is_mem_op && (sub == `HAM_MEM_ST_DMEM_W || sub == `HAM_MEM_ST_DMEM_B);
  assign dmem_addr = dmem_waddr;
  assign dmem_wdata = (sub == `HAM_MEM_ST_DMEM_B) ? mem_store8 : mem_store32;
  assign dmem_wmask = !dmem_we ? 32'h00000000 :
                      (sub == `HAM_MEM_ST_DMEM_B) ? (32'h000000FF << (8 * mem_byte_lane)) :
                      32'hFFFFFFFF;

  assign csr_we = exec_valid && instr_legal && major == `HAM_MAJOR_CSR_WR;
  assign csr_addr = instr[7:0];
  assign csr_wdata = rf_1110;

  assign bcu_tx_push = exec_valid && instr_legal && major == `HAM_MAJOR_BCU &&
                       sub == `HAM_BCU_TX;
  assign bcu_rx_pop = exec_valid && instr_legal && major == `HAM_MAJOR_BCU &&
                      sub == `HAM_BCU_RX;
  assign bcu_tx_data = rf_32;

  wire [15:0] wait_delta16 = (sub == `HAM_TIM_WAIT_PHASE_RS) ? rf_32 : {8'h00, instr[7:0]};
  wire [19:0] wait_delta = {4'h0, wait_delta16} & `HAM_MAX_WAIT_DELTA;
  wire [19:0] wait_new_deadline = anchor + wait_delta;
  wire [19:0] wait_new_diff = timer - wait_new_deadline;
  wire [19:0] wait_cur_diff = timer - wait_deadline;

  assign core_pc_out = pc;
  assign core_status = {11'h000, halted_r, deadline_missed, eq, c, z};
  assign dbg_regs_flat = {rf3, rf2, rf1, rf0};

  // ------------------------------------------------------------------------
  // Architectural state
  // ------------------------------------------------------------------------

  reg [15:0] alu_a;
  reg [16:0] alu_res;

  always @(posedge clk) begin
    if (!rst_n) begin
      rf0 <= 16'h0000;
      rf1 <= 16'h0000;
      rf2 <= 16'h0000;
      rf3 <= 16'h0000;
      pc <= 10'h000;
      z <= 1'b0;
      c <= 1'b0;
      eq <= 1'b0;
      anchor <= 20'h00000;
      deadline_missed <= 1'b0;
      halted_r <= 1'b0;
      core_out <= 4'h0;
      core_oe <= 4'h0;
      bcu_ctrl <= 8'h00;
      call0 <= 10'h000;
      call1 <= 10'h000;
      call_depth <= 2'd0;
      wait_active <= 1'b0;
      wait_deadline <= 20'h00000;
      sync_active <= 1'b0;
      sync_mask <= 4'h0;
      sync_left <= 16'h0000;
      mem_state <= MEM_NONE;
      mem_rt_r <= 2'b00;
      mem_lane <= 2'b00;
      mem_is_byte <= 1'b0;
      imem_commit_pending <= 1'b0;
      imem_commit_data <= 16'h0000;
      fetch_outstanding <= 1'b0;
    end else begin
      // Instruction-fill request bookkeeping. Invalidation always wins over
      // both an ungranted request and an in-flight response.
      if (icache_inv_all) begin
        fetch_outstanding <= 1'b0;
      end else begin
        if (fetch_need && imem_grant)
          fetch_outstanding <= 1'b1;
        if (fetch_resp)
          fetch_outstanding <= 1'b0;
      end

      if (!run || halted_r) begin
        // Architectural state is retained while stopped.
      end else if (wait_active) begin
        if (!wait_cur_diff[19]) begin
          wait_active <= 1'b0;
          pc <= pc + 10'd1;
        end
      end else if (sync_active) begin
        if (((ev_flags & sync_mask) != 8'h00) || (sync_left <= 16'd1)) begin
          sync_active <= 1'b0;
          anchor <= timer;
          pc <= pc + 10'd1;
        end else begin
          sync_left <= sync_left - 16'd1;
        end
      end else if (mem_state == MEM_DWRITE) begin
        mem_state <= MEM_NONE;
        pc <= pc + 10'd1;
      end else if (mem_state == MEM_DREAD) begin
        if (dmem_rvalid) begin
          if (!mem_is_byte) begin
            rf_write(mem_rt_r, dmem_rdata[15:0]);
          end else if (mem_lane == 2'h3)
            rf_write(mem_rt_r, {8'h00, dmem_rdata[31:24]});
          else if (mem_lane == 2'h2)
            rf_write(mem_rt_r, {8'h00, dmem_rdata[23:16]});
          else if (mem_lane == 2'h1)
            rf_write(mem_rt_r, {8'h00, dmem_rdata[15:8]});
          else
            rf_write(mem_rt_r, {8'h00, dmem_rdata[7:0]});
          mem_state <= MEM_NONE;
          pc <= pc + 10'd1;
        end
      end else if (mem_state == MEM_IREAD) begin
        if (imem_commit_pending) begin
          rf_write(mem_rt_r, imem_commit_data);
          imem_commit_pending <= 1'b0;
          mem_state <= MEM_NONE;
          pc <= pc + 10'd1;
        end else if (imem_rvalid) begin
          imem_commit_data <= mem_lane[0] ? imem_rdata[31:16] : imem_rdata[15:0];
          imem_commit_pending <= 1'b1;
        end
      end else if (exec_valid) begin
        pc <= pc + 10'd1;
        if (instr_legal) begin
          case (major)
            `HAM_MAJOR_ALU: begin
              alu_a = rf_76;
              case (sub)
                `HAM_ALU_MOV: begin
                  rf_write(alu_rd, rf_54);
                  z <= (rf_54 == 16'h0000);
                end
                `HAM_ALU_ADD: begin
                  alu_res = {1'b0, alu_a} + {1'b0, rf_54};
                  rf_write(alu_rd, alu_res[15:0]);
                  c <= alu_res[16];
                  z <= (alu_res[15:0] == 16'h0000);
                end
                `HAM_ALU_SUB: begin
                  alu_res = {1'b0, alu_a} - {1'b0, rf_54};
                  rf_write(alu_rd, alu_res[15:0]);
                  c <= (alu_a < rf_54);
                  z <= (alu_res[15:0] == 16'h0000);
                end
                `HAM_ALU_AND: begin
                  rf_write(alu_rd, alu_a & rf_54);
                  z <= ((alu_a & rf_54) == 16'h0000);
                end
                `HAM_ALU_OR: begin
                  rf_write(alu_rd, alu_a | rf_54);
                  z <= ((alu_a | rf_54) == 16'h0000);
                end
                `HAM_ALU_XOR: begin
                  rf_write(alu_rd, alu_a ^ rf_54);
                  z <= ((alu_a ^ rf_54) == 16'h0000);
                end
                `HAM_ALU_SHL: begin
                  rf_write(alu_rd, {alu_a[14:0], 1'b0});
                  c <= alu_a[15];
                  z <= (alu_a[14:0] == 15'h0000);
                end
                `HAM_ALU_SHR: begin
                  rf_write(alu_rd, {1'b0, alu_a[15:1]});
                  c <= alu_a[0];
                  z <= (alu_a[15:1] == 15'h0000);
                end
                `HAM_ALU_CMP: begin
                  c <= (alu_a < rf_54);
                  z <= ((alu_a - rf_54) == 16'h0000);
                  eq <= (alu_a == rf_54);
                end
                `HAM_ALU_NOT: begin
                  rf_write(alu_rd, ~rf_54);
                  z <= ((~rf_54) == 16'h0000);
                end
                `HAM_ALU_INC: begin
                  rf_write(alu_rd, alu_a + 16'd1);
                  z <= ((alu_a + 16'd1) == 16'h0000);
                end
                `HAM_ALU_DEC: begin
                  rf_write(alu_rd, alu_a - 16'd1);
                  z <= ((alu_a - 16'd1) == 16'h0000);
                end
                default: begin end
              endcase
            end

            `HAM_MAJOR_LDI8: begin
              if (ldi_half[0])
                rf_write(ldi_rd, {instr[7:0], rf_1110[7:0]});
              else
                rf_write(ldi_rd, {rf_1110[15:8], instr[7:0]});
            end

            `HAM_MAJOR_PIN: begin
              if (sub == `HAM_PIN_OUT)
                core_out <= (core_out & ~instr[7:4]) | (instr[3:0] & instr[7:4]);
              else if (sub == `HAM_PIN_OE)
                core_oe <= (core_oe & ~instr[7:4]) | (instr[3:0] & instr[7:4]);
              else if (sub == `HAM_PIN_IN)
                rf_write(pin_rd, {15'h0000, pin_inputs[pin_src]});
              else if (sub == `HAM_PIN_READ)
                rf_write(pin_rd, {15'h0000, lut_outputs[pin_src]});
            end

            `HAM_MAJOR_TIMING: begin
              if (sub == `HAM_TIM_WAIT_PHASE_RS || sub == `HAM_TIM_WAIT_PHASE_IMM) begin
                if (wait_new_diff[19]) begin
                  wait_active <= 1'b1;
                  wait_deadline <= wait_new_deadline;
                  pc <= pc;
                end else if (wait_new_diff != 20'h00000) begin
                  deadline_missed <= 1'b1;
                end
              end else if (sub == `HAM_TIM_SYNC_EVENT) begin
                if (((ev_flags & instr[7:4]) != 8'h00) || rf_32 == 16'h0000) begin
                  anchor <= timer;
                end else begin
                  sync_active <= 1'b1;
                  sync_mask <= instr[7:4];
                  sync_left <= rf_32;
                  pc <= pc;
                end
              end else if (sub == `HAM_TIM_SET_ANCHOR) begin
                anchor <= timer + {12'h000, instr[7:0]};
              end else if (sub == `HAM_TIM_READ_TIMER) begin
                if (instr[7:4] == `HAM_TIM_RD_HI)
                  rf_write(tim_rs, {deadline_missed, 11'h000, timer[19:16]});
                else
                  rf_write(tim_rs, timer[15:0]);
              end
            end

            `HAM_MAJOR_BCU: begin
              if (sub == `HAM_BCU_RX)
                rf_write(tim_rs, bcu_rx_data);
              else if (sub == `HAM_BCU_CTRL)
                bcu_ctrl <= instr[7:0];
              else if (sub == `HAM_BCU_STAT)
                rf_write(tim_rs, bcu_stat);
              else if (sub == `HAM_BCU_CRC_RD)
                rf_write(tim_rs, instr[7:6] == `HAM_BCU_CRC_HI ? bcu_crc_state[31:16] : bcu_crc_state[15:0]);
            end

            `HAM_MAJOR_MEM: begin
              pc <= pc;
              if (is_ld_imem && imem_grant) begin
                mem_state <= MEM_IREAD;
                mem_rt_r <= mem_rt_sel;
                mem_lane <= {1'b0, mem_ea[0]};
              end else if (is_mem_op && dmem_grant) begin
                mem_state <= dmem_we ? MEM_DWRITE : MEM_DREAD;
                mem_rt_r <= mem_rt_sel;
                mem_lane <= mem_byte_lane;
                mem_is_byte <= (sub == `HAM_MEM_LD_DMEM_B);
              end
            end

            `HAM_MAJOR_CSR_RD: begin
              rf_write(ldi_rd, csr_rdata);
            end

            `HAM_MAJOR_CSR_WR: begin
              // The external CSR bank samples the combinational write port at
              // this same edge and enforces private-bank ownership.
            end

            `HAM_MAJOR_JMP: begin
              pc <= instr[9:0];
            end

            `HAM_MAJOR_BR: begin
              case (instr[11:10])
                `HAM_BR_JZ:  if (z)  pc <= instr[9:0];
                `HAM_BR_JNZ: if (!z) pc <= instr[9:0];
                `HAM_BR_JEV: if ((ev_flags & instr[9:6]) != 8'h00) pc <= rf_32[9:0];
                `HAM_BR_CALL: begin
                  if (call_depth == 2'd0) begin
                    call0 <= pc + 10'd1;
                    call_depth <= 2'd1;
                  end else if (call_depth == 2'd1) begin
                    call1 <= pc + 10'd1;
                    call_depth <= 2'd2;
                  end else begin
                    call0 <= call1;
                    call1 <= pc + 10'd1;
                  end
                  pc <= instr[9:0];
                end
                default: begin end
              endcase
            end

            `HAM_MAJOR_MISC: begin
              if (sub == `HAM_MISC_RET) begin
                if (call_depth == 2'd0) begin
                  pc <= 10'h000;
                end else if (call_depth == 2'd1) begin
                  pc <= call0;
                  call_depth <= 2'd0;
                end else begin
                  pc <= call1;
                  call_depth <= 2'd1;
                end
              end else if (sub == `HAM_MISC_HALT) begin
                halted_r <= 1'b1;
              end
            end

            default: begin end
          endcase
        end
      end
    end
  end

  // HAM_PIN_READ is the canonical LUT-read sub-op name in hamming_defs.vh.
  wire _unused_core_id = &{1'b0, CORE_ID};

endmodule





