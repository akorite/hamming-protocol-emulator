// hamming_bcu.v - Bitstream Conditioning Unit (per-core, KTD4)
//
// Integrates the programmable serial CRC engine, the line codecs
// (NRZ/NRZI/Manchester/diff-Manchester/PWM-RZ plus an external-SCLK NRZ
// shift mode for SPI-target use), USB six-ones and CAN five-bit stuffing
// in both directions, 1-16 bit transfers with single-word TX/RX shadow
// buffers, a rx_shift-tapped pattern matcher, USB SE0/EOP handling and
// DPLL-based edge recovery.
//
// Contract summary (tools/hamming_isa.py is the single source):
//   - tx_push loads the TX shadow; the serializer pulls it as soon as the
//     current word finishes, so back-to-back pushes stream continuously.
//   - rx reassembly follows the wire order: MSB-first shifts left (the
//     first wire bit lands at bit len-1); LSB-first inserts at the bit
//     index (the first wire bit stays at bit 0). The pattern matcher taps
//     a dedicated time-ordered shift register with the newest bit at bit0.
//   - Wire bit order follows cfg_bit_len[4] (wire_msb).
//   - len encoding: cfg_bit_len[3:0] = len-1 (0 selects 1 bit).
//   - stat = {10'b0, rx_full, tx_empty, stuff_err, sfd_match, rx_eop_done,
//             crc_ok}; sticky bits are cleared by the stat_clear strobe.
//   - crc_ok is a LEVEL compare of the (xor-adjusted) CRC state against
//     zero, for CAN residue-zero style checking. USB/Ethernet firmware
//     compares the residue via bcu_crc_rd instead.
//   - The single CRC engine is shared between the TX and RX directions;
//     crc_en feeds both, so full-duplex transfers must offload CRC in at
//     most one direction.
//   - USB trailing stuffing: a data run reaching the stuffing threshold on
//     the final bit of a transfer still emits its stuffed complement bit
//     before the EOP sequence.
//   - Manchester uses the IEEE 802.3 convention: first half-cell carries
//     the complement of the data bit, the second the bit itself. The
//     manchester/diff-manchester/pwm codecs treat the DPLL period as the
//     HALF-cell interval.
//   - USB RX dribble tolerance: SE0 observed in place of a data bit ends
//     the packet without a stuffing error even immediately after six ones.
//     Two consecutive SE0 sample ticks qualify rx_eop_done.
//
// Strict Verilog-2005, default_nettype none.
`timescale 1ns/1ps
`default_nettype none

module hamming_bcu (
    input  wire        clk,
    input  wire        rst_n,

    // Core interface
    input  wire        tx_push,      // bcu_tx instruction strobe
    input  wire [15:0] tx_data,      // TX push data
    input  wire        rx_pop,       // bcu_rx instruction strobe
    output wire [15:0] rx_data,      // RX shadow readout
    output wire [15:0] stat,         // status word
    output wire [31:0] crc_state,    // CRC engine state
    input  wire [7:0]  ctrl,         // persistent control byte

    // CSR configuration
    input  wire [15:0] cfg_mode,
    input  wire [31:0] cfg_crc_poly,
    input  wire [31:0] cfg_crc_state,
    input  wire        crc_seed_we,  // synchronous seed-load strobe
    input  wire        crc_lo_we,    // CSR half-write strobes (BCU_CRC_STATE_*)
    input  wire        crc_hi_we,
    input  wire [15:0] crc_seed16,
    input  wire [15:0] cfg_bit_len,
    input  wire [15:0] cfg_pat_match,
    input  wire [15:0] cfg_pat_len,
    input  wire [15:0] cfg_dpll,

    // Wire side
    input  wire        rx_p,         // positive/single-ended wire input
    input  wire        rx_n,         // negative wire input (diff pairs)
    output reg         tx_p,
    output reg         tx_n,
    output reg         tx_oe,

    // External SCLK (nrz_extclk SPI-target mode)
    input  wire        ext_sclk
);

    // ---------------------------------------------------------------------
    // Configuration decode
    // ---------------------------------------------------------------------
    wire [2:0] codec      = cfg_mode[2:0];
    wire [1:0] stuff_mode = cfg_mode[4:3];
    wire [4:0] crc_wid_m1 = cfg_mode[10:5];
    wire       crc_msb    = cfg_mode[11];
    wire       crc_init1  = cfg_mode[12];
    wire       crc_xor1   = cfg_mode[13];
    wire       crc_en     = cfg_mode[14];
    wire       rx_diff    = cfg_mode[15];

    wire [4:0] xfr_len    = {1'b0, cfg_bit_len[3:0]} + 5'd1;  // 1..16
    wire       wire_msb   = cfg_bit_len[4];

    wire       codec_half = (codec == 3'd2) || (codec == 3'd3) ||
                            (codec == 3'd4);  // half-cell-rate codecs
    wire       ext_mode   = (codec == 3'd5);

    // ctrl bits
    wire ctl_tx_arm    = ctrl[0];
    wire ctl_rx_arm    = ctrl[1];
    wire ctl_crc_reset = ctrl[2];
    wire ctl_eop_req   = ctrl[3];
    wire ctl_stat_clr  = ctrl[4];
    wire ctl_soft_rst  = ctrl[5];

    // Edge-detected ctrl strobes
    reg stat_clr_d, soft_rst_d;
    wire stat_clr_pulse = ctl_stat_clr & ~stat_clr_d;
    wire soft_rst_pulse = ctl_soft_rst & ~soft_rst_d;

    // ---------------------------------------------------------------------
    // CRC engine
    // ---------------------------------------------------------------------
    reg         crc_step;
    reg         crc_bit;
    wire [31:0] crc_raw;

    hamming_bcu_crc u_crc (
        .clk(clk), .rst_n(rst_n),
        .cfg_width_m1(crc_wid_m1),
        .cfg_msb(crc_msb),
        .cfg_poly(cfg_crc_poly),
        .load_en(crc_seed_we | ctl_crc_reset),
        .load_state(crc_seed_we ? cfg_crc_state :
                    ((crc_lo_we | crc_hi_we) ? {16'h0000, crc_seed16} :
                     (crc_init1 ? 32'hFFFFFFFF : 32'h00000000))),
        .lo_we(crc_lo_we),
        .hi_we(crc_hi_we),
        .step_en(crc_step),
        .step_bit(crc_bit),
        .state(crc_raw),
        .width_m1_out()
    );

    wire [31:0] crc_mask = (crc_wid_m1 == 5'd31) ? 32'hFFFFFFFF :
                           ((32'h1 << (crc_wid_m1 + 5'd1)) - 32'h1);
    assign crc_state = crc_raw ^ (crc_xor1 ? crc_mask : 32'h0);

    // ---------------------------------------------------------------------
    // Wire input conditioning
    // ---------------------------------------------------------------------
    reg rxp_d, rxn_d, sclk_d;
    wire lvl_p = rx_p;
    wire se0 = rx_diff & ~rx_p & ~rx_n;
    wire wire_edge = (rx_p != rxp_d) | (rx_diff & (rx_n != rxn_d));
    wire sclk_rise = ext_mode &  ext_sclk & ~sclk_d;
    wire sclk_fall = ext_mode & ~ext_sclk &  sclk_d;

    // ---------------------------------------------------------------------
    // DPLL for internally clocked RX
    // ---------------------------------------------------------------------
    wire       dpll_sample, dpll_cell_end, dpll_eof;
    wire [3:0] dpll_miss;

    hamming_bcu_dpll u_dpll (
        .clk(clk), .rst_n(rst_n),
        .run(ctl_rx_arm & ~ext_mode),
        .edge_in(wire_edge),
        .cfg_mode(cfg_dpll[1:0]),
        .cfg_win(cfg_dpll[7:4]),
        .cfg_period(cfg_dpll[15:8]),
        .sample(dpll_sample),
        .cell_end(dpll_cell_end),
        .miss_cnt(dpll_miss),
        .eof(dpll_eof)
    );

    // RX sample strobe: DPLL or external clock.
    wire rx_tick = ext_mode ? sclk_rise : dpll_sample;

    // TX free-running symbol timer.
    reg  [8:0] tx_sym_cnt;
    wire [8:0] tx_period = (cfg_dpll[15:8] == 8'd0) ? 9'd256 :
                           {1'b0, cfg_dpll[15:8]};
    reg        tx_active;
    wire       tx_tick = tx_active &&
                         (ext_mode ? sclk_fall :
                          (tx_sym_cnt == (tx_period - 9'd1)));

    // ---------------------------------------------------------------------
    // TX engine state
    // ---------------------------------------------------------------------
    localparam TXS_IDLE   = 3'd0;
    localparam TXS_RUN    = 3'd1;
    localparam TXS_TSTUFF = 3'd2;  // one trailing stuff bit before EOP/end
    localparam TXS_EOP    = 3'd3;

    reg [2:0]  txs;
    reg [15:0] tx_shadow;
    reg        tx_shadow_v;
    reg [15:0] tx_shift;
    reg [4:0]  tx_bits_left;
    reg        tx_half;       // half-cell phase (0 = first, 1 = second)
    reg        tx_level;      // NRZI/diff-Manchester output level
    reg [2:0]  tx_run_cnt;    // consecutive equal data bits
    reg        tx_run_val;    // value of the current run
    reg        tx_stuff_pend; // owe one mid-stream stuff bit
    reg [1:0]  eop_cnt;

    // Next data bit: LSB-first consumes tx_shift[0] with a right shift,
    // MSB-first indexes the static register by the bits-left count.
    wire       tx_next_bit = wire_msb ? tx_shift[tx_bits_left - 5'd1] :
                                        tx_shift[0];

    // ---------------------------------------------------------------------
    // RX engine state
    // ---------------------------------------------------------------------
    reg [15:0] rx_shift;
    reg [15:0] mat_shift;  // time-ordered matcher tap, newest bit at bit0
    reg [15:0] rx_shadow;
    reg [4:0]  rx_cnt;
    reg        rx_full;
    reg        rx_prev;       // NRZI previous level / diff-Man prev second
                              // half level
    reg        rx_half;       // half-cell sample phase
    reg        rx_half_lvl;   // first-half level store
    reg [2:0]  rx_run_cnt;
    reg        rx_run_val;
    reg        rx_skip;       // next data bit is a stuff bit: drop it
    reg        rx_started;
    reg [1:0]  se0_cnt;

    // sticky status
    reg st_eop_done, st_sfd, st_stuff_err;

    assign rx_data = rx_shadow;
    assign stat = {10'b0, rx_full, ~tx_shadow_v, st_stuff_err, st_sfd,
                   st_eop_done, (crc_state & crc_mask) == 32'h0};

    // Pattern matcher taps the low pat_len bits of the RX reassembly; the
    // sfd_match status is raised on the bit insertion that completes the
    // pattern (an event, not a level, so stat_clear can drop it).
    wire [15:0] pat_mask = (16'h0001 << cfg_pat_len[3:0]) - 16'h0001;

    // TX output helper for a line level pair.
    task tx_drive;
        input p;
        input n;
        begin
            tx_p <= p;
            tx_n <= n;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin : bcu_seq
        // Locals for the TX bit-emission path. Declared at block scope for
        // Verilog-2005 compatibility.
        reg       emit_bit;
        reg       is_data;
        reg       consume;
        reg       trig;
        reg       last_bit;
        reg       rbit;
        reg       rvalid;
        reg [15:0] sh_next;

        if (!rst_n) begin
            txs          <= TXS_IDLE;
            tx_shadow    <= 16'h0;
            tx_shadow_v  <= 1'b0;
            tx_shift     <= 16'h0;
            tx_bits_left <= 5'd0;
            tx_half      <= 1'b0;
            tx_level     <= 1'b0;
            tx_run_cnt   <= 3'd0;
            tx_run_val   <= 1'b0;
            tx_stuff_pend <= 1'b0;
            eop_cnt      <= 2'd0;
            tx_active    <= 1'b0;
            tx_p         <= 1'b0;
            tx_n         <= 1'b0;
            tx_oe        <= 1'b0;
            tx_sym_cnt   <= 9'd0;
            rx_shift     <= 16'h0;
            mat_shift    <= 16'h0;
            rx_shadow    <= 16'h0;
            rx_cnt       <= 5'd0;
            rx_full      <= 1'b0;
            rx_prev      <= 1'b0;
            rx_half      <= 1'b0;
            rx_half_lvl  <= 1'b0;
            rx_run_cnt   <= 3'd0;
            rx_run_val   <= 1'b0;
            rx_skip      <= 1'b0;
            rx_started   <= 1'b0;
            se0_cnt      <= 2'd0;
            st_eop_done  <= 1'b0;
            st_sfd       <= 1'b0;
            st_stuff_err <= 1'b0;
            stat_clr_d   <= 1'b0;
            soft_rst_d   <= 1'b0;
            rxp_d        <= 1'b0;
            rxn_d        <= 1'b0;
            sclk_d       <= 1'b0;
            crc_step     <= 1'b0;
            crc_bit      <= 1'b0;
        end else begin
            stat_clr_d <= ctl_stat_clr;
            soft_rst_d <= ctl_soft_rst;
            rxp_d      <= rx_p;
            rxn_d      <= rx_n;
            sclk_d     <= ext_sclk;
            crc_step   <= 1'b0;

            // ---------------------------------------------------------
            // Sticky status maintenance
            // ---------------------------------------------------------
            if (stat_clr_pulse || soft_rst_pulse) begin
                st_eop_done  <= 1'b0;
                st_sfd       <= 1'b0;
                st_stuff_err <= 1'b0;
            end
            if (dpll_eof && ctl_rx_arm && rx_started && !rx_diff)
                st_eop_done <= 1'b1;

            // ---------------------------------------------------------
            // TX shadow fill from the core
            // ---------------------------------------------------------
            if (tx_push && txs != TXS_IDLE) begin
                tx_shadow   <= tx_data;
                tx_shadow_v <= 1'b1;
            end

            // ---------------------------------------------------------
            // TX free-running symbol timer
            // ---------------------------------------------------------
            if (!tx_active || soft_rst_pulse) begin
                tx_sym_cnt <= 9'd0;
            end else if (!ext_mode) begin
                if (tx_sym_cnt == (tx_period - 9'd1))
                    tx_sym_cnt <= 9'd0;
                else
                    tx_sym_cnt <= tx_sym_cnt + 9'd1;
            end

            // ---------------------------------------------------------
            // TX engine
            // ---------------------------------------------------------
            if (soft_rst_pulse || !ctl_tx_arm) begin
                txs          <= TXS_IDLE;
                tx_shadow_v  <= 1'b0;
                tx_active    <= 1'b0;
                tx_oe        <= 1'b0;
                tx_p         <= 1'b0;
                tx_n         <= 1'b0;
                tx_run_cnt   <= 3'd0;
                tx_stuff_pend <= 1'b0;
                tx_half      <= 1'b0;
            end else begin
                case (txs)
                TXS_IDLE: begin
                    tx_oe <= 1'b0;
                    if (tx_push) begin
                        // First push starts serialization immediately.
                        tx_shift     <= tx_data;
                        tx_bits_left <= xfr_len;
                        txs          <= TXS_RUN;
                        tx_active    <= 1'b1;
                        tx_half      <= 1'b0;
                        tx_run_cnt   <= 3'd0;
                        tx_stuff_pend <= 1'b0;
                        tx_sym_cnt   <= tx_period - 9'd1;
                        tx_oe        <= 1'b1;
                    end else if (tx_shadow_v) begin
                        tx_shift     <= tx_shadow;
                        tx_shadow_v  <= 1'b0;
                        tx_bits_left <= xfr_len;
                        txs          <= TXS_RUN;
                        tx_active    <= 1'b1;
                        tx_half      <= 1'b0;
                        tx_run_cnt   <= 3'd0;
                        tx_stuff_pend <= 1'b0;
                        tx_sym_cnt   <= tx_period - 9'd1;
                        tx_oe        <= 1'b1;
                    end
                end

                TXS_RUN: begin
                    tx_oe <= 1'b1;
                    if (tx_tick) begin
                        // ---------------------------------------------
                        // Bit selection with stuffing interposition
                        // ---------------------------------------------
                        trig     = 1'b0;
                        last_bit = 1'b0;
                        if (tx_stuff_pend) begin
                            emit_bit = ~tx_run_val;
                            is_data  = 1'b0;
                            consume  = ~codec_half | tx_half;
                        end else begin
                            emit_bit = tx_next_bit;
                            is_data  = 1'b1;
                            consume  = ~codec_half | tx_half;
                        end

                        // ---------------------------------------------
                        // Output formation per codec
                        // ---------------------------------------------
                        if (codec_half) begin
                            if (codec == 3'd2) begin
                                // IEEE 802.3 Manchester
                                tx_drive(tx_half ? emit_bit : ~emit_bit,
                                         tx_half ? ~emit_bit : emit_bit);
                            end else if (codec == 3'd3) begin
                                // Differential Manchester: mid-cell toggle
                                // always, boundary toggle on a 0 bit.
                                if (!tx_half) begin
                                    tx_drive(!emit_bit ? ~tx_level : tx_level,
                                             !emit_bit ? tx_level : ~tx_level);
                                    if (!emit_bit)
                                        tx_level <= ~tx_level;
                                end else begin
                                    tx_drive(~tx_level, tx_level);
                                    tx_level <= ~tx_level;
                                end
                            end else begin
                                // PWM/RZ: data in first half, zero second
                                tx_drive(tx_half ? 1'b0 : emit_bit, 1'b0);
                            end
                            tx_half <= ~tx_half;
                        end else begin
                            if (codec == 3'd1) begin
                                // NRZI: 0 toggles, 1 holds
                                tx_drive(emit_bit ? tx_level : ~tx_level,
                                         emit_bit ? ~tx_level : tx_level);
                                if (!emit_bit)
                                    tx_level <= ~tx_level;
                            end else begin
                                tx_drive(emit_bit, ~emit_bit);
                            end
                        end

                        // ---------------------------------------------
                        // CRC feed on data bits once per bit
                        // ---------------------------------------------
                        if (is_data && consume && crc_en) begin
                            crc_step <= 1'b1;
                            crc_bit  <= emit_bit;
                        end

                        // ---------------------------------------------
                        // Run tracking and stuffing trigger
                        // ---------------------------------------------
                        if (is_data && consume) begin
                            // usb6: trigger on the 6th consecutive one;
                            // can5: trigger on the 5th consecutive equal
                            // bit. run_cnt counts bits already in the run.
                            trig = 1'b0;
                            if (stuff_mode == 2'd1)
                                trig = emit_bit & tx_run_val &
                                       (tx_run_cnt == 3'd5);
                            else if (stuff_mode == 2'd2)
                                trig = (emit_bit == tx_run_val) &
                                       (tx_run_cnt == 3'd4) &
                                       (tx_run_cnt != 3'd0);
                            // update the run counter
                            if (tx_run_cnt != 3'd0 && emit_bit == tx_run_val)
                                tx_run_cnt <= tx_run_cnt + 3'd1;
                            else begin
                                tx_run_cnt <= 3'd1;
                                tx_run_val <= emit_bit;
                            end
                            if (trig) begin
                                tx_run_cnt <= 3'd0;
                                if (tx_bits_left == 5'd1) begin
                                    // trailing stuffing before end/EOP
                                    txs         <= TXS_TSTUFF;
                                    tx_run_val  <= emit_bit;
                                end else begin
                                    tx_stuff_pend <= 1'b1;
                                end
                            end
                        end else if (!is_data && consume) begin
                            tx_stuff_pend <= 1'b0;
                        end

                        // ---------------------------------------------
                        // Bit consumption and end-of-word handling
                        // ---------------------------------------------
                        if (is_data && consume) begin
                            if (!wire_msb)
                                tx_shift <= {1'b0, tx_shift[15:1]};
                            if (tx_bits_left == 5'd1) begin
                                if (!trig) begin
                                    if (tx_shadow_v) begin
                                        tx_shift     <= tx_shadow;
                                        tx_shadow_v  <= 1'b0;
                                        tx_bits_left <= xfr_len;
                                    end else if (ctl_eop_req) begin
                                        txs     <= TXS_EOP;
                                        eop_cnt <= 2'd0;
                                    end else begin
                                        txs       <= TXS_IDLE;
                                        tx_active <= 1'b0;
                                    end
                                end else begin
                                    tx_bits_left <= 5'd1;
                                end
                            end else begin
                                tx_bits_left <= tx_bits_left - 5'd1;
                            end
                        end
                    end
                end

                TXS_TSTUFF: begin
                    // Exactly one complement stuff bit, then the transfer
                    // end is re-evaluated.
                    if (tx_tick) begin
                        if (codec_half) begin
                            if (codec == 3'd2)
                                tx_drive(tx_half ? ~tx_run_val : tx_run_val,
                                         tx_half ? tx_run_val : ~tx_run_val);
                            else if (codec == 3'd3) begin
                                // stuff bits are NRZ-ish in diff manchester:
                                // hold no-boundary-transition encoding
                                tx_drive(tx_level, ~tx_level);
                            end else
                                tx_drive(tx_half ? 1'b0 : ~tx_run_val, 1'b0);
                            tx_half <= ~tx_half;
                        end else begin
                            if (codec == 3'd1) begin
                                // stuff bit value is the complement of the
                                // run, which always toggles in NRZI
                                tx_level <= ~tx_level;
                                tx_drive(~tx_level, tx_level);
                            end else begin
                                tx_drive(~tx_run_val, tx_run_val);
                            end
                        end
                        if (!codec_half || tx_half) begin
                            tx_run_cnt <= 3'd0;
                            if (tx_shadow_v) begin
                                tx_shift     <= tx_shadow;
                                tx_shadow_v  <= 1'b0;
                                tx_bits_left <= xfr_len;
                                txs          <= TXS_RUN;
                                tx_half      <= 1'b0;
                            end else if (ctl_eop_req) begin
                                txs     <= TXS_EOP;
                                eop_cnt <= 2'd0;
                                tx_half <= 1'b0;
                            end else begin
                                txs       <= TXS_IDLE;
                                tx_active <= 1'b0;
                            end
                        end
                    end
                end

                TXS_EOP: begin
                    if (tx_tick) begin
                        if (rx_diff) begin
                            // USB EOP: 2 cell-times SE0 + 1 cell-time J.
                            if (eop_cnt < 2'd2) begin
                                tx_drive(1'b0, 1'b0);
                                eop_cnt <= eop_cnt + 2'd1;
                            end else if (eop_cnt == 2'd2) begin
                                tx_drive(1'b1, 1'b0);  // J
                                eop_cnt <= eop_cnt + 2'd1;
                            end else begin
                                txs       <= TXS_IDLE;
                                tx_active <= 1'b0;
                                tx_oe     <= 1'b0;
                            end
                        end else begin
                            txs       <= TXS_IDLE;
                            tx_active <= 1'b0;
                            tx_oe     <= 1'b0;
                        end
                    end
                end

                default: txs <= TXS_IDLE;
                endcase
            end

            // ---------------------------------------------------------
            // RX engine
            // ---------------------------------------------------------
            if (soft_rst_pulse || !ctl_rx_arm) begin
                rx_shift   <= 16'h0;
                mat_shift  <= 16'h0;
                rx_cnt     <= 5'd0;
                rx_run_cnt <= 3'd0;
                rx_skip    <= 1'b0;
                rx_started <= 1'b0;
                rx_half    <= 1'b0;
                rx_prev    <= 1'b0;
                se0_cnt    <= 2'd0;
            end else if (rx_tick) begin
                if (se0) begin
                    // USB SE0: end-of-packet with dribble tolerance. Two
                    // consecutive SE0 sample ticks qualify rx_eop_done; SE0
                    // in place of a stuff/data bit never raises stuff_err.
                    rx_started <= 1'b1;
                    rx_run_cnt <= 3'd0;
                    rx_skip    <= 1'b0;
                    if (se0_cnt != 2'd3)
                        se0_cnt <= se0_cnt + 2'd1;
                    if (se0_cnt >= 2'd1)
                        st_eop_done <= 1'b1;
                end else begin
                    se0_cnt <= 2'd0;
                    // Decode the raw level into a data bit.
                    rvalid = 1'b1;
                    rbit   = lvl_p;
                    if (codec == 3'd1) begin
                        rbit    = (lvl_p == rx_prev);
                        rx_prev <= lvl_p;
                    end else if (codec == 3'd2) begin
                        if (!rx_half) begin
                            rx_half_lvl <= lvl_p;
                            rvalid = 1'b0;
                        end else begin
                            rbit = lvl_p;
                        end
                        rx_half <= ~rx_half;
                    end else if (codec == 3'd3) begin
                        if (!rx_half) begin
                            rx_half_lvl <= lvl_p;
                            rvalid = 1'b0;
                        end else begin
                            rbit    = (rx_half_lvl == rx_prev);
                            rx_prev <= lvl_p;
                        end
                        rx_half <= ~rx_half;
                    end else if (codec == 3'd4) begin
                        if (!rx_half) begin
                            rbit = lvl_p;
                        end else begin
                            rvalid = 1'b0;
                        end
                        rx_half <= ~rx_half;
                    end

                    if (rvalid) begin
                        rx_started <= 1'b1;
                        if (rx_skip) begin
                            // Dropping one stuff bit: it must be the
                            // complement of the run value.
                            rx_skip    <= 1'b0;
                            rx_run_cnt <= 3'd0;
                            if (rbit == rx_run_val)
                                st_stuff_err <= 1'b1;
                        end else begin
                            if ((stuff_mode == 2'd1 && rbit &&
                                 rx_run_val && rx_run_cnt == 3'd5) ||
                                (stuff_mode == 2'd2 &&
                                 rbit == rx_run_val &&
                                 rx_run_cnt == 3'd4 && rx_run_cnt != 3'd0)) begin
                                rx_skip    <= 1'b1;
                                rx_run_cnt <= 3'd0;
                            end else if (rx_run_cnt != 3'd0 &&
                                         rbit == rx_run_val) begin
                                rx_run_cnt <= rx_run_cnt + 3'd1;
                            end else begin
                                rx_run_cnt <= 3'd1;
                                rx_run_val <= rbit;
                            end

                            // Reassembly: MSB-first shifts left so the
                            // first wire bit lands at bit len-1; LSB-first
                            // inserts at the bit-count index so the first
                            // wire bit stays at bit 0.
                            if (wire_msb)
                                sh_next = {rx_shift[14:0], rbit};
                            else
                                sh_next = (rx_cnt == 5'd0 ? 16'h0000 :
                                           rx_shift) |
                                          (rbit ? (16'h0001 << rx_cnt[3:0])
                                                : 16'h0000);
                            rx_shift  <= sh_next;
                            mat_shift <= {mat_shift[14:0], rbit};
                            if ((cfg_pat_len[3:0] != 4'd0) &&
                                (({mat_shift[14:0], rbit} & pat_mask) ==
                                 (cfg_pat_match & pat_mask)))
                                st_sfd <= 1'b1;
                            if (crc_en) begin
                                crc_step <= 1'b1;
                                crc_bit  <= rbit;
                            end
                            if (rx_cnt == (xfr_len - 5'd1)) begin
                                rx_shadow <= sh_next;
                                rx_full   <= 1'b1;
                                rx_cnt    <= 5'd0;
                            end else begin
                                rx_cnt <= rx_cnt + 5'd1;
                            end
                        end
                    end
                end
            end

            // RX shadow consumption.
            if (rx_pop)
                rx_full <= 1'b0;
        end
    end

endmodule
`default_nettype wire
