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
        reg [15:0] rev16;

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
                            // CAN/USB stuffing is defined on the physical
                            // emitted stream: the inserted complement bit is
                            // the first bit of the next run.  Seeding the
                            // run counter here is essential for CAN; without
                            // it five data bits equal to a prior stuff bit
                            // could produce six equal bits on the wire.
                            tx_run_cnt  <= 3'd1;
                            tx_run_val  <= emit_bit;
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
                            // As in the mid-stream stuffing path, the
                            // trailing stuff bit seeds the next physical-run
                            // count even when the transfer ends immediately
                            // afterwards (the count is reinitialized on the
                            // next TXS_IDLE entry).
                            tx_run_cnt <= 3'd1;
                            tx_run_val <= ~tx_run_val;
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
                            // complement of the run value.  The dropped bit
                            // nevertheless remains part of the physical wire
                            // stream and seeds the next run, matching the
                            // transmitter and the CAN five-bit wire rule.
                            rx_skip    <= 1'b0;
                            if (stuff_mode == 2'd2) begin
                                rx_run_cnt <= 3'd1;
                                rx_run_val <= rbit;
                            end else begin
                                rx_run_cnt <= 3'd0;
                            end
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

                            // Reassembly is time-ordered in both bit
                            // orders: the newest wire bit enters at bit 0.
                            // This doubles as the pattern-match window;
                            // LSB-first frames reverse/align at completion.
                            sh_next   = {rx_shift[14:0], rbit};
                            rx_shift  <= sh_next;
                            if ((cfg_pat_len[3:0] != 4'd0) &&
                                ((sh_next & pat_mask) ==
                                 (cfg_pat_match & pat_mask)))
                                st_sfd <= 1'b1;
                            if (crc_en) begin
                                crc_step <= 1'b1;
                                crc_bit  <= rbit;
                            end
                            if (rx_cnt == (xfr_len - 5'd1)) begin
                                if (wire_msb) begin
                                    rx_shadow <= sh_next;
                                end else begin
                                    rev16 = {sh_next[0], sh_next[1],
                                             sh_next[2], sh_next[3],
                                             sh_next[4], sh_next[5],
                                             sh_next[6], sh_next[7],
                                             sh_next[8], sh_next[9],
                                             sh_next[10], sh_next[11],
                                             sh_next[12], sh_next[13],
                                             sh_next[14], sh_next[15]};
                                    rx_shadow <= rev16 >> (5'd16 - xfr_len);
                                end
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

`ifdef FORMAL
    // --------------------------------------------------------------------
    // Formal bit-stuffing contract.  The logical emitted-bit stream is
    // reconstructed at the same tick and half-cell phase at which the TX
    // engine consumes a bit.  NRZ and NRZI are the codecs used by the CAN
    // and USB stuffing profiles; Manchester-family codecs have a different
    // line-level contract and are outside this particular invariant.
    // --------------------------------------------------------------------
    reg f_past_valid;
    initial f_past_valid = 1'b0;

    wire f_tx_control_clear = soft_rst_pulse || !ctl_tx_arm;
    wire f_tx_run_event = tx_active && !f_tx_control_clear && tx_tick &&
                          (txs == TXS_RUN) && (!codec_half || tx_half);
    wire f_tx_tstuff_event = tx_active && !f_tx_control_clear && tx_tick &&
                             (txs == TXS_TSTUFF) && (!codec_half || tx_half);
    wire f_tx_bit_event = f_tx_run_event || f_tx_tstuff_event;
    wire f_tx_bit_isstuff = (txs == TXS_TSTUFF) ||
                            ((txs == TXS_RUN) && tx_stuff_pend);
    wire f_tx_bit_value = f_tx_bit_isstuff ? ~tx_run_val : tx_next_bit;
    wire f_tx_codec01 = (codec == 3'd0) || (codec == 3'd1);
    wire f_tx_stream_active = tx_active &&
                              ((txs == TXS_RUN) || (txs == TXS_TSTUFF));
    wire f_tx_run_tracked = f_tx_codec01 &&
        ((stuff_mode == 2'd2) ||
         ((stuff_mode == 2'd1) && tx_run_val));
    wire f_tx_stream_clear = f_tx_control_clear || !tx_active ||
                             (txs == TXS_IDLE) || (txs == TXS_EOP);

    reg       f_tx_last_valid;
    reg       f_tx_last_bit;
    reg [3:0] f_tx_run_len;

    // The monitor state is asynchronous-reset only to match the DUT's reset
    // discipline.  It carries no silicon cost because the whole block is
    // excluded unless FORMAL is defined.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f_tx_last_valid <= 1'b0;
            f_tx_last_bit   <= 1'b0;
            f_tx_run_len    <= 4'd0;
        end else if (f_tx_stream_clear) begin
            f_tx_last_valid <= 1'b0;
            f_tx_last_bit   <= 1'b0;
            f_tx_run_len    <= 4'd0;
        end else if (f_tx_bit_event) begin
            f_tx_last_valid <= 1'b1;
            f_tx_last_bit   <= f_tx_bit_value;
            if (!f_tx_last_valid || (f_tx_bit_value != f_tx_last_bit))
                f_tx_run_len <= 4'd1;
            else
                f_tx_run_len <= f_tx_run_len + 4'd1;
        end
    end

    always @(posedge clk) begin
        f_past_valid <= 1'b1;
        if (!f_past_valid)
            assume (!rst_n);

        // Protocol configuration is quasi-static while a transmission is in
        // progress.  It may be reprogrammed before arming or after idle, but
        // switching the stuffing rule or transfer length mid-frame is outside
        // the documented software contract.
        if (f_past_valid && tx_active) begin
            // tx_active is already high in the cycle preceding the first bit
            // tick, so stability must include that activation edge rather
            // than waiting for one additional bit period.
            assume ($stable(cfg_mode));
            assume ($stable(cfg_bit_len));
        end

        if (f_past_valid && rst_n) begin
            // Basic state well-formedness lemmas.  Besides being useful
            // safety assertions, they keep the induction step away from
            // impossible bit-index states.
            assert (txs <= TXS_EOP);
            if (txs == TXS_RUN)
                assert (tx_bits_left != 5'd0);

            // Internal run-counter bounds implied by the two trigger rules:
            // CAN resets on the fifth equal bit and USB on the sixth one.
            if ((stuff_mode == 2'd2) && tx_active && f_tx_codec01)
                assert (tx_run_cnt <= 3'd4);
            if ((stuff_mode == 2'd1) && tx_active && tx_run_val && f_tx_codec01)
                assert (tx_run_cnt <= 3'd5);

            // Correlation lemmas between the physical-stream monitor and the
            // implementation's run counter.  These are the key induction
            // lemmas: without them k-induction may choose an unreachable
            // state in which the two run-length representations disagree.
            // A nonzero counter exactly counts the current logical run for
            // CAN (both polarities) and for USB one-runs.  The counter is
            // zero only before the first bit or in the trigger gap before
            // the complement stuff bit.
            if (f_tx_stream_active && f_tx_run_tracked &&
                f_tx_last_valid && (tx_run_cnt != 3'd0)) begin
                assert (tx_run_val == f_tx_last_bit);
                assert ({1'b0, tx_run_cnt} == f_tx_run_len);
            end

            // In the trigger gap the pending/trailing stuff bit must still
            // be the complement of the bit that closed the run.  Besides
            // being a useful safety fact, this makes the stuff-bit transition
            // in the correlation lemma inductive.
            if (f_tx_stream_active && f_tx_codec01 &&
                ((stuff_mode == 2'd1) || (stuff_mode == 2'd2)) &&
                (tx_stuff_pend || (txs == TXS_TSTUFF))) begin
                assert (f_tx_last_valid);
                assert (f_tx_last_bit == tx_run_val);
            end

            // State-form bounds for the reconstructed logical stream.  USB
            // permits at most six consecutive ones; CAN permits at most five
            // consecutive equal bits of either polarity.  The stream-active
            // guard avoids interpreting the monitor's one-cycle drained state
            // after a frame as part of a newly reprogrammed following frame.
            if ((stuff_mode == 2'd1) && f_tx_stream_active && f_tx_codec01 &&
                f_tx_last_valid && f_tx_last_bit)
                assert (f_tx_run_len <= 4'd6);
            if ((stuff_mode == 2'd2) && f_tx_stream_active && f_tx_codec01 &&
                f_tx_last_valid)
                assert (f_tx_run_len <= 4'd5);

            // Transition-form consequences: emitting one more identical bit
            // may produce the sixth USB one or fifth CAN bit, but never the
            // seventh/sixth respectively.  Thus a complement stuff bit must
            // have been interposed immediately after the threshold bit.
            if (f_tx_bit_event && f_tx_last_valid) begin
                if ((stuff_mode == 2'd1) && f_tx_codec01 &&
                    f_tx_bit_value && f_tx_last_bit)
                    assert (f_tx_run_len < 4'd6);
                if ((stuff_mode == 2'd2) && f_tx_codec01 &&
                    (f_tx_bit_value == f_tx_last_bit))
                    assert (f_tx_run_len < 4'd5);
            end

            // Software/soft reset and TX disarm really clear the stuffing
            // machinery rather than leaving a pending stuff bit or stale run.
            if ($past(rst_n) && $past(soft_rst_pulse || !ctl_tx_arm)) begin
                assert (txs == TXS_IDLE);
                assert (!tx_active);
                assert (!tx_stuff_pend);
                assert (tx_run_cnt == 3'd0);
                assert (!tx_oe);
            end

            // Non-vacuity witnesses.  These require actual stuff insertion in
            // both protocols, both CAN polarities, a trailing USB stuff bit,
            // the largest legal USB run, and reset of a genuinely pending
            // stuff state.
            cover (f_tx_bit_event && (txs == TXS_RUN) && tx_stuff_pend &&
                   (stuff_mode == 2'd1) && (codec == 3'd1));
            cover ((txs == TXS_TSTUFF) && (stuff_mode == 2'd1));
            cover (f_tx_bit_event && (txs == TXS_RUN) && tx_stuff_pend &&
                   (stuff_mode == 2'd2) && !tx_run_val);
            cover (f_tx_bit_event && (txs == TXS_RUN) && tx_stuff_pend &&
                   (stuff_mode == 2'd2) && tx_run_val);
            cover ((stuff_mode == 2'd1) && f_tx_codec01 &&
                   f_tx_last_valid && f_tx_last_bit && (f_tx_run_len == 4'd6));
            cover (soft_rst_pulse && tx_stuff_pend);
        end
    end
`endif

endmodule
`default_nettype wire
