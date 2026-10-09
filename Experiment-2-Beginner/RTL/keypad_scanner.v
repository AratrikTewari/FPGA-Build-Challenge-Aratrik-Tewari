`timescale 1ns/1ps

// ============================================================================
// Module Name:   keypad_scanner
// Project:       Zynq-7000 Digital Door Lock System (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 SoC (XC7Z020-1CLG400C)
//
// Description:
//   Hardware scanning controller for a standard 4x4 matrix keypad connected
//   via Pmod A. Drives active-low walking-zero patterns across row outputs
//   and monitors column return lines to detect and decode switch closures.
//
// System Context & Interfacing:
//   - Input Conditioning: Instantiates 4 parallel debouncer instances to 
//     filter mechanical bounce on noisy column inputs before decoding.
//   - Freeze Mechanism: Immediately pauses row scanning upon switch contact
//     detection (raw or debounced) to latch the stable row/column intersection.
//   - FSM Handshake: Emits an exact single-cycle strobe (key_valid) and the
//     associated 4-bit nibble (key_data) directly into top_module's key
//     arbitration multiplexer, matching the software injection interface.
//
// Timing Specifications (at 125 MHz system clock):
//   - Row Scan Rate: 1 kHz (125,000 clock cycles per row step).
//   - Column Debounce Filter: 10 ms (1,250,000 clock cycles rejection window).
// ============================================================================

module keypad_scanner #(
    parameter integer SCAN_DIVIDER     = 125_000,   // 1.0 ms row step period @ 125 MHz
    parameter integer DEBOUNCE_CYCLES  = 1_250_000  // 10.0 ms switch debounce filter @ 125 MHz
) (
    // Global synchronous control lines (clocked by Zynq PS FCLK_CLK0)
    input  wire        clk,
    input  wire        rst,

    // Physical Pmod A hardware interface
    output reg  [3:0]  row,        // Active-low walking row drive outputs (JA1_P..JA4_P)
    input  wire [3:0]  col,        // Active-low column inputs with internal pull-ups (JA1_N..JA4_N)

    // Synchronous status bus to door_lock_fsm / top_module multiplexer
    output reg         key_valid,  // Single-cycle active-high pulse asserting valid key event
    output reg  [3:0]  key_data    // Decoded 4-bit hexadecimal key value (0x0 to 0xF)
);

    // ------------------------------------------------------------------------
    // Column Line Conditioning: Parallel Switch Debouncing
    // ------------------------------------------------------------------------
    // Mechanical key contacts bounce for several milliseconds upon closure.
    // Each column pin is filtered through an independent integrator/counter.
    wire [3:0] col_clean;
    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : DEBOUNCE_COLS
            debouncer #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) u_deb (
                .clk       (clk),
                .rst       (rst),
                .noisy_in  (col[i]),
                .clean_out (col_clean[i])
            );
        end
    endgenerate

    // ------------------------------------------------------------------------
    // Scan Preservation & Hold Logic
    // ------------------------------------------------------------------------
    // If any column line pulls LOW (contact made), halt the walking row counter
    // to keep the active row steady while the debouncer settles the column data.
    wire key_detected = (col != 4'b1111) || (col_clean != 4'b1111);

    // ------------------------------------------------------------------------
    // Scanning Timebase Generator (1 kHz Scan Tick)
    // ------------------------------------------------------------------------
    reg [16:0] div_cnt;
    reg        scan_tick;

    always @(posedge clk) begin
        if (rst) begin
            div_cnt   <= 17'd0;
            scan_tick <= 1'b0;
        end else if (!key_detected) begin
            // Free-run the scan counter only while no keys are engaged
            if (div_cnt >= SCAN_DIVIDER - 1) begin
                div_cnt   <= 17'd0;
                scan_tick <= 1'b1; // Trigger next row transition
            end else begin
                div_cnt   <= div_cnt + 17'd1;
                scan_tick <= 1'b0;
            end
        end else begin
            // Freeze scanning counter during key closure and debounce integration
            div_cnt   <= 17'd0;
            scan_tick <= 1'b0;
        end
    end

    // ------------------------------------------------------------------------
    // Walking-Zero Row Sequencer
    // ------------------------------------------------------------------------
    // Sweeps an active-low '0' across the matrix rows sequentially.
    // Internal pull-ups hold columns at '1' until a pressed switch shorts row to col.
    always @(posedge clk) begin
        if (rst) begin
            row <= 4'b1110; // Default to asserting Row 0
        end else if (scan_tick) begin
            case (row)
                4'b1110: row <= 4'b1101; // Advance from Row 0 to Row 1
                4'b1101: row <= 4'b1011; // Advance from Row 1 to Row 2
                4'b1011: row <= 4'b0111; // Advance from Row 2 to Row 3
                4'b0111: row <= 4'b1110; // Wrap back to Row 0
                default: row <= 4'b1110;
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // Matrix Coordinate Decoder (Row x Clean Column -> Hex Nibble)
    // ------------------------------------------------------------------------
    // Maps coordinate intersections to numerical digit values. Unused keys or
    // unasserted combinations evaluate to 4'hF (null/unmapped).
    reg [3:0] decoded;

    always @(*) begin
        case ({row, col_clean})
            // Row 0 Active (4'b1110): Keys '1', '2', '3', 'A'
            {4'b1110, 4'b1110}: decoded = 4'h1;
            {4'b1110, 4'b1101}: decoded = 4'h2;
            {4'b1110, 4'b1011}: decoded = 4'h3;

            // Row 1 Active (4'b1101): Keys '4', '5', '6', 'B'
            {4'b1101, 4'b1110}: decoded = 4'h4;
            {4'b1101, 4'b1101}: decoded = 4'h5;
            {4'b1101, 4'b1011}: decoded = 4'h6;

            // Row 2 Active (4'b1011): Keys '7', '8', '9', 'C'
            {4'b1011, 4'b1110}: decoded = 4'h7;
            {4'b1011, 4'b1101}: decoded = 4'h8;
            {4'b1011, 4'b1011}: decoded = 4'h9;

            // Row 3 Active (4'b0111): Keys '*', '0', '#', 'D'
            {4'b0111, 4'b1101}: decoded = 4'h0;

            // Default safe state for multiple presses, open lines, or unmapped keys
            default:            decoded = 4'hF;
        endcase
    end

    // ------------------------------------------------------------------------
    // Strobe Generator & Keystroke Edge Detector
    // ------------------------------------------------------------------------
    // Emits key_valid for EXACTLY ONE 8 ns clock cycle on the rising edge of a
    // debounced press. Prevents multi-cycle oversampling by the security FSM.
    wire debounced_press = (col_clean != 4'b1111);
    reg  prev_pressed;

    always @(posedge clk) begin
        if (rst) begin
            prev_pressed <= 1'b0;
            key_valid    <= 1'b0;
            key_data     <= 4'h0;
        end else begin
            key_valid <= 1'b0; // Default deassertion: guarantees single-pulse behavior

            // Trigger on 0 -> 1 edge of debounced press condition with valid key decoded
            if (debounced_press && !prev_pressed && (decoded != 4'hF)) begin
                key_valid <= 1'b1;
                key_data  <= decoded;
            end

            prev_pressed <= debounced_press;
        end
    end

endmodule
