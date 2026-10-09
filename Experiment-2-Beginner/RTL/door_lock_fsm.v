`timescale 1ns/1ps

// ============================================================================
// Module Name:   door_lock_fsm
// Project:       Zynq-7000 Digital Door Lock System (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 SoC (XC7Z020-1CLG400C)
//
// Description:
//   Core security Finite State Machine (FSM) mediating access control.
//   Accepts 4-bit synchronous keystrokes (sourced either from the matrix
//   keypad scanner or the Zynq PS AXI GPIO injection bridge), verifies 4-digit
//   PIN sequences via a serial-in shift register, drives system status
//   indicators, and coordinates hardware lock actuation.
//
// Key Specifications & Safety Features:
//   - Timing: 5.0-second auto-relock window (parameterized for 125 MHz system clock).
//   - Security: 3-strike brute-force lockout latching into a persistent alarm state.
//   - Integration: Direct handshakes to pwm_generator (servo actuation) and
//     top_module status aggregation (AXI GPIO telemetry and LD4 RGB driver).
// ============================================================================

module door_lock_fsm #(
    // 5.0 seconds hold interval calculated at 125 MHz: (125,000,000 * 5 = 625,000,000 cycles)
    parameter UNLOCK_HOLD_CYCLES = 30'd625_000_000
) (
    // Synchronous clock and reset lines (rst synchronized at top-level)
    input  wire        clk,
    input  wire        rst,

    // Arbitrated single-cycle strobe and keystroke data from top_module
    input  wire        key_valid,
    input  wire [3:0]  key_data,

    // PL Subsystem Control Flags
    output reg         unlock_sig,  // High enables PWM servo rotation to 90 degrees
    output reg  [2:0]  rgb_status,  // One-hot state feedback: [2]=Red, [1]=Green, [0]=Blue
    output reg         alarm_sig    // Tamper alert: drives alarm blinker and PS telemetry
);

    // ------------------------------------------------------------------------
    // FSM State Encoding (Sequential Binary)
    // ------------------------------------------------------------------------
    localparam S_IDLE     = 2'd0;  // Quiescent state: door securely locked
    localparam S_ENTRY    = 2'd1;  // Digit collection: serial shift register active
    localparam S_UNLOCKED = 2'd2;  // Passcode matched: servo open, auto-lock timer running
    localparam S_ALARM    = 2'd3;  // Security lockout: max failed attempts reached

    // Internal Registers
    reg [1:0]  state;
    reg [15:0] shift_reg;        // 4 x 4-bit nibbles storing the active sequence
    reg [2:0]  digit_count;      // Keystroke tracker (0 to 4 digits)
    reg [1:0]  failed_attempts;  // Tracks invalid attempts (trips alarm on 3rd failure)
    
    // Timer register sized to 30 bits ($2^{30} = 1,073,741,824 > 625,000,000)
    // Prevents integer rollover before reaching the full 5.0s unlock duration
    reg [29:0] unlock_timer;

    // Hardcoded 4-digit system access PIN: 1-2-3-4
    localparam [15:0] CORRECT_PASSCODE = 16'h1234;

    // ------------------------------------------------------------------------
    // Synchronous FSM State Transition & Datapath Logic
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            // Hardware reset state (triggered via debounced/synchronized BTN0)
            state           <= S_IDLE;
            shift_reg       <= 16'h0000;
            digit_count     <= 3'd0;
            failed_attempts <= 2'd0;
            unlock_timer    <= 30'd0;
            unlock_sig      <=
