`timescale 1ns/1ps

// ============================================================================
// Module Name:   top_module
// Project:       Zynq-7000 Digital Door Lock System (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 SoC (XC7Z020-1CLG400C)
//
// Description:
//   Top-level hardware integration and arbitration module for the digital
//   door lock architecture. Integrates the Zynq Processing System (PS) with
//   Programmable Logic (PL) resources, mediating dual-source authentication
//   inputs (physical 4x4 matrix keypad vs. AXI GPIO software injection),
//   driving visual feedback, and controlling servo motor actuation.
//
// Architecture & Inter-Module Hierarchy:
//   - top_module
//       ├── keypad_scanner (u_keypad)   : 1 kHz matrix scan with contact debounce
//       │     └── debouncer             : 4x parallel column noise rejection filters
//       ├── door_lock_fsm  (u_fsm)      : Core 4-digit security state machine
//       └── pwm_generator  (u_pwm)      : 50 Hz PWM servo deadbolt controller
//
// Key Engineering Features:
//   1. Asynchronous Reset Conditioning: Double-register flip-flop synchronizer
//      protects the 125 MHz clock domain from metastability on BTN0 release.
//   2. PS-to-PL Strobe Conditioning: 3-stage shift-register synchronizer and
//      rising-edge detector converts millisecond-scale AXI software writes into
//      an exact 1-clock-cycle pulse, preventing multi-cycle FSM oversampling.
//   3. Hardware Multiplexing: Arbitrates between physical matrix scanning and
//      Jupyter/Python software key injection without bus contention.
//   4. Diagnostics & Telemetry: Aggregates real-time PL security flags and
//      LED states into an AXI GPIO readback bus for live OS monitoring.
// ============================================================================

module top_module #(
    // Hold interval parameter passed to FSM: 5.0 seconds at 125 MHz system clock
    parameter UNLOCK_HOLD_CYCLES = 30'd625_000_000
) (
    // Global Clock & Reset
    input  wire        clk,            // 125 MHz fabric clock sourced from Zynq PS (FCLK_CLK0)
    input  wire        btn_rst,        // Asynchronous active-high manual reset (BTN0, Pin D19)

    // Physical Hardware Interface (Pmod A - Keypad Matrix)
    output wire [3:0]  kp_row,         // Walking-zero active-low row drive pins (JA1_P..JA4_P)
    input  wire [3:0]  kp_col,         // Active-low column return pins with pull-ups (JA1_N..JA4_N)

    // Physical Actuator Interface (Pmod B - Servo Motor)
    output wire        servo_pwm,      // 50 Hz PWM control output for SG90 servo (Pin W14)

    // Physical Visual Interface (Onboard RGB LED LD4)
    output wire        led_r,          // Red channel cathode driver (Pin N15)
    output wire        led_g,          // Green channel cathode driver (Pin G17)
    output wire        led_b,          // Blue channel cathode driver (Pin L15)

    // Zynq-7000 PS AXI4-Lite GPIO Bridges
    // axi_gpio_1 [Output channel]: Software keystroke injection register
    //   ps_key_inject[4]   = Strobe enable (sys_key_valid trigger)
    //   ps_key_inject[3:0] = Injected hexadecimal key code (0x0 to 0xF)
    input  wire [4:0]  ps_key_inject,

    // axi_gpio_0 [Input channel]: Real-time hardware status and telemetry register
    //   ps_status_out[3]   = Alarm tamper lockout active flag
    //   ps_status_out[2:0] = Visual state indicators ([2]=Red, [1]=Green, [0]=Blue)
    output wire [3:0]  ps_status_out
);

    // ------------------------------------------------------------------------
    // Reset Domain Synchronization
    // ------------------------------------------------------------------------
    // Conditions the raw asynchronous BTN0 push-button input through a 2-stage
    // synchronizer to prevent metastability hazards across internal FSM registers.
    reg rst_sync0, rst_sync1;
    always @(posedge clk) begin
        rst_sync0 <= btn_rst;
        rst_sync1 <= rst_sync0;
    end
    wire rst = rst_sync1;

    // ------------------------------------------------------------------------
    // PS Software Injection: Synchronizer, Latch & 1-Cycle Edge Detector
    // ------------------------------------------------------------------------
    // AXI bus writes from Python/Linux hold the valid line high for thousands
    // of 125 MHz clock cycles. This circuit detects the 0 -> 1 software transition
    // and produces a clean, single-cycle pulse (ps_key_pulse) while safely latching
    // the target key nibble.
    reg [2:0] ps_valid_sync;
    reg [3:0] ps_data_latched;

    always @(posedge clk) begin
        if (rst) begin
            ps_valid_sync   <= 3'b000;
            ps_data_latched <= 4'h0;
        end else begin
            // 3-stage shift register for metastability hardening and edge sampling
            ps_valid_sync <= {ps_valid_sync[1:0], ps_key_inject[4]};

            // Latch 4-bit data on the arrival of the software injection request
            if (ps_key_inject[4]) begin
                ps_data_latched <= ps_key_inject[3:0];
            end
        end
    end

    // Single 8 ns clock pulse asserted exclusively on the rising edge of software valid
    wire ps_key_pulse = (ps_valid_sync[1] && !ps_valid_sync[2]);

    // ------------------------------------------------------------------------
    // Subsystem Interconnect Signals
    // ------------------------------------------------------------------------
    wire       hw_key_valid;  // Single-cycle strobe from physical matrix scanner
    wire [3:0] hw_key_data;   // Decoded nibble from physical matrix scanner
    wire       unlock_sig;    // Binary access granted flag from door_lock_fsm
    wire [2:0] rgb_status;    // Raw FSM state color vector: [2]=Red, [1]=Green, [0]=Blue
    wire       alarm_sig;     // Persistent security lockout flag from door_lock_fsm

    // ------------------------------------------------------------------------
    // Peripheral Instance: Physical 4x4 Matrix Keypad Scanner
    // ------------------------------------------------------------------------
    // Continuously scans Pmod A row/column lines, filters contact chatter,
    // and outputs single-cycle valid strobes upon switch closure.
    keypad_scanner u_keypad (
        .clk       (clk),
        .rst       (rst),
        .row       (kp_row),
        .col       (kp_col),
        .key_valid (hw_key_valid),
        .key_data  (hw_key_data)
    );

    // ------------------------------------------------------------------------
    // Keystroke Arbitration & Multiplexing
    // ------------------------------------------------------------------------
    // Merges physical matrix scanner strobes with software AXI injection pulses.
    // Software pulse takes priority
