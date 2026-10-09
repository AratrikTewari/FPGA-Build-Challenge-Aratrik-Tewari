`timescale 1ns / 1ps

// ============================================================================
// Module:        elevator_top
// Project:       Multi-Floor FPGA Elevator Controller (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 SoC (XC7Z020-1CLG400C)
//
// System Context & Top-Level Integration:
//   This module serves as the primary hardware integration boundary and top-level
//   RTL wrapper for the FPGA elevator control system. It interfaces directly with
//   the physical peripherals of the PYNQ-Z2 development board:
//
//     - Inputs:
//         * clk           : 125 MHz system oscillator (Package Pin H16)
//         * rst_btn       : Global asynchronous reset push button (BTN0, Pin D19)
//         * target_sw     : 2-bit binary target floor request (SW0: Pin M20, SW1: Pin M19)
//         * current_fl_sw : 2-bit cabin floor arrival simulator (BTN1: Pin D20, BTN2: Pin L20)
//         * door_sens_btn : Optical safety beam / obstacle sensor (BTN3, Pin L19)
//
//     - Outputs:
//         * motor_up_led   : Hoist UP active indicator (LD4 Red, Pin N15)
//         * motor_down_led : Hoist DOWN active indicator (LD4 Blue, Pin L15)
//         * door_open_led  : Door open dwell indicator (LD5 Green, Pin L14)
//
// Architectural Hierarchy:
//   1. Input Conditioning Subsystem:
//      - Multi-channel instantiation of parameterizable counter-based debouncers
//        (`debouncer.v`) provides metastability mitigation and a 10 ms stability
//        window across all user-actuated push buttons.
//
//   2. Cabin Position Latching Register:
//      - Physical push buttons are momentary. An active-holding synchronous register
//        (`latched_current_floor`) maintains persistent cabin elevation coordinates
//        upon button release, decoupling transient operator presses from real-time FSM tracking.
//
//   3. Finite State Machine Supervisor:
//      - Connects conditioned inputs and the latched cabin position into the core
//        motion and safety state machine ('elevator_fsm.v').
// ============================================================================

module elevator_top #(
    parameter DB_LIMIT   = 1250000,    // 10 ms debounce interval at 125 MHz clock
    parameter DOOR_LIMIT = 250000000   // 2.0-second door dwell period at 125 MHz clock
)(
    input  wire       clk,             // 125 MHz on-board master clock source (Pin H16)
    input  wire       rst_btn,         // Raw reset push button (BTN0)
    input  wire [1:0] target_sw,       // Target floor binary select switches (SW1=MSB, SW0=LSB)
    input  wire [1:0] current_fl_sw,   // Floor sensor simulation buttons (BTN2=MSB, BTN1=LSB)
    input  wire       door_sens_btn,   // Door safety beam obstruction push button (BTN3)
    output wire       motor_up_led,    // Motor UP drive status indicator (LD4 Red)
    output wire       motor_down_led,  // Motor DOWN drive status indicator (LD4 Blue)
    output wire       door_open_led    // Door open actuator status indicator (LD5 Green)
);

    // ------------------------------------------------------------------------
    // Internal Interconnect Signals
    // ------------------------------------------------------------------------
    wire       clean_rst;              // Glitch-free master reset from BTN0 debouncer
    wire       clean_door;             // Conditioned optical obstruction sensor from BTN3
    wire [1:0] clean_btn_floor;        // Conditioned floor arrival pulses from BTN1 and BTN2

    // Persistent cabin floor register: Initialized to Ground Floor (Floor 0) on FPGA power-up
    reg  [1:0] latched_current_floor = 2'b00;

    // ------------------------------------------------------------------------
    // Subsystem 1: Input Debounce Filtering
    // Filters mechanical contact bounce and synchronizes asynchronous inputs to clk.
    // ------------------------------------------------------------------------

    // Debouncer for System Master Reset (BTN0)
    // Note: Local filter reset is tied low so BTN0 can self-clear from power-on
    debouncer #( .DEBOUNCE_LIMIT(DB_LIMIT) ) rst_db (
        .clk     (clk), 
        .rst     (1'b0), 
        .btn_in  (rst_btn), 
        .btn_out (clean_rst)
    );

    // Debouncer for Optical Door Clearance Sensor (BTN3)
    debouncer #( .DEBOUNCE_LIMIT(DB_LIMIT) ) door_db (
        .clk     (clk), 
        .rst     (clean_rst), 
        .btn_in  (door_sens_btn), 
        .btn_out (clean_door)
    );

    // Debouncer for Floor Arrival Sensor Bit 0 (BTN1 -> Floor 1)
    debouncer #( .DEBOUNCE_LIMIT(DB_LIMIT) ) curr_fl_db0 (
        .clk     (clk), 
        .rst     (clean_rst), 
        .btn_in  (current_fl_sw[0]), 
        .btn_out (clean_btn_floor[0])
    );

    // Debouncer for Floor Arrival Sensor Bit 1 (BTN2 -> Floor 2)
    debouncer #( .DEBOUNCE_LIMIT(DB_LIMIT) ) curr_fl_db1 (
        .clk     (clk), 
        .rst     (clean_rst), 
        .btn_in  (current_fl_sw[1]), 
        .btn_out (clean_btn_floor[1])
    );

    // ------------------------------------------------------------------------
    // Subsystem 2: Cabin Floor Position Memory & State Latch
    // 
    // Mechanical buttons return to logic 0 when released. To reflect realistic elevator
    // shaft limit switches, this block captures the moment a floor switch is triggered
    // and holds that floor index steady until a new floor arrival is detected.
    // ------------------------------------------------------------------------
    always @(posedge clk or posedge clean_rst) begin
        if (clean_rst) begin
            latched_current_floor <= 2'b00; // Reset condition: Cabin defaults to Floor 0
        end else begin
            case (clean_btn_floor)
                2'b01:   latched_current_floor <= 2'b01; // BTN1 pressed: Arrived at Floor 1
                2'b10:   latched_current_floor <= 2'b10; // BTN2 pressed: Arrived at Floor 2
                2'b11:   latched_current_floor <= 2'b11; // Concurrent press: Arrived at Floor 3
                default: latched_current_floor <= latched_current_floor; // Retain current floor
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // Subsystem 3: Core Finite State Machine (FSM) Controller
    // Arbitrates motion requests, floor matching, and door interlock sequences.
    // ------------------------------------------------------------------------
    elevator_fsm #( .DOOR_LIMIT(DOOR_LIMIT) ) u_fsm (
        .clk           (clk),
        .reset         (clean_rst),             
        .target_floor  (target_sw),              // Direct user call from slide switches
        .current_floor (latched_current_floor),  // Persistent latched position
        .door_sensor   (clean_door),             // Conditioned passenger safety sensor
        .motor_up      (motor_up_led),           // Drives LD4 Red
        .motor_down    (motor_down_led),         // Drives LD4 Blue
        .door_open     (door_open_led)           // Drives LD5 Green
    );

endmodule
