`timescale 1ns / 1ps

// ============================================================================
// Module:        elevator_fsm
// Project:       Multi-Floor FPGA Elevator Controller (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 (XC7Z020-1CLG400C)
//
// System Context:
//   This module acts as the core algorithmic engine and motion supervisor for
//   the elevator system. Located downstream of the input debouncers and cabin
//   floor-latching registers in 'elevator_top', this Finite State Machine
//   continuously coordinates motion decisions (hoist up, hoist down, or rest)
//   and passenger safety interlocks (door dwell timing and beam obstruction).
//
// Architectural Principles:
//   - Classic Two-Process FSM: Synchronous state register & dwell timer clocked
//     at 125 MHz paired with a pure combinational next-state/output decoder.
//   - Latch-Free Default Assignment: All output drivers are unconditionally
//     defaulted to zero at the head of the combinational block to avoid inferred
//     transparent latches during logic synthesis.
//   - Bound Check Transition: Uses directional inequality checks (>= and <=)
//     rather than strict equality (==) during motion states to guarantee 
//     fail-safe arrival transitions even during dynamic target changes.
//   - Passenger Safety Interlock: The door dwell interval enforces both a
//     hardware real-time minimum delay (DOOR_LIMIT) and an active beam clearance
//     condition (!door_sensor) before permitting state re-entry to IDLE.
//
// Hardware Timing Derivation:
//   System Clock:     125 MHz (Period T_clk = 8.0 ns)
//   Door Dwell Limit: 2.0 seconds = 2,000,000,000 ns / 8.0 ns = 250,000,000 cycles
//   Timer Register:   ceil(log2(250,000,000)) = 28 bits (Max capacity = 268,435,455)
// ============================================================================

module elevator_fsm #(
    parameter DOOR_LIMIT = 250000000 // 2-second dwell interval at 125 MHz
)(
    input  wire       clk,           // 125 MHz system master clock
    input  wire       reset,         // Debounced asynchronous master reset (BTN0)
    input  wire [1:0] target_floor,  // Binary requested floor index [0..3] from SW0/SW1
    input  wire [1:0] current_floor, // Latched cabin floor position [0..3] from BTN1/BTN2
    input  wire       door_sensor,   // Optical safety sensor / obstacle input (BTN3)
    output reg        motor_up,      // Hoist motor UP drive signal (mapped to LD4 Red)
    output reg        motor_down,    // Hoist motor DOWN drive signal (mapped to LD4 Blue)
    output reg        door_open      // Door actuator open indicator (mapped to LD5 Green)
);

    // ------------------------------------------------------------------------
    // FSM State Encoding (Gray/Sequential 2-bit state vector)
    // ------------------------------------------------------------------------
    localparam IDLE            = 2'b00; // Resting at floor; awaiting service call
    localparam MOVE_UP         = 2'b01; // Motor driving cabin upward toward target
    localparam MOVE_DOWN       = 2'b10; // Motor driving cabin downward toward target
    localparam DOOR_OPEN_STATE = 2'b11; // Cabin stationary; passenger transfer phase

    reg [1:0]  state, next_state;
    reg [27:0] door_timer; // Hardware real-time cycle accumulator for door open duration

    // ------------------------------------------------------------------------
    // Process 1: Synchronous State Register & Dwell Timer
    // ------------------------------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state      <= IDLE;
            door_timer <= 28'd0;
        end else begin
            state <= next_state;
            
            // Real-time door timer advances strictly while doors remain open;
            // synchronously cleared in all other operational states.
            if (state == DOOR_OPEN_STATE)
                door_timer <= door_timer + 1'b1;
            else
                door_timer <= 28'd0;
        end
    end

    // ------------------------------------------------------------------------
    // Process 2: Combinational Next-State Evaluation & Output Generation
    // ------------------------------------------------------------------------
    always @(*) begin
        // Safe default assignments: Prevent latch inference and guarantee motor cutoff
        motor_up   = 1'b0; 
        motor_down = 1'b0; 
        door_open  = 1'b0;
        next_state = state;

        case (state)
            // ----------------------------------------------------------------
            // State: IDLE
            // Floor comparator continuously arbitrates directional dispatch.
            // ----------------------------------------------------------------
            IDLE: begin
                if (target_floor > current_floor)
                    next_state = MOVE_UP;
                else if (target_floor < current_floor)
                    next_state = MOVE_DOWN;
                else
                    next_state = IDLE;
            end

            // ----------------------------------------------------------------
            // State: MOVE_UP
            // Energize hoist UP coil until arrival threshold is verified.
            // ----------------------------------------------------------------
            MOVE_UP: begin
                motor_up = 1'b1;
                // Transition to door dwell once cabin reaches or crosses target
                if (current_floor >= target_floor)
                    next_state = DOOR_OPEN_STATE;
            end

            // ----------------------------------------------------------------
            // State: MOVE_DOWN
            // Energize hoist DOWN coil until arrival threshold is verified.
            // ----------------------------------------------------------------
            MOVE_DOWN: begin
                motor_down = 1'b1;
                // Transition to door dwell once cabin reaches or crosses target
                if (current_floor <= target_floor)
                    next_state = DOOR_OPEN_STATE;
            end

            // ----------------------------------------------------------------
            // State: DOOR_OPEN_STATE
            // Assert door open signal. Hold position until both the minimum
            // dwell timer expires AND the active optical obstruction is clear.
            // ----------------------------------------------------------------
            DOOR_OPEN_STATE: begin
                door_open = 1'b1;
                if ((door_timer >= DOOR_LIMIT) && !door_sensor) begin
                    next_state = IDLE;
                end
            end

            // Safe recovery vector in case of radiation-induced soft errors
            default: next_state = IDLE;
        endcase
    end

endmodule
