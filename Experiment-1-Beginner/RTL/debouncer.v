`timescale 1ns/1ps

// ============================================================================
// Module:        debouncer
// Project:       Multi-Floor FPGA Elevator Controller (PYNQ-Z2)
// Description:   Digital input conditioner providing metastability protection
//                and counter-based switch contact debouncing.
//
// System Context:
//   Mechanical inputs on the PYNQ-Z2 board (tactile push buttons and slide
//   switches) exhibit physical contact bounce lasting several milliseconds.
//   This module conditions asynchronous manual inputs (BTN0-BTN3, SW0-SW1)
//   before feeding the core finite state machine (elevator_fsm) and cabin floor
//   latch in elevator_top, preventing false state transitions or spurious multi-triggers.
//
// Timing Derivation:
//   System Clock: 125 MHz (Period = 8 ns)
//   Default DEBOUNCE_CYCLES = 1,250,000 cycles
//   Debounce Window = 1,250,000 * 8 ns = 10 ms (sufficient to reject standard mechanical bounce)
// ============================================================================

module debouncer #(
    parameter integer DEBOUNCE_CYCLES = 1_250_000
) (
    input  wire clk,        // 125 MHz board master clock
    input  wire rst,        // Synchronous system reset
    input  wire noisy_in,   // Raw asynchronous mechanical input from board pin
    output reg  clean_out   // Glitch-free, synchronized output driven to core logic
);

    // 21-bit counter supports counting up to 2,097,151 (covers the 1.25M cycle threshold)
    reg [20:0] counter;
    
    // Two-stage flip-flop synchronizer to safely mitigate metastability from asynchronous inputs
    reg        sync_0, sync_1;

    always @(posedge clk) begin
        if (rst) begin
            // Reset synchronizer chain and filter counter
            sync_0    <= 1'b0;
            sync_1    <= 1'b0;
            counter   <= 20'd0;
            
            // Default idle state on reset
            clean_out <= 1'b1;
        end else begin
            // Double-register incoming asynchronous signal across clock boundaries
            sync_0 <= noisy_in;
            sync_1 <= sync_0;

            // Stability check: Compare synchronized input against current filtered state
            if (sync_1 == clean_out) begin
                // Input matches current stable state; reset counter to filter momentary spikes
                counter <= 20'd0;
            end else begin
                // Input has transitioned; require it to remain steady for the full duration
                counter <= counter + 21'd1;
                
                if (counter >= DEBOUNCE_CYCLES) begin
                    // Sustained stable state validated; commit new level to output
                    clean_out <= sync_1;
                    counter   <= 20'd0;
                end
            end
        end
    end

endmodule
