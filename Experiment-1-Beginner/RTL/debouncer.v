`timescale 1ns / 1ps

// ============================================================================
// Module:        debouncer
// Project:       Multi-Floor FPGA Elevator Controller (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 (XC7Z020-1CLG400C)
//
// System Context:
//   Physical human-interface peripherals (push buttons BTN0-BTN3 and slide
//   switches SW0-SW1) produce mechanical contact chatter lasting several 
//   milliseconds. If sampled raw at the 125 MHz system clock, a single physical
//   press registers as multiple rapid transitions, causing spurious floor calls
//   and erratic state progression in the elevator controller.
//
//   This module acts as an inline hardware low-pass filter. It monitors the 
//   raw input line and asserts a synchronized, glitch-free output signal only 
//   after the mechanical contact has remained continuously stable across a full
//   10 ms sampling window.
//
// Hardware Timing Derivation:
//   Board Master Clock:  125 MHz (T_clk = 8.0 ns)
//   Stability Duration:  10 ms (10,000,000 ns)
//   Required Cycles:     10,000,000 ns / 8.0 ns = 1,250,000 clock cycles
//   Counter Width:       ceil(log2(1,250,000)) = 21 bits (Max capacity = 2,097,151)
// ============================================================================

module debouncer (
    input  wire clk,        // 125 MHz board clock from pin H16
    input  wire rst,        // Asynchronous reset (BTN0 / system-level reset)
    input  wire btn_in,     // Raw asynchronous contact line from push button/switch
    output reg  btn_out     // Debounced, clock-synchronized output to top-level logic
);

    // 1,250,000 clock cycles at 125 MHz provides the 10 ms stability filter window
    parameter DEBOUNCE_LIMIT = 1250000; 

    // 21-bit register accurately accommodates counts up to DEBOUNCE_LIMIT
    reg [20:0] counter;

    // Flip-flop pipeline stage for detecting signal transitions and stability
    reg        q_reg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Hardware reset vector: restore filter to idle quiescent state
            counter <= 21'd0;
            q_reg   <= 1'b0;
            btn_out <= 1'b0;
        end else begin
            // Sample input to detect dynamic voltage fluctuations across adjacent clock edges
            q_reg <= btn_in;
            
            // Dynamic edge evaluation:
            // If the incoming line does not match the previous cycle sample, contact
            // bounce or intentional switching is occurring; reset the stability timer.
            if (q_reg != btn_in) begin
                counter <= 21'd0; 
            end 
            // Signal remains quiescent; accumulate consecutive stable clock cycles
            else if (counter < DEBOUNCE_LIMIT) begin
                counter <= counter + 21'd1;
            end 
            // Stability qualification complete: signal has stayed steady for 10 ms
            else begin
                btn_out <= q_reg;
            end
        end
    end

endmodule
