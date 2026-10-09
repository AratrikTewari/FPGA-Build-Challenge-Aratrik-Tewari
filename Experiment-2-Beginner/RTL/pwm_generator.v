`timescale 1ns/1ps

// ============================================================================
// Module Name:   pwm_generator
// Project:       Zynq-7000 Digital Door Lock System (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 SoC (XC7Z020-1CLG400C)
//
// Description:
//   Precision Pulse Width Modulation (PWM) generator potentially driving an external
//   tower/hobby servo motor (e.g., TowerPro SG90 / MG90S) connected via Pmod B.
//   Translates the binary access state (unlock_sig) from the security FSM
//   into standard 50 Hz analog servo control signals.
//
// Control Timing & Angular Mapping (at 125 MHz system clock):
//   - Frame Rate:  50 Hz carrier frequency (20.0 ms frame period = 2,500,000 cycles).
//   - Locked (0°):   1.0 ms pulse width (125,000 cycles, 5.0% duty cycle) -> deadbolt engaged.
//   - Unlocked (90°): 2.0 ms pulse width (250,000 cycles, 10.0% duty cycle) -> deadbolt retracted.
//
// System Context:
//   Directly instantiated by top_module. Operates synchronously within the
//   primary Zynq PS clock domain (FCLK_CLK0). Output servo_pwm routes to
//   package pin W14 (Pmod B, Pin JB1_P) through 3.3V LVCMOS signaling.
// ============================================================================

module pwm_generator #(
    // Base 50 Hz PWM frame timing at 125 MHz: 125,000,000 / 50 = 2,500,000 cycles
    // Requires a 22-bit counter ($2^{22} = 4,194,304 > 2,500,000$)
    parameter PERIOD_CYCLES = 22'd2_500_000,

    // High-time pulse parameters (18-bit width: $2^{18} = 262,144 > 250,000$)
    parameter LOCKED_HIGH   = 18'd125_000, // 1.0 ms high-pulse (0° position)
    parameter UNLOCKED_HIGH = 18'd250_000  // 2.0 ms high-pulse (90° position)
) (
    // Global synchronous control lines (clocked by Zynq PS FCLK_CLK0)
    input  wire clk,
    input  wire rst,

    // Hardware status handshake from door_lock_fsm
    input  wire unlock_sig, // High indicates valid PIN match and active 5s unlock window

    // Physical PWM output line to Pmod B (routed to Pin W14)
    output reg  servo_pwm
);

    // ------------------------------------------------------------------------
    // Internal Registers & Datapath
    // ------------------------------------------------------------------------
    reg [21:0] counter; // Free-running 20 ms timebase generator

    // Dynamically select target pulse high-time based on lock state
    wire [17:0] high_cycles = unlock_sig ? UNLOCKED_HIGH : LOCKED_HIGH;

    // ------------------------------------------------------------------------
    // Synchronous PWM Generation & Counter Logic
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            counter   <= 22'd0;
            servo_pwm <= 1'b0; // Default to zero output during reset
        end else begin
            // 20 ms Period Counter: resets every 2,500,000 cycles
            if (counter >= PERIOD_CYCLES - 1)
                counter <= 22'd0;
            else
                counter <= counter + 22'd1;

            // Generate active-high control pulse at the beginning of each 20 ms window
            // Evaluates to 1'b1 while counter is below the threshold, 1'b0 otherwise
            servo_pwm <= (counter < high_cycles) ? 1'b1 : 1'b0;
        end
    end

endmodule
