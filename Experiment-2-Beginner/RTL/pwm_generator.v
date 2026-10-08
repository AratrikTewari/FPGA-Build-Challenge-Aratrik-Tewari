`timescale 1ns/1ps

// pwm_generator.v
// Generates a 50Hz PWM signal for an SG90 servo.
// clk = 125MHz -> 20ms period = 2,500,000 cycles.
// Locked (0deg):   1ms pulse = 125,000 cycles
// Unlocked (90deg): 2ms pulse = 250,000 cycles

module pwm_generator #(
    parameter PERIOD_CYCLES = 22'd2_500_000,
    parameter LOCKED_HIGH   = 18'd125_000,
    parameter UNLOCKED_HIGH = 18'd250_000
) (
    input  wire clk,
    input  wire rst,
    input  wire unlock_sig,
    output reg  servo_pwm
);

    reg [21:0] counter;
    wire [17:0] high_cycles = unlock_sig ? UNLOCKED_HIGH : LOCKED_HIGH;

    always @(posedge clk) begin
        if (rst) begin
            counter   <= 22'd0;
            servo_pwm <= 1'b0;
        end else begin
            if (counter >= PERIOD_CYCLES - 1)
                counter <= 22'd0;
            else
                counter <= counter + 22'd1;

            servo_pwm <= (counter < high_cycles) ? 1'b1 : 1'b0;
        end
    end

endmodule
