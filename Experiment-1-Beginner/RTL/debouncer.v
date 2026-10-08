`timescale 1ns/1ps

// debouncer.v
// Simple counter-based debouncer. Output changes only after the input has
// been stable for DEBOUNCE_CYCLES consecutive clock cycles.
// At 125MHz, DEBOUNCE_CYCLES = 1_250_000 gives ~10ms debounce.

module debouncer #(
    parameter integer DEBOUNCE_CYCLES = 1_250_000
) (
    input  wire clk,
    input  wire rst,
    input  wire noisy_in,
    output reg  clean_out
);

    // 21 bits represent the default 1,250,000-cycle debounce interval.
    reg [20:0] counter;
    reg        sync_0, sync_1; // 2-flop synchronizer for the async input

    always @(posedge clk) begin
        if (rst) begin
            sync_0    <= 1'b0;
            sync_1    <= 1'b0;
            counter   <= 20'd0;
            // Keypad column lines are pulled high when no key is pressed.
            clean_out <= 1'b1;
        end else begin
            sync_0 <= noisy_in;
            sync_1 <= sync_0;

            if (sync_1 == clean_out) begin
                counter <= 20'd0;
            end else begin
                counter <= counter + 21'd1;
                if (counter >= DEBOUNCE_CYCLES) begin
                    clean_out <= sync_1;
                    counter   <= 20'd0;
                end
            end
        end
    end

endmodule
