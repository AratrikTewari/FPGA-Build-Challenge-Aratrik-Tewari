`timescale 1ns/1ps

// door_lock_fsm.v
// Finite State Machine controlling lock status, passcode verification,
// 5-second automatic relock timing, and tamper lockout.

module door_lock_fsm #(
    parameter UNLOCK_HOLD_CYCLES = 30'd625_000_000 // 5 seconds at 125 MHz
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        key_valid,
    input  wire [3:0]  key_data,

    output reg         unlock_sig,
    output reg  [2:0]  rgb_status, // [2]=Red, [1]=Green, [0]=Blue
    output reg         alarm_sig
);

    // State Encoding
    localparam S_IDLE     = 2'd0;
    localparam S_ENTRY    = 2'd1;
    localparam S_UNLOCKED = 2'd2;
    localparam S_ALARM    = 2'd3;

    reg [1:0]  state;
    reg [15:0] shift_reg;
    reg [2:0]  digit_count;
    reg [1:0]  failed_attempts;
    reg [29:0] unlock_timer; // Sized up to 30 bits for 625,000,000 cycles

    // Predefined 4-digit passcode: 1-2-3-4
    localparam [15:0] CORRECT_PASSCODE = 16'h1234;

    always @(posedge clk) begin
        if (rst) begin
            state           <= S_IDLE;
            shift_reg       <= 16'h0000;
            digit_count     <= 3'd0;
            failed_attempts <= 2'd0;
            unlock_timer    <= 30'd0;
            unlock_sig      <= 1'b0;
            rgb_status      <= 3'b100; // Red (Locked)
            alarm_sig       <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    unlock_sig   <= 1'b0;
                    alarm_sig    <= 1'b0;
                    unlock_timer <= 30'd0;
                    rgb_status   <= 3'b100; // Red

                    if (key_valid) begin
                        shift_reg   <= {12'h000, key_data};
                        digit_count <= 3'd1;
                        state       <= S_ENTRY;
                        rgb_status  <= 3'b001; // Blue (Entering PIN)
                    end
                end

                S_ENTRY: begin
                    rgb_status <= 3'b001; // Blue
                    unlock_sig <= 1'b0;

                    if (key_valid) begin
                        shift_reg <= {shift_reg[11:0], key_data};
                        
                        if (digit_count == 3'd3) begin
                            // Evaluates the full 4-digit combination
                            if ({shift_reg[11:0], key_data} == CORRECT_PASSCODE) begin
                                state           <= S_UNLOCKED;
                                unlock_sig      <= 1'b1;
                                rgb_status      <= 3'b010; // Green (Unlocked)
                                failed_attempts <= 2'd0;
                                unlock_timer    <= 30'd0;
                            end else begin
                                if (failed_attempts >= 2'd2) begin
                                    state     <= S_ALARM;
                                    alarm_sig <= 1'b1;
                                end else begin
                                    failed_attempts <= failed_attempts + 2'd1;
                                    state           <= S_IDLE;
                                    rgb_status      <= 3'b100; // Red
                                end
                            end
                            digit_count <= 3'd0;
                            shift_reg   <= 16'h0000;
                        end else begin
                            digit_count <= digit_count + 3'd1;
                        end
                    end
                end

                S_UNLOCKED: begin
                    unlock_sig <= 1'b1;
                    rgb_status <= 3'b010; // Green

                    if (unlock_timer >= UNLOCK_HOLD_CYCLES) begin
                        unlock_timer <= 30'd0;
                        unlock_sig   <= 1'b0;
                        state        <= S_IDLE;
                        rgb_status   <= 3'b100; // Return to Red
                    end else begin
                        unlock_timer <= unlock_timer + 30'd1;
                    end
                end

                S_ALARM: begin
                    unlock_sig <= 1'b0;
                    alarm_sig  <= 1'b1;
                    // Latch in alarm state until hardware reset (btn_rst)
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule