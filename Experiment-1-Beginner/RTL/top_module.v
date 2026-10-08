`timescale 1ns/1ps

// top_module.v
// Top-level integration for the Digital Door Lock on PYNQ-Z2.
// Integrates physical hardware, 1-cycle edge-pulsed PS injection, and active-high RGB mappings.

module top_module #(
    parameter UNLOCK_HOLD_CYCLES = 30'd625_000_000 // 5s @ 125MHz default
) (
    input  wire clk,          
    input  wire btn_rst,      

    // Physical Hardware I/O
    output wire [3:0] kp_row,
    input  wire [3:0] kp_col,
    output wire servo_pwm,
    output wire led_r,
    output wire led_g,
    output wire led_b,

    // Zynq PS AXI GPIO Ports
    input  wire [4:0] ps_key_inject, // [4]: python_key_valid, [3:0]: python_key_data
    output wire [3:0] ps_status_out  // [3]: alarm_sig, [2:0]: rgb_status
);

    // Synchronize asynchronous push-button reset to system clock
    reg rst_sync0, rst_sync1;
    always @(posedge clk) begin
        rst_sync0 <= btn_rst;
        rst_sync1 <= rst_sync0;
    end
    wire rst = rst_sync1;

    // -------------------------------------------------------------
    // PS Key Injection Synchronizer & 1-Cycle Rising Edge Detector
    // Resolves software oversampling by generating an exact 1-cycle pulse
    // -------------------------------------------------------------
    reg [2:0] ps_valid_sync;
    reg [3:0] ps_data_latched;

    always @(posedge clk) begin
        if (rst) begin
            ps_valid_sync   <= 3'b000;
            ps_data_latched <= 4'h0;
        end else begin
            ps_valid_sync <= {ps_valid_sync[1:0], ps_key_inject[4]};
            if (ps_key_inject[4]) begin
                ps_data_latched <= ps_key_inject[3:0];
            end
        end
    end

    // Produces a single clock cycle pulse on software 0 -> 1 transition
    wire ps_key_pulse = (ps_valid_sync[1] && !ps_valid_sync[2]);

    wire       hw_key_valid;
    wire [3:0] hw_key_data;
    wire       unlock_sig;
    wire [2:0] rgb_status; // [2]=Red, [1]=Green, [0]=Blue
    wire       alarm_sig;

    // Physical matrix keypad scanner
    keypad_scanner u_keypad (
        .clk       (clk),
        .rst       (rst),
        .row       (kp_row),
        .col       (kp_col),
        .key_valid (hw_key_valid),
        .key_data  (hw_key_data)
    );

    // Multiplexer: Combine keypad and software injection
    wire sys_key_valid      = hw_key_valid | ps_key_pulse;
    wire [3:0] sys_key_data = ps_key_pulse ? ps_data_latched : hw_key_data;

    // Core Door Lock FSM
    door_lock_fsm #(
        .UNLOCK_HOLD_CYCLES(UNLOCK_HOLD_CYCLES)
    ) u_fsm (
        .clk        (clk),
        .rst        (rst),
        .key_valid  (sys_key_valid),
        .key_data   (sys_key_data),
        .unlock_sig (unlock_sig),
        .rgb_status (rgb_status),
        .alarm_sig  (alarm_sig)
    );

    // 50Hz PWM Servo Generator
    pwm_generator u_pwm (
        .clk        (clk),
        .rst        (rst),
        .unlock_sig (unlock_sig),
        .servo_pwm  (servo_pwm)
    );

    // Route PL status out to Processing System AXI GPIO
    assign ps_status_out = {alarm_sig, rgb_status};

    // Flashing circuit for alarm (~2Hz)
    reg [26:0] flash_cnt;
    reg        flash_bit;
    always @(posedge clk) begin
        if (rst) begin
            flash_cnt <= 27'd0;
            flash_bit <= 1'b0;
        end else if (flash_cnt >= 27'd62_500_000) begin 
            flash_cnt <= 27'd0;
            flash_bit <= ~flash_bit;
        end else begin
            flash_cnt <= flash_cnt + 27'd1;
        end
    end

    // Selected RGB bit vector:
    // [2] = Red, [1] = Green, [0] = Blue
    wire [2:0] active_rgb = alarm_sig ? {flash_bit, flash_bit, flash_bit} : rgb_status;

    // Active-high driving: 1 = ON, 0 = OFF (fixes inverted polarity and additive color mixing)
    assign led_r = active_rgb[2]; // Bit 2 -> Red Pin (N15)
    assign led_g = active_rgb[1]; // Bit 1 -> Green Pin (G17)
    assign led_b = active_rgb[0]; // Bit 0 -> Blue Pin (L15)

endmodule