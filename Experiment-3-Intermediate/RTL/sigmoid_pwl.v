`timescale 1ns / 1ps
//
// Pipelined Q4.12 piecewise-linear sigmoid core.
// Bit-exact contract: model/fixed_point_model.py::plan_pwl_fixed.
// Latency: valid_in to valid_out = 2 rising clock edges.
//
module sigmoid_pwl (
    input  wire              clk,
    input  wire              rst_n,
    input  wire signed [15:0] din_q4_12,
    input  wire               valid_in,
    output reg  signed [15:0] dout_q4_12,
    output reg                valid_out
);

    // Q4.12 PWL constants; gains are right shifts only.
    localparam [15:0] Q_ONE                = 16'h1000;
    localparam [15:0] LOW_THRESHOLD        = 16'h1000; // 1.0
    localparam [15:0] MID_THRESHOLD        = 16'h2600; // 2.375
    localparam [15:0] SATURATION_THRESHOLD = 16'h5000; // 5.0
    localparam [15:0] LOW_INTERCEPT        = 16'h0800; // 0.5
    localparam [15:0] MID_INTERCEPT        = 16'h0A00; // 0.625
    localparam [15:0] HIGH_INTERCEPT       = 16'h0D80; // 0.84375

    // Pipeline Stage 1 Registers
    reg        sign_reg;
    reg [15:0] abs_x;
    reg        stage1_valid;

    // Pipeline Stage 2 Registers
    reg        [15:0] y_pos;
    reg               stage2_sign;
    reg               stage2_valid;

    // Stage 1: Absolute value extraction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sign_reg     <= 1'b0;
            abs_x        <= 16'd0;
            stage1_valid <= 1'b0;
        end else begin
            stage1_valid <= valid_in;
            sign_reg     <= din_q4_12[15];
            abs_x        <= din_q4_12[15] ? -din_q4_12 : din_q4_12;
        end
    end

    // Stage 2: Piecewise linear calculation using shifts and adds
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_pos        <= 16'd0;
            stage2_sign  <= 1'b0;
            stage2_valid <= 1'b0;
        end else begin
            stage2_valid <= stage1_valid;
            stage2_sign  <= sign_reg;

            if (abs_x >= SATURATION_THRESHOLD) begin
                y_pos <= Q_ONE;                       // y = 1.0
            end else if (abs_x >= MID_THRESHOLD) begin
                y_pos <= (abs_x >> 5) + HIGH_INTERCEPT; // y = x/32 + 0.84375
            end else if (abs_x >= LOW_THRESHOLD) begin
                y_pos <= (abs_x >> 3) + MID_INTERCEPT;  // y = x/8 + 0.625
            end else begin
                y_pos <= (abs_x >> 2) + LOW_INTERCEPT;  // y = x/4 + 0.5
            end
        end
    end

    // Stage 3: Output generation (Inversion for negative inputs)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dout_q4_12 <= 16'd0;
            valid_out  <= 1'b0;
        end else begin
            valid_out  <= stage2_valid;
            dout_q4_12 <= stage2_sign ? (Q_ONE - y_pos) : y_pos;
        end
    end

endmodule
