`timescale 1ns / 1ps

// ============================================================================
// Module:        collision_engine
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Advanced FPGA architecture implementation featuring Kalman filters,
//   collision detection, and multi-target tracking.
//
// Architectural Hierarchy:
//   Part of the Experiment 5 RTL / Simulation / Testbench ecosystem.
// ============================================================================

module collision_engine #(
    parameter integer W = 20,
    parameter integer F = 12,
    parameter integer NUM_TARGETS = 8
)(
    input logic clk, input logic rst, input logic start,
    input logic signed [W-1:0] target_state [0:NUM_TARGETS-1][0:3],
    input logic signed [W-1:0] prediction_horizon,
    input logic signed [W-1:0] collision_threshold,
    output logic collision_matrix [0:NUM_TARGETS-1][0:NUM_TARGETS-1],
    output logic collision_detected, output logic busy, output logic done
);

    localparam integer PROD_W = 2 * W;
    localparam integer SQ_W   = PROD_W + 1;

    logic signed [W-1:0] state_latched [0:NUM_TARGETS-1][0:3];
    logic signed [W-1:0] horizon_latched;
    logic signed [W-1:0] threshold_latched;

    logic signed [W-1:0] predicted_x [0:NUM_TARGETS-1];
    logic signed [W-1:0] predicted_y [0:NUM_TARGETS-1];

    // Pipeline arrays for parallel multiplication
    logic signed [PROD_W-1:0] vx_time_raw_arr [0:NUM_TARGETS-1];
    logic signed [PROD_W-1:0] vy_time_raw_arr [0:NUM_TARGETS-1];

    integer pair_i, pair_j;
    logic signed [W-1:0] dx, dy;
    logic signed [PROD_W-1:0] dx_sq_raw, dy_sq_raw, threshold_sq_raw;
    logic signed [SQ_W-1:0] distance_sq, threshold_sq;

    typedef enum logic [3:0] {
        IDLE, LATCH, 
        PREDICT_MULT, PREDICT_ADD, 
        COMPUTE_PAIR, CHECK_MULT, CHECK_ADD, CHECK_EVAL, 
        FINISH
    } state_t;
    state_t state;
    integer i, j;

    always_ff @(posedge clk) begin
        if (rst) begin
            state <= IDLE; busy <= 1'b0; done <= 1'b0; collision_detected <= 1'b0;
            horizon_latched <= '0; threshold_latched <= '0; pair_i <= 0; pair_j <= 1;
            dx <= '0; dy <= '0; dx_sq_raw <= '0; dy_sq_raw <= '0; threshold_sq_raw <= '0;
            distance_sq <= '0; threshold_sq <= '0;
            for (i = 0; i < NUM_TARGETS; i = i + 1) begin
                predicted_x[i] <= '0; predicted_y[i] <= '0;
                vx_time_raw_arr[i] <= '0; vy_time_raw_arr[i] <= '0;
                for (j = 0; j < 4; j = j + 1) state_latched[i][j] <= '0;
                for (j = 0; j < NUM_TARGETS; j = j + 1) collision_matrix[i][j] <= 1'b0;
            end
        end else begin
            done <= 1'b0;
            case (state)
                IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy <= 1'b1; collision_detected <= 1'b0;
                        pair_i <= 0; pair_j <= 1; state <= LATCH;
                    end
                end

                LATCH: begin
                    horizon_latched <= prediction_horizon;
                    threshold_latched <= collision_threshold;
                    for (i = 0; i < NUM_TARGETS; i = i + 1) begin
                        for (j = 0; j < 4; j = j + 1) state_latched[i][j] <= target_state[i][j];
                        for (j = 0; j < NUM_TARGETS; j = j + 1) collision_matrix[i][j] <= 1'b0;
                    end
                    state <= PREDICT_MULT;
                end

                // Pipelined Prediction (Isolating Multipliers)
                PREDICT_MULT: begin
                    for (i = 0; i < NUM_TARGETS; i = i + 1) begin
                        vx_time_raw_arr[i] <= $signed(state_latched[i][2]) * $signed(horizon_latched);
                        vy_time_raw_arr[i] <= $signed(state_latched[i][3]) * $signed(horizon_latched);
                    end
                    threshold_sq_raw <= $signed(threshold_latched) * $signed(threshold_latched);
                    state <= PREDICT_ADD;
                end

                // Pipelined Prediction (Isolating Adders)
                PREDICT_ADD: begin
                    for (i = 0; i < NUM_TARGETS; i = i + 1) begin
                        predicted_x[i] <= $signed(state_latched[i][0]) + $signed(vx_time_raw_arr[i] >>> F);
                        predicted_y[i] <= $signed(state_latched[i][1]) + $signed(vy_time_raw_arr[i] >>> F);
                    end
                    threshold_sq <= $signed(threshold_sq_raw >>> F);
                    state <= COMPUTE_PAIR;
                end

                COMPUTE_PAIR: begin
                    if (pair_i >= NUM_TARGETS - 1) state <= FINISH;
                    else begin
                        dx <= $signed(predicted_x[pair_i]) - $signed(predicted_x[pair_j]);
                        dy <= $signed(predicted_y[pair_i]) - $signed(predicted_y[pair_j]);
                        state <= CHECK_MULT;
                    end
                end

                // Pipelined Distance (Isolating Multipliers)
                CHECK_MULT: begin
                    dx_sq_raw <= $signed(dx) * $signed(dx);
                    dy_sq_raw <= $signed(dy) * $signed(dy);
                    state <= CHECK_ADD;
                end

                // Pipelined Distance (Isolating Adders)
                CHECK_ADD: begin
                    distance_sq <= $signed(dx_sq_raw >>> F) + $signed(dy_sq_raw >>> F);
                    state <= CHECK_EVAL;
                end

                CHECK_EVAL: begin
                    if (distance_sq <= threshold_sq) begin
                        collision_matrix[pair_i][pair_j] <= 1'b1;
                        collision_matrix[pair_j][pair_i] <= 1'b1;
                        collision_detected <= 1'b1;
                    end

                    if (pair_j == NUM_TARGETS - 1) begin
                        if (pair_i == NUM_TARGETS - 2) state <= FINISH;
                        else begin
                            pair_i <= pair_i + 1;
                            pair_j <= pair_i + 2;
                            state <= COMPUTE_PAIR;
                        end
                    end else begin
                        pair_j <= pair_j + 1;
                        state <= COMPUTE_PAIR;
                    end
                end

                FINISH: begin busy <= 1'b0; done <= 1'b1; state <= IDLE; end
                default: begin state <= IDLE; busy <= 1'b0; end
            endcase
        end
    end
endmodule
