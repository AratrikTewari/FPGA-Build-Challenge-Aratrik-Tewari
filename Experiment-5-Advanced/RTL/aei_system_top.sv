`timescale 1ns / 1ps

// ============================================================================
// Module:        aei_system_top.sv
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Topmost system integration file wrapping the AEI engine with PS-PL boundary components (e.g. AXI interconnects, clock domain crossing).
//
// Architectural Hierarchy:
//   - Instantiated within the broader Experiment 5 RTL/Verification ecosystem.
//   - Synthesizable for PL (Programmable Logic) deployment unless marked as TB.
// ============================================================================
// ============================================================
// AEI Engine System Top
// ============================================================

module aei_system_top #(
    parameter integer W = 20,
    parameter integer F = 12,
    parameter integer ACC_W = 44,
    parameter integer INV_W = 16,
    parameter integer NUM_TARGETS = 8
)(
    input logic clk,
    input logic rst,

    input logic frame_start,
    input logic measurement_valid,
    input logic signed [W-1:0] z_in [0:1],

    input logic load_enable,
    input logic [$clog2(NUM_TARGETS)-1:0] load_target,
    input logic signed [W-1:0] x_load [0:3],
    input logic signed [W-1:0] P_load [0:3][0:3],
    output logic load_done,

    input logic write_enable,
    input logic signed [W-1:0] x_write [0:3],
    input logic signed [W-1:0] P_write [0:3][0:3],

    input logic signed [W-1:0] F_mat [0:3][0:3],
    input logic signed [W-1:0] Q [0:3][0:3],
    input logic signed [W-1:0] R_diag [0:1],

    input logic signed [W-1:0] prediction_horizon,
    input logic signed [W-1:0] collision_threshold,

    output logic busy,
    output logic frame_done,
    output logic [$clog2(NUM_TARGETS)-1:0] target_id,
    output logic signed [W-1:0] x_out [0:3],
    output logic signed [W-1:0] P_out [0:3][0:3],

    output logic collision_busy,
    output logic collision_done,
    output logic collision_detected,
    output logic collision_matrix [0:NUM_TARGETS-1][0:NUM_TARGETS-1]
);

    localparam integer ID_W = $clog2(NUM_TARGETS);

    logic aei_busy;
    logic aei_frame_done;
    logic [ID_W-1:0] aei_target_id;
    logic signed [W-1:0] aei_x_out [0:3];
    logic signed [W-1:0] aei_P_out [0:3][0:3];
    logic aei_load_done;
    logic aei_collision_busy;
    logic aei_collision_done;
    logic aei_collision_detected;
    logic aei_collision_matrix [0:NUM_TARGETS-1][0:NUM_TARGETS-1];

    aei_engine_top #(
        .W(W),
        .F(F),
        .ACC_W(ACC_W),
        .INV_W(INV_W),
        .NUM_TARGETS(NUM_TARGETS)
    ) u_aei_engine_top (
        .clk(clk),
        .rst(rst),
        .frame_start(frame_start),
        .measurement_valid(measurement_valid),
        .z_in(z_in),
        .load_enable(load_enable),
        .load_target(load_target),
        .x_load(x_load),
        .P_load(P_load),
        .load_done(aei_load_done),
        .write_enable(write_enable),
        .x_write(x_write),
        .P_write(P_write),
        .F_mat(F_mat),
        .Q(Q),
        .R_diag(R_diag),
        .prediction_horizon(prediction_horizon),
        .collision_threshold(collision_threshold),
        .busy(aei_busy),
        .frame_done(aei_frame_done),
        .target_id(aei_target_id),
        .x_out(aei_x_out),
        .P_out(aei_P_out),
        .collision_busy(aei_collision_busy),
        .collision_done(aei_collision_done),
        .collision_detected(aei_collision_detected),
        .collision_matrix(aei_collision_matrix)
    );

    assign busy = aei_busy;
    assign frame_done = aei_frame_done;
    assign target_id = aei_target_id;
    assign x_out = aei_x_out;
    assign P_out = aei_P_out;
    assign load_done = aei_load_done;
    assign collision_busy = aei_collision_busy;
    assign collision_done = aei_collision_done;
    assign collision_detected = aei_collision_detected;

    genvar i;
    genvar j;
    generate
        for (i = 0; i < NUM_TARGETS; i = i + 1) begin : GEN_MATRIX_I
            for (j = 0; j < NUM_TARGETS; j = j + 1) begin : GEN_MATRIX_J
                assign collision_matrix[i][j] = aei_collision_matrix[i][j];
            end
        end
    endgenerate

endmodule
