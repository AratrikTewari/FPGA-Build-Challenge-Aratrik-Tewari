`timescale 1ns / 1ps

module multi_target_engine #(
    parameter integer W = 20,
    parameter integer F = 12,
    parameter integer ACC_W = 44,
    parameter integer INV_W = 16,
    parameter integer NUM_TARGETS = 8,
    parameter integer ID_W = 3
)(
    input logic clk,
    input logic rst,
    input logic start,

    // Measurements for all targets
    input logic signed [W-1:0] z [0:NUM_TARGETS-1][0:1],

    // Kalman model
    input logic signed [W-1:0] F_mat [0:3][0:3],
    input logic signed [W-1:0] Q [0:3][0:3],
    input logic signed [W-1:0] R_diag [0:1],

    // Updated target outputs
    output logic signed [W-1:0] x_out [0:NUM_TARGETS-1][0:3],
    output logic signed [W-1:0] P_out [0:NUM_TARGETS-1][0:3][0:3],

    output logic busy,
    output logic done
);


    // ============================================================
    // Scheduler
    // ============================================================

    logic [ID_W-1:0] target_id;

    logic memory_read_request;
    logic memory_read_valid;

    logic memory_write_enable;

    logic kalman_start;
    logic kalman_busy;
    logic kalman_done;


    // ============================================================
    // Memory & Flags
    // ============================================================

    logic signed [W-1:0] x_read [0:3];
    logic signed [W-1:0] P_read [0:3][0:3];

    logic signed [W-1:0] x_write [0:3];
    logic signed [W-1:0] P_write [0:3][0:3];

    // NEW: Persist maneuver state per target
    logic maneuver_flags [0:NUM_TARGETS-1]; 
    logic maneuver_kalman_out;


    // ============================================================
    // Kalman engine
    // ============================================================

    logic signed [W-1:0] x_kalman [0:3];
    logic signed [W-1:0] P_kalman [0:3][0:3];


    // ============================================================
    // Target scheduler
    // ============================================================

    target_scheduler #(
        .NUM_TARGETS(NUM_TARGETS),
        .ID_W(ID_W)
    ) scheduler_inst (

        .clk(clk),
        .rst(rst),

        .start(start),

        .target_id(target_id),

        .memory_read_request(memory_read_request),
        .memory_read_valid(memory_read_valid),

        .memory_write_enable(memory_write_enable),

        .kalman_start(kalman_start),
        .kalman_busy(kalman_busy),
        .kalman_done(kalman_done),

        .busy(busy),
        .frame_done(done)
    );


    // ============================================================
    // Target memory
    // ============================================================

    target_memory #(
        .W(W),
        .NUM_TARGETS(NUM_TARGETS)
    ) memory_inst (

        .clk(clk),
        .rst(rst),

        .target_id(target_id),

        .write_enable(memory_write_enable),

        .x_write(x_write),
        .P_write(P_write),

        .x_read(x_read),
        .P_read(P_read),

        .read_valid(memory_read_valid)
    );


    // ============================================================
    // Kalman engine
    // ============================================================

    kalman_engine #(
        .W(W),
        .F(F),
        .ACC_W(ACC_W),
        .INV_W(INV_W)
    ) kalman_inst (

        .clk(clk),
        .rst(rst),

        .start(kalman_start),
        .maneuver_in(maneuver_flags[target_id]), // NEW: Feed target's saved maneuver state

        .x_in(x_read),
        .P_in(P_read),

        .z(z[target_id]),

        .F_mat(F_mat),
        .Q(Q),
        .R_diag(R_diag),

        .x_out(x_kalman),
        .P_out(P_kalman),
        
        .maneuver_out(maneuver_kalman_out),     // NEW: Capture computed maneuver status
        .busy(kalman_busy),
        .done(kalman_done)
    );


    // ============================================================
    // Connect Kalman result to memory write port
    // ============================================================

    always_comb begin

        for (integer i = 0; i < 4; i = i + 1) begin

            x_write[i] = x_kalman[i];

            for (integer j = 0; j < 4; j = j + 1) begin
                P_write[i][j] = P_kalman[i][j];
            end

        end

    end


    // ============================================================
    // Export completed target results and update flags
    // ============================================================

    always_ff @(posedge clk) begin

        if (rst) begin

            for (integer t = 0; t < NUM_TARGETS; t = t + 1) begin
                
                maneuver_flags[t] <= 1'b0; // Reset flags

                for (integer i = 0; i < 4; i = i + 1) begin

                    x_out[t][i] <= '0;

                    for (integer j = 0; j < 4; j = j + 1) begin
                        P_out[t][i][j] <= '0;
                    end

                end

            end

        end
        else if (memory_write_enable) begin
            
            maneuver_flags[target_id] <= maneuver_kalman_out; // Save flag for next frame

            for (integer i = 0; i < 4; i = i + 1) begin

                x_out[target_id][i] <= x_write[i];

                for (integer j = 0; j < 4; j = j + 1) begin
                    P_out[target_id][i][j] <= P_write[i][j];
                end

            end

        end

    end

endmodule