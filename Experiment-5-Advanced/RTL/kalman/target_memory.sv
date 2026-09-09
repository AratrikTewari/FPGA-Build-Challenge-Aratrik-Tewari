`timescale 1ns / 1ps

module target_memory #(
    parameter integer W = 20,
    parameter integer NUM_TARGETS = 8
)(
    input logic clk,
    input logic rst,

    // ============================================================
    // Target selection
    // ============================================================

    input logic [$clog2(NUM_TARGETS)-1:0] target_id,

    // ============================================================
    // Normal runtime write
    // ============================================================

    input logic write_enable,

    input logic signed [W-1:0] x_write [0:3],
    input logic signed [W-1:0] P_write [0:3][0:3],

    // ============================================================
    // Synchronous read
    // ============================================================

    output logic signed [W-1:0] x_read [0:3],
    output logic signed [W-1:0] P_read [0:3][0:3],

    output logic read_valid,

    // ============================================================
    // Initialization / load interface
    // ============================================================

    input logic load_enable,

    input logic [$clog2(NUM_TARGETS)-1:0] load_target,

    input logic signed [W-1:0] x_load [0:3],
    input logic signed [W-1:0] P_load [0:3][0:3],

    output logic load_done
);

    localparam integer ID_W = $clog2(NUM_TARGETS);

    // ============================================================
    // Memory arrays
    // ============================================================

    logic signed [W-1:0] x_mem
        [0:NUM_TARGETS-1][0:3];

    logic signed [W-1:0] P_mem
        [0:NUM_TARGETS-1][0:3][0:3];

    integer i;
    integer j;
    integer k;

    // ============================================================
    // Sequential memory logic
    // ============================================================

    always_ff @(posedge clk) begin

        if (rst) begin

            read_valid <= 1'b0;
            load_done  <= 1'b0;

            for (i = 0; i < NUM_TARGETS; i = i + 1) begin

                for (j = 0; j < 4; j = j + 1) begin

                    x_mem[i][j] <= '0;

                    for (k = 0; k < 4; k = k + 1) begin
                        P_mem[i][j][k] <= '0;
                    end

                end

            end

            for (j = 0; j < 4; j = j + 1) begin

                x_read[j] <= '0;

                for (k = 0; k < 4; k = k + 1) begin
                    P_read[j][k] <= '0;
                end

            end

        end

        else begin

            // ----------------------------------------------------
            // Default status
            // ----------------------------------------------------

            read_valid <= 1'b1;
            load_done  <= 1'b0;

            // ----------------------------------------------------
            // Initialization load
            //
            // Load has priority over runtime write.
            // ----------------------------------------------------

            if (load_enable) begin

                for (j = 0; j < 4; j = j + 1) begin

                    x_mem[load_target][j] <= x_load[j];

                    for (k = 0; k < 4; k = k + 1) begin
                        P_mem[load_target][j][k] <= P_load[j][k];
                    end

                end

                load_done <= 1'b1;

            end

            // ----------------------------------------------------
            // Normal runtime write
            // ----------------------------------------------------

            else if (write_enable) begin

                for (j = 0; j < 4; j = j + 1) begin

                    x_mem[target_id][j] <= x_write[j];

                    for (k = 0; k < 4; k = k + 1) begin
                        P_mem[target_id][j][k] <= P_write[j][k];
                    end

                end

            end

            // ----------------------------------------------------
            // Synchronous read
            // ----------------------------------------------------

            for (j = 0; j < 4; j = j + 1) begin

                x_read[j] <= x_mem[target_id][j];

                for (k = 0; k < 4; k = k + 1) begin
                    P_read[j][k] <= P_mem[target_id][j][k];
                end

            end

        end

    end

endmodule