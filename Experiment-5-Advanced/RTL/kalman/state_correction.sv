`timescale 1ns / 1ps

module state_correction #(
    parameter integer W = 20,
    parameter integer F = 12,
    parameter integer ACC_W = 44
)(
    input logic clk,
    input logic rst,
    input logic start,

    input logic signed [W-1:0] x_pred [0:3],
    input logic signed [W-1:0] K [0:3][0:1],
    input logic signed [W-1:0] innovation [0:1],

    output logic signed [W-1:0] x_updated [0:3],

    output logic busy,
    output logic done
);

    typedef enum logic [1:0] {
        IDLE,
        COMPUTE_MULT,
        COMPUTE_ACCUM
    } state_t;

    state_t state;

    logic signed [W-1:0] x_pred_reg [0:3];
    logic signed [W-1:0] K_reg [0:3][0:1];
    logic signed [W-1:0] innovation_reg [0:1];

    (* max_fanout = "8" *) logic [1:0] row;

    // Pipeline registers to break combinational paths
    logic signed [2*W-1:0] product0_reg;
    logic signed [2*W-1:0] product1_reg;

    integer i;
    integer j;

    always_ff @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            row  <= '0;
            product0_reg <= '0;
            product1_reg <= '0;

            for (i = 0; i < 4; i = i + 1) begin
                x_pred_reg[i] <= '0;
                x_updated[i] <= '0;
                for (j = 0; j < 2; j = j + 1) begin
                    K_reg[i][j] <= '0;
                end
            end

            innovation_reg[0] <= '0;
            innovation_reg[1] <= '0;

        end else begin
            done <= 1'b0;

            case (state)
                IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        row  <= 2'd0;

                        for (i = 0; i < 4; i = i + 1) begin
                            x_pred_reg[i] <= x_pred[i];
                            for (j = 0; j < 2; j = j + 1) begin
                                K_reg[i][j] <= K[i][j];
                            end
                        end

                        innovation_reg[0] <= innovation[0];
                        innovation_reg[1] <= innovation[1];

                        state <= COMPUTE_MULT;
                    end
                end

                COMPUTE_MULT: begin
                    // Stage 1: Register multipliers to cut logic depth
                    product0_reg <= $signed(K_reg[row][0]) * $signed(innovation_reg[0]);
                    product1_reg <= $signed(K_reg[row][1]) * $signed(innovation_reg[1]);
                    state <= COMPUTE_ACCUM;
                end

                COMPUTE_ACCUM: begin
                    // Stage 2: Accumulate, shift, and add to state prediction
                    logic signed [ACC_W-1:0] accumulator;
                    logic signed [ACC_W-1:0] correction;
                    logic signed [ACC_W-1:0] updated_value;

                    accumulator = product0_reg + product1_reg;
                    correction = accumulator >>> F;
                    updated_value = x_pred_reg[row] + correction;

                    x_updated[row] <= updated_value[W-1:0];

                    if (row == 3) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                        state <= IDLE;
                    end else begin
                        row <= row + 1'b1;
                        state <= COMPUTE_MULT;
                    end
                end

                default: begin
                    state <= IDLE;
                    busy <= 1'b0;
                end
            endcase
        end
    end

endmodule