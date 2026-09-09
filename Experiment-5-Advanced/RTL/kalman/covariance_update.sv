`timescale 1ns / 1ps

module covariance_update #(
    parameter integer W = 20,
    parameter integer F = 12,
    parameter integer ACC_W = 44
)(
    input logic clk,
    input logic rst,
    input logic start,

    input logic signed [W-1:0] I_KH [0:3][0:3],
    input logic signed [W-1:0] P_pred [0:3][0:3],

    output logic signed [W-1:0] P_updated [0:3][0:3],

    output logic busy,
    output logic done
);

    typedef enum logic [1:0] {
        IDLE,
        MULT_STAGE,
        ADD_STAGE_1,
        ADD_STAGE_2
    } state_t;

    state_t state;

    logic signed [W-1:0] I_KH_reg [0:3][0:3];
    logic signed [W-1:0] P_pred_reg [0:3][0:3];

    (* max_fanout = "8" *) logic [1:0] row;
    (* max_fanout = "8" *) logic [1:0] col;

    logic signed [39:0] p0, p1, p2, p3;
    logic signed [ACC_W-1:0] sum0, sum1;
    logic signed [ACC_W-1:0] shifted_result;

    integer i, j;

    always_ff @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0; done <= 1'b0; row <= '0; col <= '0;
            p0 <= '0; p1 <= '0; p2 <= '0; p3 <= '0; sum0 <= '0; sum1 <= '0;
            for (i = 0; i < 4; i = i + 1)
                for (j = 0; j < 4; j = j + 1) begin
                    I_KH_reg[i][j] <= '0; P_pred_reg[i][j] <= '0; P_updated[i][j] <= '0;
                end
        end else begin
            done <= 1'b0;
            case (state)
                IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy <= 1'b1; row <= 2'd0; col <= 2'd0;
                        for (i = 0; i < 4; i = i + 1)
                            for (j = 0; j < 4; j = j + 1) begin
                                I_KH_reg[i][j] <= I_KH[i][j];
                                P_pred_reg[i][j] <= P_pred[i][j];
                            end
                        state <= MULT_STAGE;
                    end
                end

                MULT_STAGE: begin
                    p0 <= $signed(I_KH_reg[row][0]) * $signed(P_pred_reg[0][col]);
                    p1 <= $signed(I_KH_reg[row][1]) * $signed(P_pred_reg[1][col]);
                    p2 <= $signed(I_KH_reg[row][2]) * $signed(P_pred_reg[2][col]);
                    p3 <= $signed(I_KH_reg[row][3]) * $signed(P_pred_reg[3][col]);
                    state <= ADD_STAGE_1;
                end

                ADD_STAGE_1: begin
                    sum0 <= p0 + p1;
                    sum1 <= p2 + p3;
                    state <= ADD_STAGE_2;
                end

                ADD_STAGE_2: begin
                    shifted_result = (sum0 + sum1) >>> F;
                    P_updated[row][col] <= shifted_result[W-1:0];

                    if (col == 3) begin
                        col <= 2'd0;
                        if (row == 3) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            state <= IDLE;
                        end else begin
                            row <= row + 1'b1;
                            state <= MULT_STAGE;
                        end
                    end else begin
                        col <= col + 1'b1;
                        state <= MULT_STAGE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule