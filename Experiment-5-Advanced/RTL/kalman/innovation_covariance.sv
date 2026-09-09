`timescale 1ns / 1ps

module innovation_covariance #(
    parameter integer W = 20
)(
    input logic clk,
    input logic rst,
    input logic start,

    input logic signed [W-1:0] P_pred [0:3][0:3],
    input logic signed [W-1:0] R_diag [0:1],

    output logic signed [W-1:0] S [0:1],

    output logic busy,
    output logic done
);

    logic signed [W-1:0] P_reg [0:3][0:3];
    logic signed [W-1:0] R_reg [0:1];

    logic [1:0] index;

    logic signed [W:0] sum;

    integer i;
    integer j;

    always_ff @(posedge clk) begin

        if (rst) begin

            busy <= 1'b0;
            done <= 1'b0;
            index <= '0;

            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    P_reg[i][j] <= '0;
                end
            end

            for (i = 0; i < 2; i = i + 1) begin
                R_reg[i] <= '0;
                S[i] <= '0;
            end

        end else begin

            done <= 1'b0;

            if (!busy) begin

                if (start) begin

                    busy <= 1'b1;
                    index <= 2'd0;

                    for (i = 0; i < 4; i = i + 1) begin
                        for (j = 0; j < 4; j = j + 1) begin
                            P_reg[i][j] <= P_pred[i][j];
                        end
                    end

                    for (i = 0; i < 2; i = i + 1) begin
                        R_reg[i] <= R_diag[i];
                    end

                end

            end else begin

                case (index)

                    2'd0: begin

                        sum = P_reg[0][0] + R_reg[0];

                        S[0] <= sum[W-1:0];

                        index <= 2'd1;

                    end

                    2'd1: begin

                        sum = P_reg[1][1] + R_reg[1];

                        S[1] <= sum[W-1:0];

                        busy <= 1'b0;
                        done <= 1'b1;

                    end

                    default: begin

                        busy <= 1'b0;
                        done <= 1'b0;
                        index <= 2'd0;

                    end

                endcase

            end

        end

    end

endmodule