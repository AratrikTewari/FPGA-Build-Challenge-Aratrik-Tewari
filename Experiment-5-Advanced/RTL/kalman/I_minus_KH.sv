`timescale 1ns / 1ps

module I_minus_KH #(
    parameter integer W = 20,
    parameter integer IDENTITY = 4096
)(
    input logic clk,
    input logic rst,
    input logic start,

    input logic signed [W-1:0] KH [0:3][0:3],

    output logic signed [W-1:0] I_KH [0:3][0:3],

    output logic busy,
    output logic done
);

    logic signed [W-1:0] KH_reg [0:3][0:3];

    logic [2:0] row;

    logic signed [W:0] result;

    integer i;
    integer j;

    always_ff @(posedge clk) begin

        if (rst) begin

            busy <= 1'b0;
            done <= 1'b0;
            row  <= '0;

            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    KH_reg[i][j] <= '0;
                    I_KH[i][j] <= '0;
                end
            end

        end
        else begin

            done <= 1'b0;

            if (!busy) begin

                if (start) begin

                    busy <= 1'b1;
                    row  <= 3'd0;

                    for (i = 0; i < 4; i = i + 1) begin
                        for (j = 0; j < 4; j = j + 1) begin
                            KH_reg[i][j] <= KH[i][j];
                        end
                    end

                end

            end
            else begin

                // I is the 4x4 identity matrix.
                //
                // Identity value in Q8.12:
                // 1.0 = 4096
                //
                // I-KH:
                // diagonal     = 4096 - KH
                // off-diagonal = -KH

                if (row == 0) begin

                    result = IDENTITY - KH_reg[0][0];
                    I_KH[0][0] <= result[W-1:0];

                    result = -KH_reg[0][1];
                    I_KH[0][1] <= result[W-1:0];

                    result = -KH_reg[0][2];
                    I_KH[0][2] <= result[W-1:0];

                    result = -KH_reg[0][3];
                    I_KH[0][3] <= result[W-1:0];

                end
                else if (row == 1) begin

                    result = -KH_reg[1][0];
                    I_KH[1][0] <= result[W-1:0];

                    result = IDENTITY - KH_reg[1][1];
                    I_KH[1][1] <= result[W-1:0];

                    result = -KH_reg[1][2];
                    I_KH[1][2] <= result[W-1:0];

                    result = -KH_reg[1][3];
                    I_KH[1][3] <= result[W-1:0];

                end
                else if (row == 2) begin

                    result = -KH_reg[2][0];
                    I_KH[2][0] <= result[W-1:0];

                    result = -KH_reg[2][1];
                    I_KH[2][1] <= result[W-1:0];

                    result = IDENTITY - KH_reg[2][2];
                    I_KH[2][2] <= result[W-1:0];

                    result = -KH_reg[2][3];
                    I_KH[2][3] <= result[W-1:0];

                end
                else begin

                    result = -KH_reg[3][0];
                    I_KH[3][0] <= result[W-1:0];

                    result = -KH_reg[3][1];
                    I_KH[3][1] <= result[W-1:0];

                    result = -KH_reg[3][2];
                    I_KH[3][2] <= result[W-1:0];

                    result = IDENTITY - KH_reg[3][3];
                    I_KH[3][3] <= result[W-1:0];

                end

                if (row == 3) begin

                    busy <= 1'b0;
                    done <= 1'b1;

                end
                else begin

                    row <= row + 1'b1;

                end

            end

        end

    end

endmodule