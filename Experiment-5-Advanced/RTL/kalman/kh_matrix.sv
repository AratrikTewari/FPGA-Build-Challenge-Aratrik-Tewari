`timescale 1ns / 1ps

module kh_matrix #(
    parameter integer W = 20
)(
    input logic clk,
    input logic rst,
    input logic start,

    input logic signed [W-1:0] K [0:3][0:1],

    output logic signed [W-1:0] KH [0:3][0:3],

    output logic busy,
    output logic done
);

    logic signed [W-1:0] K_reg [0:3][0:1];

    logic [2:0] row;

    integer i;
    integer j;

    always_ff @(posedge clk) begin

        if (rst) begin

            busy <= 1'b0;
            done <= 1'b0;
            row  <= '0;

            for (i = 0; i < 4; i = i + 1) begin

                for (j = 0; j < 2; j = j + 1) begin
                    K_reg[i][j] <= '0;
                end

                for (j = 0; j < 4; j = j + 1) begin
                    KH[i][j] <= '0;
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
                        for (j = 0; j < 2; j = j + 1) begin
                            K_reg[i][j] <= K[i][j];
                        end
                    end

                end

            end
            else begin

                // H =
                //
                // [1 0 0 0]
                // [0 1 0 0]
                //
                // Therefore:
                //
                // KH =
                //
                // [K00 K01  0  0]
                // [K10 K11  0  0]
                // [K20 K21  0  0]
                // [K30 K31  0  0]

                KH[row][0] <= K_reg[row][0];
                KH[row][1] <= K_reg[row][1];
                KH[row][2] <= '0;
                KH[row][3] <= '0;

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