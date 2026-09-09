`timescale 1ns / 1ps

module ph_transpose #(
    parameter integer W = 20
)(
    input logic clk,
    input logic rst,
    input logic start,

    input logic signed [W-1:0] P_pred [0:3][0:3],

    output logic signed [W-1:0] PHt [0:3][0:1],

    output logic busy,
    output logic done
);

    logic signed [W-1:0] P_reg [0:3][0:3];

    logic [2:0] row;

    integer i;
    integer j;

    always_ff @(posedge clk) begin

        if (rst) begin

            busy <= 1'b0;
            done <= 1'b0;
            row  <= '0;

            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    P_reg[i][j] <= '0;
                end
            end

            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 2; j = j + 1) begin
                    PHt[i][j] <= '0;
                end
            end

        end else begin

            done <= 1'b0;

            if (!busy) begin

                if (start) begin

                    busy <= 1'b1;
                    row  <= 3'd0;

                    for (i = 0; i < 4; i = i + 1) begin
                        for (j = 0; j < 4; j = j + 1) begin
                            P_reg[i][j] <= P_pred[i][j];
                        end
                    end

                end

            end else begin

                PHt[row][0] <= P_reg[row][0];
                PHt[row][1] <= P_reg[row][1];

                if (row == 3) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end else begin
                    row <= row + 1'b1;
                end

            end

        end
    end

endmodule