`timescale 1ns / 1ps

module measurement_update #(
    parameter integer W = 20
)(
    input logic clk,
    input logic rst,
    input logic start,

    input logic signed [W-1:0] x_pred [0:3],
    input logic signed [W-1:0] z      [0:1],

    output logic signed [W-1:0] innovation [0:1],

    output logic busy,
    output logic done
);

    logic signed [W-1:0] x_pred_reg [0:3];
    logic signed [W-1:0] z_reg      [0:1];

    logic [1:0] index;

    integer i;

    always_ff @(posedge clk) begin

        if (rst) begin

            busy <= 1'b0;
            done <= 1'b0;
            index <= '0;

            for (i = 0; i < 4; i = i + 1)
                x_pred_reg[i] <= '0;

            for (i = 0; i < 2; i = i + 1)
                z_reg[i] <= '0;

            for (i = 0; i < 2; i = i + 1)
                innovation[i] <= '0;

        end else begin

            done <= 1'b0;

            if (!busy) begin

                if (start) begin

                    busy <= 1'b1;
                    index <= 2'd0;

                    for (i = 0; i < 4; i = i + 1)
                        x_pred_reg[i] <= x_pred[i];

                    for (i = 0; i < 2; i = i + 1)
                        z_reg[i] <= z[i];

                end

            end else begin

                case (index)

                    2'd0: begin
                        innovation[0] <= z_reg[0] - x_pred_reg[0];
                        index <= 2'd1;
                    end

                    2'd1: begin
                        innovation[1] <= z_reg[1] - x_pred_reg[1];

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