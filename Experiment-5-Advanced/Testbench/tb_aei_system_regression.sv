`timescale 1ns / 1ps

module tb_aei_system_regression;

localparam integer W = 20;
localparam integer F = 12;
localparam integer ACC_W = 44;
localparam integer INV_W = 16;
localparam integer NUM_TARGETS = 8;


// ============================================================
// Clock / reset
// ============================================================

logic clk;
logic rst;


// ============================================================
// Regression status
// ============================================================

logic regression_done;
logic regression_pass;


// ============================================================
// Clock
// ============================================================

initial begin

    clk = 1'b0;

    forever #5 clk = ~clk;

end


// ============================================================
// Reset
// ============================================================

initial begin

    rst = 1'b1;

    repeat (3)
        @(posedge clk);

    rst = 1'b0;

end


// ============================================================
// Regression DUT
// ============================================================

aei_system_regression #(
    .W(W),
    .F(F),
    .ACC_W(ACC_W),
    .INV_W(INV_W),
    .NUM_TARGETS(NUM_TARGETS)
) dut (
    .clk(clk),
    .rst(rst),

    .regression_done(regression_done),
    .regression_pass(regression_pass)
);


// ============================================================
// Wait for completion
// ============================================================

initial begin

    wait (regression_done == 1'b1);

    #10;

    $display("");
    $display("==============================================");
    $display("AEI SYSTEM REGRESSION TEST COMPLETE");
    $display("==============================================");


    if (regression_pass !== 1'b1) begin

        $display(
            "FAIL: AEI multi-frame system regression failed."
        );

    end
    else begin

        $display(
            "PASS: AEI multi-frame system regression matches expected behavior."
        );

    end


    $finish;

end


// ============================================================
// Global timeout
//
// Prevents a broken handshake from hanging simulation.
// ============================================================

initial begin

    #200000;

    $display("");
    $display(
        "ERROR: regression timeout."
    );

    $finish;

end

endmodule