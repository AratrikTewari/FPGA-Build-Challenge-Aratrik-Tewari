`timescale 1ns / 1ps

module tb_covariance_predict;

    logic clk;
    logic rst;
    logic start;

    logic maneuver_flag; // NEW

    logic signed [19:0] F_mat [0:3][0:3];
    logic signed [19:0] P_in  [0:3][0:3];
    logic signed [19:0] Q_in  [0:3][0:3]; // ADDED MISSING PORT

    logic signed [19:0] P_pred   [0:3][0:3]; // ADDED MISSING PORT
    logic signed [19:0] FPFt_out [0:3][0:3];

    logic busy;
    logic done;

    integer i;
    integer j;
    integer errors;

    logic signed [19:0] expected_FPFt [0:3][0:3];

    covariance_predict dut (
        .clk           (clk),
        .rst           (rst),
        .start         (start),
        .maneuver_flag (maneuver_flag),
        .F_mat         (F_mat),
        .P_in          (P_in),
        .Q_in          (Q_in),
        .P_pred        (P_pred),
        .FPFt_out      (FPFt_out),
        .busy          (busy),
        .done          (done)
    );

    // 100 MHz clock
    always #5 clk = ~clk;

    initial begin

        clk = 1'b0;
        rst = 1'b1;
        start = 1'b0;
        maneuver_flag = 1'b0;
        errors = 0;

        // F matrix in Q8.12
        F_mat = '{
            '{4096,    0,   41,    0},
            '{   0, 4096,    0,   41},
            '{   0,    0, 4096,    0},
            '{   0,    0,    0, 4096}
        };

        // P matrix in Q8.12
        P_in = '{
            '{8192,  819, 2048,  410},
            '{ 819,12288,  410, 1638},
            '{2048,  410, 6144, 1229},
            '{ 410, 1638, 1229, 8192}
        };

        // Q matrix in Q8.12 (Base values)
        Q_in = '{
            '{  41,    0,    0,    0},
            '{   0,   41,    0,    0},
            '{   0,    0,  410,    0},
            '{   0,    0,    0,  410}
        };

        // Expected FPFt
        expected_FPFt = '{
            '{8233,  827, 2109,  422},
            '{ 827,12321,  422, 1720},
            '{2109,  422, 6144, 1229},
            '{ 422, 1720, 1229, 8192}
        };

        #20;
        rst = 1'b0;

        // =======================================================
        // TEST 1: Normal Operation (maneuver_flag = 0)
        // =======================================================
        #10;
        start = 1'b1;
        #10;
        start = 1'b0;

        while (!done) @(negedge clk);

        $display("\n==============================================");
        $display("       NORMAL OPERATION TEST");
        $display("==============================================");

        for (i = 0; i < 4; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                if (FPFt_out[i][j] !== expected_FPFt[i][j]) begin
                    $display("FAIL FPFt[%0d][%0d]: expected=%0d got=%0d", i, j, expected_FPFt[i][j], FPFt_out[i][j]);
                    errors = errors + 1;
                end
                
                // P_pred should be FPFt + Q_in
                if (P_pred[i][j] !== (expected_FPFt[i][j] + Q_in[i][j])) begin
                    $display("FAIL P_pred[%0d][%0d]: expected=%0d got=%0d", i, j, (expected_FPFt[i][j] + Q_in[i][j]), P_pred[i][j]);
                    errors = errors + 1;
                end
            end
        end

        // =======================================================
        // TEST 2: Maneuver Detected (maneuver_flag = 1)
        // =======================================================
        #20;
        maneuver_flag = 1'b1;
        start = 1'b1;
        #10;
        start = 1'b0;

        while (!done) @(negedge clk);

        $display("\n==============================================");
        $display("       MANEUVER DETECTED TEST (Q SCALED 4x)");
        $display("==============================================");

        for (i = 0; i < 4; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                // P_pred should be FPFt + (Q_in * 4)
                if (P_pred[i][j] !== (expected_FPFt[i][j] + (Q_in[i][j] <<< 2))) begin
                    $display("FAIL P_pred[%0d][%0d] SCALE: expected=%0d got=%0d", i, j, (expected_FPFt[i][j] + (Q_in[i][j] <<< 2)), P_pred[i][j]);
                    errors = errors + 1;
                end
            end
        end

        $display("\nCOVARIANCE PREDICT TEST COMPLETE. ERRORS: %0d", errors);
        if (errors == 0) $display("PASS: RTL dynamically scales Q-matrix correctly.");
        else $display("FAIL: RTL mismatch.");
        $finish;
    end
endmodule