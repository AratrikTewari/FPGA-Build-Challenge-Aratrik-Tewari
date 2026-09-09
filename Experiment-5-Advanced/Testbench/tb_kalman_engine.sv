`timescale 1ns / 1ps

module tb_kalman_engine;

    localparam integer W = 20;
    localparam integer NUM_TARGETS = 8;

    logic clk;
    logic rst;
    logic start;

    logic maneuver_in;  // NEW
    logic maneuver_out; // NEW

    logic signed [W-1:0] x_in [0:3];
    logic signed [W-1:0] P_in [0:3][0:3];
    logic signed [W-1:0] z [0:1];

    logic signed [W-1:0] F_mat [0:3][0:3];
    logic signed [W-1:0] Q [0:3][0:3];
    logic signed [W-1:0] R_diag [0:1];

    logic signed [W-1:0] x_out [0:3];
    logic signed [W-1:0] P_out [0:3][0:3];

    logic busy;
    logic done;

    integer errors;
    integer i;
    integer j;
    integer target;


    // ================================================================
    // Expected results
    // ================================================================

    integer expected [0:NUM_TARGETS-1][0:3];


    // ================================================================
    // DUT
    // ================================================================

    kalman_engine dut (
        .clk(clk),
        .rst(rst),
        .start(start),

        .maneuver_in(maneuver_in),   // MAPPED
        .maneuver_out(maneuver_out), // MAPPED

        .x_in(x_in),
        .P_in(P_in),
        .z(z),

        .F_mat(F_mat),
        .Q(Q),
        .R_diag(R_diag),

        .x_out(x_out),
        .P_out(P_out),

        .busy(busy),
        .done(done)
    );


    // ================================================================
    // Clock
    // ================================================================

    always #5 clk = ~clk;


    // ================================================================
    // Check one target
    // ================================================================

    task check_target;
        input integer target_index;

        begin

            for (i = 0; i < 4; i = i + 1) begin

                if (x_out[i] !== expected[target_index][i]) begin

                    $display(
                        "ERROR: target %0d x_out[%0d] = %0d, expected %0d",
                        target_index,
                        i,
                        x_out[i],
                        expected[target_index][i]
                    );

                    errors = errors + 1;

                end
                else begin

                    $display(
                        "PASS: target %0d x_out[%0d] = %0d",
                        target_index,
                        i,
                        x_out[i]
                    );

                end

            end

        end
    endtask


    // ================================================================
    // Configure common matrices
    // ================================================================

    task configure_common;

        begin

            // P = identity
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    P_in[i][j] = 20'sd0;
                end
            end

            P_in[0][0] = 20'sd4096;
            P_in[1][1] = 20'sd4096;
            P_in[2][2] = 20'sd4096;
            P_in[3][3] = 20'sd4096;


            // Constant velocity F
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    F_mat[i][j] = 20'sd0;
                end
            end

            F_mat[0][0] = 20'sd4096;
            F_mat[0][2] = 20'sd41;
            F_mat[1][1] = 20'sd4096;
            F_mat[1][3] = 20'sd41;
            F_mat[2][2] = 20'sd4096;
            F_mat[3][3] = 20'sd4096;


            // Q = zero
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    Q[i][j] = 20'sd0;
                end
            end

            // R = identity
            R_diag[0] = 20'sd4096;
            R_diag[1] = 20'sd4096;

        end

    endtask


    // ================================================================
    // Test
    // ================================================================

    initial begin

        clk = 1'b0;
        rst = 1'b1;
        start = 1'b0;
        maneuver_in = 1'b0; // Default to no previous maneuver
        errors = 0;

        // Expected outputs
        expected[0][0] = 40980;
        expected[0][1] = 81961;
        expected[0][2] = 4095;
        expected[0][3] = 8191;

        expected[1][0] = 22487;
        expected[1][1] = 122460;
        expected[1][2] = -8172;
        expected[1][3] = 4091;

        expected[2][0] = 60781;
        expected[2][1] = 42959;
        expected[2][2] = 12280;
        expected[2][3] = -4077;

        expected[3][0] = 100147;
        expected[3][1] = 200806;
        expected[3][2] = -40978;
        expected[3][3] = 20438;

        expected[4][0] = -1004;
        expected[4][1] = 12247;
        expected[4][2] = 4105;
        expected[4][3] = -8232;

        expected[5][0] = 30938;
        expected[5][1] = 69117;
        expected[5][2] = -12335;
        expected[5][3] = 23445;

        expected[6][0] = 390410;
        expected[6][1] = 109795;
        expected[6][2] = 81818;
        expected[6][3] = -40861;

        expected[7][0] = 2994;
        expected[7][1] = 4505;
        expected[7][2] = -981;
        expected[7][3] = 995;

        configure_common();

        #20;
        rst = 1'b0;

        // TARGET 0
        x_in[0] = 40960; x_in[1] = 81920; x_in[2] = 4096; x_in[3] = 8192;
        z[0] = 40960; z[1] = 81920;
        @(posedge clk); start = 1'b1; @(posedge clk); start = 1'b0;
        wait(done); #1; check_target(0);

        // TARGET 1
        x_in[0] = 20480; x_in[1] = 122880; x_in[2] = -8192; x_in[3] = 4096;
        z[0] = 24576; z[1] = 122000;
        @(posedge clk); start = 1'b1; @(posedge clk); start = 1'b0;
        wait(done); #1; check_target(1);

        // TARGET 2
        x_in[0] = 61440; x_in[1] = 40960; x_in[2] = 12288; x_in[3] = -4096;
        z[0] = 60000; z[1] = 45000;
        @(posedge clk); start = 1'b1; @(posedge clk); start = 1'b0;
        wait(done); #1; check_target(2);

        // TARGET 3
        x_in[0] = 102400; x_in[1] = 204800; x_in[2] = -40960; x_in[3] = 20480;
        z[0] = 98304; z[1] = 196608;
        @(posedge clk); start = 1'b1; @(posedge clk); start = 1'b0;
        wait(done); #1; check_target(3);

        // TARGET 4
        x_in[0] = -2048; x_in[1] = 16384; x_in[2] = 4096; x_in[3] = -8192;
        z[0] = 0; z[1] = 8192;
        @(posedge clk); start = 1'b1; @(posedge clk); start = 1'b0;
        wait(done); #1; check_target(4);

        // TARGET 5
        x_in[0] = 30000; x_in[1] = 70000; x_in[2] = -12345; x_in[3] = 23456;
        z[0] = 32000; z[1] = 68000;
        @(posedge clk); start = 1'b1; @(posedge clk); start = 1'b0;
        wait(done); #1; check_target(5);

        // TARGET 6
        x_in[0] = 400000; x_in[1] = 100000; x_in[2] = 81920; x_in[3] = -40960;
        z[0] = 380000; z[1] = 120000;
        @(posedge clk); start = 1'b1; @(posedge clk); start = 1'b0;
        wait(done); #1; check_target(6);

        // TARGET 7 (Expect maneuver_out = 1 here due to huge innovation)
        x_in[0] = 1000; x_in[1] = 5000; x_in[2] = -1000; x_in[3] = 1000;
        z[0] = 5000; z[1] = 4000;
        @(posedge clk); start = 1'b1; @(posedge clk); start = 1'b0;
        wait(done); #1; check_target(7);
        
        if (maneuver_out) $display("PASS: Maneuver dynamically detected on Target 7!");
        else $display("FAIL: Maneuver not detected on Target 7!");

        // Final result
        $display("\n==============================================");
        $display("8-TARGET SEQUENTIAL KALMAN ENGINE TEST");
        $display("==============================================");
        if (errors == 0) $display("PASS: all 8 sequential target updates match expected arithmetic.");
        else $display("FAIL: %0d errors detected.", errors);

        #20;
        $finish;
    end
endmodule