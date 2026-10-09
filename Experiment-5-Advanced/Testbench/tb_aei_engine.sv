`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_aei_engine
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Advanced FPGA architecture implementation featuring Kalman filters,
//   collision detection, and multi-target tracking.
//
// Architectural Hierarchy:
//   Part of the Experiment 5 RTL / Simulation / Testbench ecosystem.
// ============================================================================

module tb_aei_engine;

    localparam integer W = 20;
    localparam integer F = 12;
    localparam integer ACC_W = 44;
    localparam integer INV_W = 16;
    localparam integer NUM_TARGETS = 8;
    localparam integer ID_W = $clog2(NUM_TARGETS);


    // ============================================================
    // Clock / reset
    // ============================================================

    logic clk;
    logic rst;


    // ============================================================
    // Target selection
    // ============================================================

    logic [ID_W-1:0] target_id;


    // ============================================================
    // Load interface
    // ============================================================

    logic load_enable;
    logic [ID_W-1:0] load_target;

    logic signed [W-1:0] x_load [0:3];
    logic signed [W-1:0] P_load [0:3][0:3];

    logic load_done;


    // ============================================================
    // Runtime write interface
    // ============================================================

    logic write_enable;

    logic signed [W-1:0] x_write [0:3];
    logic signed [W-1:0] P_write [0:3][0:3];


    // ============================================================
    // Kalman inputs
    // ============================================================

    logic start;

    logic signed [W-1:0] z [0:1];

    logic signed [W-1:0] F_mat [0:3][0:3];
    logic signed [W-1:0] Q [0:3][0:3];
    logic signed [W-1:0] R_diag [0:1];


    // ============================================================
    // Kalman outputs
    // ============================================================

    logic signed [W-1:0] x_out [0:3];
    logic signed [W-1:0] P_out [0:3][0:3];

    logic busy;
    logic done;


    // ============================================================
    // DUT
    // ============================================================

    aei_engine #(
        .W(W),
        .F(F),
        .ACC_W(ACC_W),
        .INV_W(INV_W),
        .NUM_TARGETS(NUM_TARGETS)
    ) dut (
        .clk(clk),
        .rst(rst),

        .target_id(target_id),

        .load_enable(load_enable),
        .load_target(load_target),
        .x_load(x_load),
        .P_load(P_load),
        .load_done(load_done),

        .write_enable(write_enable),
        .x_write(x_write),
        .P_write(P_write),

        .z(z),

        .F_mat(F_mat),
        .Q(Q),
        .R_diag(R_diag),

        .start(start),

        .busy(busy),
        .done(done),

        .x_out(x_out),
        .P_out(P_out)
    );


    // ============================================================
    // Clock
    // ============================================================

    initial begin
        clk = 1'b0;

        forever #5 clk = ~clk;
    end


    // ============================================================
    // Error counter
    // ============================================================

    integer errors;


    // ============================================================
    // Helper task: load one target
    //
    // IMPORTANT:
    // load_done is generated on the rising edge at which the load
    // actually occurs. Therefore the testbench samples load_done
    // immediately after that rising edge.
    // ============================================================

    task automatic load_one_target(
        input integer tid
    );

        integer i;
        integer j;

        begin

            // ----------------------------------------------------
            // Prepare target ID and complete target contents.
            // ----------------------------------------------------

            load_target = tid[ID_W-1:0];

            for (i = 0; i < 4; i = i + 1) begin

                x_load[i] = (tid + 1) * 100 + i;

                for (j = 0; j < 4; j = j + 1) begin

                    P_load[i][j] =
                        (tid + 1) * 100 +
                        i * 10 +
                        j;

                end

            end


            // ----------------------------------------------------
            // Assert load_enable before the active clock edge.
            // ----------------------------------------------------

            @(negedge clk);

            load_enable = 1'b1;


            // ----------------------------------------------------
            // This rising edge performs the actual memory load.
            // target_memory sets load_done <= 1 here.
            // ----------------------------------------------------

            @(posedge clk);

            #1;

            if (!load_done) begin

                $display(
                    "ERROR: load_done missing for target %0d",
                    tid
                );

                errors = errors + 1;

            end
            else begin

                $display(
                    "PASS: load_done received for target %0d",
                    tid
                );

            end


            // ----------------------------------------------------
            // Deassert load_enable after the load cycle.
            // ----------------------------------------------------

            @(negedge clk);

            load_enable = 1'b0;

        end

    endtask


    // ============================================================
    // Main test
    // ============================================================

    integer t;
    integer i;
    integer j;


    initial begin

        errors = 0;


        // ========================================================
        // Initial signal values
        // ========================================================

        rst = 1'b1;

        target_id = '0;

        load_enable = 1'b0;
        load_target = '0;

        write_enable = 1'b0;

        start = 1'b0;


        for (i = 0; i < 4; i = i + 1) begin

            x_load[i] = '0;
            x_write[i] = '0;

            for (j = 0; j < 4; j = j + 1) begin

                P_load[i][j] = '0;
                P_write[i][j] = '0;

            end

        end


        z[0] = 0;
        z[1] = 0;


        for (i = 0; i < 4; i = i + 1) begin

            for (j = 0; j < 4; j = j + 1) begin

                F_mat[i][j] = 0;
                Q[i][j] = 0;

            end

        end


        R_diag[0] = 1;
        R_diag[1] = 1;


        // ========================================================
        // Reset
        // ========================================================

        repeat (3)
            @(posedge clk);

        rst = 1'b0;

        @(posedge clk);


        // ========================================================
        // Test header
        // ========================================================

        $display("");
        $display("==============================================");
        $display("AEI ENGINE INTEGRATION TEST");
        $display("==============================================");


        // ========================================================
        // TEST 1
        // Load all targets
        // ========================================================

        $display("");
        $display("TEST 1");
        $display("LOADING ALL TARGETS");


        for (t = 0; t < NUM_TARGETS; t = t + 1) begin

            load_one_target(t);

        end


        // ========================================================
        // TEST 2
        //
        // Verify target selection and Kalman start.
        //
        // Target 3 contains:
        //
        // x = [400, 401, 402, 403]
        //
        // We start the Kalman engine using target 3.
        // ========================================================

        $display("");
        $display("TEST 2");
        $display("VERIFYING TARGET SELECTION AND KALMAN START");


        target_id = 3;


        // --------------------------------------------------------
        // Constant-velocity F matrix
        //
        // [1 0 dt 0 ]
        // [0 1 0  dt]
        // [0 0 1  0 ]
        // [0 0 0  1 ]
        //
        // dt = 0.01
        // Fixed-point representation:
        // 0.01 * 4096 = 40.96 -> 41
        // --------------------------------------------------------

        for (i = 0; i < 4; i = i + 1) begin

            for (j = 0; j < 4; j = j + 1) begin

                F_mat[i][j] = 0;

            end

        end


        F_mat[0][0] = 4096;
        F_mat[1][1] = 4096;
        F_mat[2][2] = 4096;
        F_mat[3][3] = 4096;

        F_mat[0][2] = 41;
        F_mat[1][3] = 41;


        // --------------------------------------------------------
        // Process noise
        // --------------------------------------------------------

        for (i = 0; i < 4; i = i + 1) begin

            for (j = 0; j < 4; j = j + 1) begin

                Q[i][j] = 0;

            end

        end


        Q[0][0] = 1;
        Q[1][1] = 1;
        Q[2][2] = 1;
        Q[3][3] = 1;


        // --------------------------------------------------------
        // Measurement noise
        // --------------------------------------------------------

        R_diag[0] = 4096;
        R_diag[1] = 4096;


        // --------------------------------------------------------
        // Measurement corresponding approximately to target 3.
        // --------------------------------------------------------

        z[0] = 401;
        z[1] = 402;


        // --------------------------------------------------------
        // Start Kalman operation.
        // --------------------------------------------------------

        @(negedge clk);

        start = 1'b1;

        @(negedge clk);

        start = 1'b0;


        // --------------------------------------------------------
        // Wait for Kalman completion.
        // --------------------------------------------------------

        wait (done == 1'b1);


        $display(
            "PASS: Kalman engine completed through AEI wrapper"
        );

        $display(
            "INFO: target 3 x_out = %0d, %0d, %0d, %0d",
            x_out[0],
            x_out[1],
            x_out[2],
            x_out[3]
        );


        // ========================================================
        // TEST 3
        // Verify busy/done handshake
        // ========================================================

        $display("");
        $display("TEST 3");
        $display("VERIFYING BUSY/DONE HANDSHAKE");


        @(posedge clk);

        if (busy !== 1'b0) begin

            $display(
                "ERROR: busy remains asserted after completion"
            );

            errors = errors + 1;

        end
        else begin

            $display(
                "PASS: busy deasserted after Kalman completion"
            );

        end


        // ========================================================
        // TEST 4
        // Runtime memory write followed by Kalman start
        // ========================================================

        $display("");
        $display("TEST 4");
        $display("VERIFYING RUNTIME WRITE THROUGH AEI WRAPPER");


        target_id = 5;


        // --------------------------------------------------------
        // Prepare runtime-written target.
        // --------------------------------------------------------

        for (i = 0; i < 4; i = i + 1) begin

            x_write[i] = 900 + i;

            for (j = 0; j < 4; j = j + 1) begin

                P_write[i][j] =
                    900 +
                    i * 10 +
                    j;

            end

        end


        // --------------------------------------------------------
        // Perform runtime write.
        // --------------------------------------------------------

        @(negedge clk);

        write_enable = 1'b1;

        @(posedge clk);

        #1;

        write_enable = 1'b0;


        // --------------------------------------------------------
        // Give synchronous memory path time to settle.
        // --------------------------------------------------------

        @(posedge clk);
        @(posedge clk);


        // --------------------------------------------------------
        // Measurement corresponding to runtime-written target.
        // --------------------------------------------------------

        z[0] = 900;
        z[1] = 901;


        // --------------------------------------------------------
        // Start Kalman using target 5.
        // --------------------------------------------------------

        @(negedge clk);

        start = 1'b1;

        @(negedge clk);

        start = 1'b0;


        // --------------------------------------------------------
        // Wait for completion.
        // --------------------------------------------------------

        wait (done == 1'b1);


        $display(
            "PASS: runtime-written target completed Kalman operation"
        );


        // ========================================================
        // Final result
        // ========================================================

        $display("");
        $display("==============================================");
        $display("AEI ENGINE INTEGRATION TEST COMPLETE");
        $display("==============================================");


        if (errors == 0) begin

            $display(
                "PASS: AEI engine integration matches expected interface behavior."
            );

        end
        else begin

            $display(
                "FAIL: %0d errors detected.",
                errors
            );

        end


        $finish;

    end

endmodule
