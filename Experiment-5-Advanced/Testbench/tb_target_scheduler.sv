`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_target_scheduler
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

module tb_target_scheduler;

    localparam integer NUM_TARGETS = 8;
    localparam integer ID_W = 3;

    logic clk;
    logic rst;
    logic start;

    logic [ID_W-1:0] target_id;

    logic memory_read_request;
    logic memory_read_valid;
    logic memory_write_enable;

    logic kalman_start;
    logic kalman_busy;
    logic kalman_done;

    logic busy;
    logic frame_done;

    integer errors;
    integer read_count;
    integer write_count;
    integer kalman_count;

    logic [ID_W-1:0] expected_target;


    // ============================================================
    // DUT
    // ============================================================

    target_scheduler #(
        .NUM_TARGETS(NUM_TARGETS),
        .ID_W(ID_W)
    ) dut (
        .clk(clk),
        .rst(rst),

        .start(start),

        .target_id(target_id),

        .memory_read_request(memory_read_request),
        .memory_read_valid(memory_read_valid),

        .memory_write_enable(memory_write_enable),

        .kalman_start(kalman_start),
        .kalman_busy(kalman_busy),
        .kalman_done(kalman_done),

        .busy(busy),
        .frame_done(frame_done)
    );


    // ============================================================
    // Clock
    // ============================================================

    always #5 clk = ~clk;


    // ============================================================
    // Behavioral memory model
    // ============================================================

    always @(posedge clk) begin

        memory_read_valid <= 1'b0;

        if (memory_read_request) begin

            if (target_id !== expected_target) begin

                $display(
                    "ERROR: memory read target = %0d, expected %0d",
                    target_id,
                    expected_target
                );

                errors = errors + 1;

            end
            else begin

                $display(
                    "PASS: READ target %0d",
                    target_id
                );

            end

            read_count = read_count + 1;

            // One-cycle memory response
            memory_read_valid <= 1'b1;

        end


        if (memory_write_enable) begin

            if (target_id !== expected_target) begin

                $display(
                    "ERROR: memory write target = %0d, expected %0d",
                    target_id,
                    expected_target
                );

                errors = errors + 1;

            end
            else begin

                $display(
                    "PASS: WRITE target %0d",
                    target_id
                );

            end

            write_count = write_count + 1;

        end

    end


    // ============================================================
    // Behavioral Kalman engine model
    // ============================================================

    always @(posedge clk) begin

        kalman_done <= 1'b0;

        if (kalman_start) begin

            if (kalman_busy) begin

                $display("ERROR: Kalman started while busy");

                errors = errors + 1;

            end

            kalman_count = kalman_count + 1;

            $display(
                "PASS: KALMAN START target %0d",
                expected_target
            );

            // Complete one cycle later
            kalman_done <= 1'b1;

        end

    end


    // ============================================================
    // Main test
    // ============================================================

    initial begin

        clk = 1'b0;
        rst = 1'b1;
        start = 1'b0;

        memory_read_valid = 1'b0;

        kalman_busy = 1'b0;
        kalman_done = 1'b0;

        errors = 0;
        read_count = 0;
        write_count = 0;
        kalman_count = 0;

        expected_target = 0;


        // --------------------------------------------------------
        // Reset
        // --------------------------------------------------------

        #20;
        rst = 1'b0;


        // --------------------------------------------------------
        // Start frame
        // --------------------------------------------------------

        @(posedge clk);
        start <= 1'b1;

        @(posedge clk);
        start <= 1'b0;


        // --------------------------------------------------------
        // Monitor target advancement
        // --------------------------------------------------------

        forever begin

            @(posedge clk);

            if (memory_write_enable) begin

                if (expected_target == NUM_TARGETS-1) begin

                    expected_target <= expected_target;

                end
                else begin

                    expected_target <= expected_target + 1'b1;

                end

            end

            if (frame_done) begin

                break;

            end

        end


        // --------------------------------------------------------
        // Final checks
        // --------------------------------------------------------

        #1;

        $display("");
        $display("==============================================");
        $display("TARGET SCHEDULER TEST COMPLETE");
        $display("==============================================");

        if (read_count != NUM_TARGETS) begin

            $display(
                "ERROR: read_count = %0d, expected %0d",
                read_count,
                NUM_TARGETS
            );

            errors = errors + 1;

        end
        else begin

            $display(
                "PASS: %0d target reads completed.",
                read_count
            );

        end


        if (write_count != NUM_TARGETS) begin

            $display(
                "ERROR: write_count = %0d, expected %0d",
                write_count,
                NUM_TARGETS
            );

            errors = errors + 1;

        end
        else begin

            $display(
                "PASS: %0d target writes completed.",
                write_count
            );

        end


        if (kalman_count != NUM_TARGETS) begin

            $display(
                "ERROR: kalman_count = %0d, expected %0d",
                kalman_count,
                NUM_TARGETS
            );

            errors = errors + 1;

        end
        else begin

            $display(
                "PASS: %0d Kalman operations completed.",
                kalman_count
            );

        end


        if (errors == 0) begin

            $display(
                "PASS: 8-target scheduler sequencing is correct."
            );

        end
        else begin

            $display(
                "FAIL: %0d errors detected.",
                errors
            );

        end


        #20;
        $finish;

    end

endmodule
