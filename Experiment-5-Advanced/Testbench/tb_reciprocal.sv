`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_reciprocal.sv
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Unit testbench for the hardware division/reciprocal accelerator (e.g. Newton-Raphson or LUT-based), essential for matrix inversion.
//
// Architectural Hierarchy:
//   - Instantiated within the broader Experiment 5 RTL/Verification ecosystem.
//   - Synthesizable for PL (Programmable Logic) deployment unless marked as TB.
// ============================================================================
module tb_reciprocal;
    logic clk=0, rst=1, start=0;
    logic signed [19:0] x_in;
    logic signed [15:0] reciprocal_out;
    logic busy, done;
    integer fd, count=0, errors=0, expected, actual, x;

    reciprocal dut(
        .clk(clk), .rst(rst), .start(start), .x_in(x_in),
        .reciprocal_out(reciprocal_out), .busy(busy), .done(done)
    );

    always #5 clk = ~clk;

    task automatic run_one(input integer raw_x, input integer expected_raw);
        begin
            @(negedge clk);
            while (busy) @(negedge clk);
            x_in = raw_x;
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            while (!done) @(negedge clk);
            actual = reciprocal_out;
            if (actual !== expected_raw) begin
                errors = errors + 1;
                if (errors <= 20)
                    $display("FAIL x=%05h expected=%04h got=%04h",
                             raw_x,expected_raw,actual);
            end
            count = count + 1;
        end
    endtask

    initial begin
        x_in = '0;
        repeat (4) @(negedge clk);
        rst = 1'b0;

        // Vivado imports the .mem into the XSim working directory.
        fd = $fopen("reciprocal_vectors.mem","r");
        if (fd == 0) begin
            $display("ERROR: cannot open reciprocal_vectors.mem");
            $finish;
        end

        while (!$feof(fd))
            if ($fscanf(fd,"%h %h\n",x,expected) == 2)
                run_one(x,expected);

        $fclose(fd);
        $display("");
        $display("RECIPROCAL TESTS: %0d, ERRORS: %0d",count,errors);
        if (errors == 0)
            $display("PASS: reciprocal RTL matches golden vectors.");
        else
            $display("FAIL: reciprocal RTL does not match golden vectors.");
        $finish;
    end
endmodule
