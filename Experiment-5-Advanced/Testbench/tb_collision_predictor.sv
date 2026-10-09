`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_collision_predictor.sv
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Verifies the forward-time projection logic of the collision detection system, predicting TTI (Time to Intersection).
//
// Architectural Hierarchy:
//   - Instantiated within the broader Experiment 5 RTL/Verification ecosystem.
//   - Synthesizable for PL (Programmable Logic) deployment unless marked as TB.
// ============================================================================
module tb_collision_predictor;
    logic clk=0, rst=1, start=0;
    
    // States
    logic signed [19:0] ax, ay, avx, avy;
    logic signed [19:0] bx, by, bvx, bvy;
    
    // Outputs
    logic coll, early, busy, done;
    logic [6:0] risk;

    collision_predictor dut (
        .clk(clk), .rst(rst), .start(start),
        .state_a_x(ax), .state_a_y(ay), .state_a_vx(avx), .state_a_vy(avy),
        .state_b_x(bx), .state_b_y(by), .state_b_vx(bvx), .state_b_vy(bvy),
        .collision_predicted(coll), .early_exit(early), .coarse_risk(risk),
        .busy(busy), .done(done)
    );

    always #5 clk = ~clk;

    // Helper to convert standard floats to your rigid Q8.12 format
    function automatic logic signed [19:0] q(input real val);
        return int'(val * 4096.0);
    endfunction

    task run_test(
        input string name,
        input real a_x, input real a_y, input real a_vx, input real a_vy,
        input real b_x, input real b_y, input real b_vx, input real b_vy
    );
        begin
            @(negedge clk);
            ax=q(a_x); ay=q(a_y); avx=q(a_vx); avy=q(a_vy);
            bx=q(b_x); by=q(b_y); bvx=q(b_vx); bvy=q(b_vy);
            start = 1;
            @(negedge clk);
            start = 0;
            while(!done) @(negedge clk);
            $display("Test: %-15s | Collide: %b | Early Exit: %b | Risk: %0d%%", 
                     name, coll, early, risk);
        end
    endtask

    initial begin
        repeat(4) @(negedge clk);
        rst = 0;
        
        $display("--- Starting Hardware Early Exit Tests ---");
        
        // 1. Head-On: Moving towards each other, will hit within 2.0s
        run_test("Head-On",       0.0, 0.0,  1.0, 0.0,    3.0, 0.0, -1.0, 0.0);
        
        // 2. Moving Away: Diverging paths, should trigger early exit immediately
        run_test("Moving Away",   0.0, 0.0, -1.0, 0.0,    3.0, 0.0,  1.0, 0.0);
        
        // 3. Far Miss: Converging on Y, but X distance prevents collision
        run_test("Far Miss",      0.0, 0.0,  0.0, 1.0,    5.0, 2.0,  0.0, -1.0);
        
        // 4. Horizon Miss: Head-on, but will hit AFTER the 2.0s prediction horizon
        run_test("Horizon Miss",  0.0, 0.0,  1.0, 0.0,   10.0, 0.0, -1.0, 0.0);

        $display("--- Tests Complete ---");
        $finish;
    end
endmodule
