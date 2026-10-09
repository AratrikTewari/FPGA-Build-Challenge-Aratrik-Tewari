`timescale 1ns/1ps

// ============================================================================
// Module Name:   tb_debouncer
// Project:       Zynq-7000 Digital Door Lock System (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 SoC (XC7Z020-1CLG400C)
//
// Description:
//   Unit testbench verifying the digital integration filter (debouncer.v)
//   utilized by keypad_scanner.v to sanitize mechanical switch closures.
//   Simulates realistic asynchronous contact chatter on both falling edges
//   (key press / switch closure) and rising edges (key release).
//
// Verification Methodology & Timing Calibration:
//   - Clock Generation: 125 MHz system clock emulation (8 ns period: #4 high / #4 low).
//   - Parameter Scaling: The hardware implementation operates at 1,250,000 cycles
//     (10.0 ms integration window). For behavioral simulation efficiency,
//     DEBOUNCE_CYCLES is scaled down to 10 cycles (80 ns threshold) to verify
//     rejection filters and steady-state transition guarantees without simulation bloat.
// ============================================================================

module tb_debouncer;

    // ------------------------------------------------------------------------
    // Testbench Stimulus Signals & DUT Interconnects
    // ------------------------------------------------------------------------
    reg  clk      = 0;  // 125 MHz clock reference
    reg  rst      = 0;  // Synchronous system reset
    reg  noisy_in = 1;  // Mechanical switch input (defaults to HIGH via pull-up)
    wire clean_out;     // Conditioned digital output line

    // ------------------------------------------------------------------------
    // Clock Generator: 125 MHz (Period = 8.0 ns)
    // ------------------------------------------------------------------------
    always #4 clk = ~clk;

    // ------------------------------------------------------------------------
    // Device Under Test (DUT) Instantiation
    // Scaled parameter: 10 cycles threshold = 80 ns continuous stable hold
    // ------------------------------------------------------------------------
    debouncer #(
        .DEBOUNCE_CYCLES(10)
    ) uut (
        .clk       (clk),
        .rst       (rst),
        .noisy_in  (noisy_in),
        .clean_out (clean_out)
    );

    // ------------------------------------------------------------------------
    // Simulation Stimulus Sequence
    // ------------------------------------------------------------------------
    initial begin
        $display("[%0t] Starting Debouncer Test...", $time);
        $monitor("[%0t] noisy_in = %b | clean_out = %b", $time, noisy_in, clean_out);
        
        // 1. Apply Power-On Reset Pulse (20 ns pulse covers > 2 clock edges)
        rst = 1; 
        #20; 
        rst = 0;
        
        // --------------------------------------------------------------------
        // Test Phase 1: Contact Chatter during Key Press (Pulling Line LOW)
        // Rapid 10 ns toggles represent mechanical bounce; each bounce window
        // is shorter than the 80 ns debouncing threshold and must be rejected.
        // --------------------------------------------------------------------
        $display("\n[%0t] --- Simulating contact bounce (going LOW) ---", $time);
        noisy_in = 0; #10;
        noisy_in = 1; #10;
        noisy_in = 0; 
        #150; // Stable low interval exceeding the 10-cycle (80 ns) threshold
        $display("[%0t] Signal settled LOW.", $time);
        
        // --------------------------------------------------------------------
        // Test Phase 2: Contact Chatter during Key Release (Returning HIGH)
        // Mechanical switch releases; internal pull-up restores 3.3V with chatter.
        // --------------------------------------------------------------------
        $display("\n[%0t] --- Simulating contact bounce (going HIGH) ---", $time);
        noisy_in = 1; #10;
        noisy_in = 0; #10;
        noisy_in = 1; 
        #150; // Stable high interval exceeding the 10-cycle (80 ns) threshold
        $display("[%0t] Signal settled HIGH.", $time);
        
        $display("\n[%0t] Debouncer Test Complete.\n", $time);
        $finish;
    end

endmodule
