`timescale 1ns / 1ps

// ============================================================================
// Testbench:     debouncer_tb
// Project:       Multi-Floor FPGA Elevator Controller (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 SoC (XC7Z020-1CLG400C)
//
// Verification Scope & System Context:
//   This self-checking testbench validates the input-conditioning layer of the
//   elevator control system ('debouncer.v'). In physical operation, mechanical
//   contact bounce across push buttons (BTN0-BTN3) and slide switches (SW0-SW1)
//   can inject high-frequency electrical transients into the 125 MHz clock domain.
//   Left unfiltered, this chatter would trigger spurious floor calls and corrupt
//   FSM state transitions.
//
// Verification Methodology:
//   1. Simulation Speed-Up Scaling:
//      - The hardware parameter 'DEBOUNCE_LIMIT' is scaled down from its physical
//        value of 1,250,000 cycles (10 ms at 125 MHz) to 10 cycles. This enables
//        exhaustive cycle-accurate verification in nanoseconds of simulation time
//        without altering internal counter-comparison mechanics.
//
//   2. High-Frequency Contact Chatter Injection:
//      - Injects sub-threshold, non-monotonic bursts during both mechanical press
//        and mechanical release phases to confirm strict noise rejection.
//
//   3. Continuous Stability Qualification:
//      - Asserts steady states exceeding the parameterized threshold (#120 ns
//        vs. required #80 ns) to verify output assertion and clearance.
//
// Timing Derivation:
//   Clock Period (T_clk) = 8.0 ns (always #4 clk = ~clk; -> 125 MHz equivalent)
//   Threshold: 10 clock cycles = 80.0 ns
// ============================================================================

module debouncer_tb;

    // ------------------------------------------------------------------------
    // Testbench Stimulus Signals & DUT Output Probes
    // ------------------------------------------------------------------------
    reg  clk;        // Emulated 125 MHz system clock generator
    reg  rst;        // Synchronous/asynchronous active-high reset stimulus
    reg  btn_in;     // Simulated raw mechanical switch contact (with bounce)
    wire btn_out;    // Conditioned glitch-free output from DUT

    // ------------------------------------------------------------------------
    // Device Under Test (DUT) Instantiation
    // Overriding physical 10 ms counter to 10 cycles for simulation efficiency
    // ------------------------------------------------------------------------
    debouncer #(
        .DEBOUNCE_LIMIT(10)
    ) uut (
        .clk     (clk),
        .rst     (rst),
        .btn_in  (btn_in),
        .btn_out (btn_out)
    );

    // ------------------------------------------------------------------------
    // Clock Generator: 125 MHz System Clock Emulation
    // Toggle every 4 ns -> Period = 8 ns -> Frequency = 125 MHz
    // ------------------------------------------------------------------------
    always #4 clk = ~clk;

    // ------------------------------------------------------------------------
    // Main Verification Stimulus Sequence
    // ------------------------------------------------------------------------
    initial begin
        $display("==================================================");
        $display("[Time %0t ns] STARTING DEBOUNCER TEST", $time);$display("==================================================");
        
        // --------------------------------------------------------------------
        // Step 1: Quiescent State Initialization & Hardware Reset
        // --------------------------------------------------------------------
        clk    = 0;
        rst    = 1;
        btn_in = 0;

        #20;
        rst    = 0; // Release system reset
        #20;

        // --------------------------------------------------------------------
        // Step 2: Inject Mechanical Switch Chatter on Key PRESS
        // Introduce rapid non-monotonic edges (< 10 consecutive clock cycles)
        // --------------------------------------------------------------------
        $display("[Time %0t ns] Simulating mechanical contact bounce on PRESS...", $time);
        btn_in = 1; #16; // Stable for 2 cycles (sub-threshold)
        btn_in = 0; #24; // Glitch low for 3 cycles
        btn_in = 1; #32; // Glitch high for 4 cycles
        btn_in = 0; #16; // Return low for 2 cycles

        // Validation: Output must reject transients and remain logic 0
        if (btn_out == 0)
            $display("[PASS] Button output cleanly stayed 0 during input noise.");
        else
            $display("[FAIL] Glitch detected! Output went high prematurely.");

        // --------------------------------------------------------------------
        // Step 3: Sustained Physical Press (Exceeding Debounce Window)
        // Hold input high for 15 cycles (120 ns > 80 ns required limit)
        // --------------------------------------------------------------------
        $display("[Time %0t ns] Holding button stable for >10 cycles...", $time);
        btn_in = 1;
        #120; 

        // Validation: Output must now register a clean, qualified logic 1
        if (btn_out == 1)
            $display("[PASS] Button output successfully registered 1 after stability.");
        else
            $display("[FAIL] Button output failed to transition to 1.");

        // --------------------------------------------------------------------
        // Step 4: Inject Mechanical Switch Chatter on Key RELEASE
        // Simulate contact bounce during physical button spring-back
        // --------------------------------------------------------------------
        $display("[Time %0t ns] Simulating mechanical contact bounce on RELEASE...", $time);
        btn_in = 0; #16; // Brief drop for 2 cycles
        btn_in = 1; #24; // Contact bounce-back high for 3 cycles
        btn_in = 0; #32; // Intermittent bounce low for 4 cycles
        btn_in = 1; #16; // Transitory re-connection high for 2 cycles

        // Validation: Output must hold logic 1 until low state qualifies fully
        if (btn_out == 1)
            $display("[PASS] Button output cleanly stayed 1 during release noise.");
        else
            $display("[FAIL] Glitch detected! Output dropped prematurely.");

        // --------------------------------------------------------------------
        // Step 5: Sustained Physical Release (Return to Quiescent Zero)
        // Hold input low for 15 cycles (120 ns > 80 ns required limit)
        // --------------------------------------------------------------------
        $display("[Time %0t ns] Leaving button stable low...", $time);
        btn_in = 0;
        #120;

        // Validation: Output must de-assert back to clean logic 0
        if (btn_out == 0)
            $display("[PASS] Button output successfully returned to 0.");
        else
            $display("[FAIL] Button output failed to clear to 0.");

        $display("==================================================");
        $display("[Time %0t ns] DEBOUNCER TEST COMPLETE", $time);$display("==================================================");
        $finish;
    end

endmodule
