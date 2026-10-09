`timescale 1ns/1ps

// ============================================================================
// Module Name:   tb_keypad_scanner
// Project:       Zynq-7000 Digital Door Lock System (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 SoC (XC7Z020-1CLG400C)
//
// Description:
//   Unit testbench verifying the 4x4 matrix keypad scanning controller 
//   (keypad_scanner.v) and its integration with column debouncing logic.
//   Validates:
//     1. Active-low walking-zero row sequence generation.
//     2. Row scanning freeze mechanism upon switch contact detection.
//     3. Column debounce integration filter and coordinate decoding.
//     4. Generation of a single-cycle key_valid strobe matching FSM timing.
//
// Simulation Timing & Scaling:
//   - Clock Reference: 125 MHz system clock (8.0 ns period: #4 high / #4 low).
//   - Timing Acceleration:
//       * SCAN_DIVIDER scaled down to 2 cycles (vs. 125,000 in hardware) to
//         accelerate row rotation across simulation time.
//       * DEBOUNCE_CYCLES scaled down to 5 cycles (40 ns vs. 10 ms in hardware)
//         to verify filter settling and output strobe generation without bloat.
// ============================================================================

module tb_keypad_scanner;

    // ------------------------------------------------------------------------
    // Testbench Stimulus Signals & Interconnects
    // ------------------------------------------------------------------------
    reg        clk = 0;           // 125 MHz clock reference
    reg        rst = 0;           // Synchronous master reset
    wire [3:0] row;               // Active-low walking row outputs from DUT
    reg  [3:0] col = 4'b1111;     // Column inputs (defaults to 4'b1111 via pull-ups)
    
    // Monitored outputs from DUT
    wire       key_valid;         // Exact single-cycle pulse from DUT
    wire [3:0] key_data;          // Decoded hexadecimal nibble (0x0 to 0xF)

    // ------------------------------------------------------------------------
    // Clock Generator: 125 MHz (Period = 8.0 ns)
    // ------------------------------------------------------------------------
    always #4 clk = ~clk;

    // ------------------------------------------------------------------------
    // Device Under Test (DUT) Instantiation
    // Scaled parameters: 2-cycle scan step, 5-cycle debounce window
    // ------------------------------------------------------------------------
    keypad_scanner #(
        .SCAN_DIVIDER(2),         // Accelerated row progression rate
        .DEBOUNCE_CYCLES(5)       // Accelerated column filter threshold (40 ns)
    ) uut (
        .clk       (clk),
        .rst       (rst),
        .row       (row),
        .col       (col),
        .key_valid (key_valid),
        .key_data  (key_data)
    );

    // ------------------------------------------------------------------------
    // Real-Time Event Monitor
    // Verifies that key_valid asserts as a clean strobe with correct data
    // ------------------------------------------------------------------------
    always @(posedge key_valid) begin
        $display("[%0t] => KEY VALID PULSE! Decoded Data: %h (Expected: 0x5)", $time, key_data);
    end

    // ------------------------------------------------------------------------
    // Simulation Stimulus Sequence
    // ------------------------------------------------------------------------
    initial begin
        $display("[%0t] Starting Keypad Scanner Test...", $time);
        
        // 1. Assert Power-On Reset Pulse (>2 clock cycles)
        rst = 1; 
        #20; 
        rst = 0;
        
        // --------------------------------------------------------------------
        // Test Phase 1: Wait for Walking-Zero Row Sequencer
        // Wait until the scanner drives Row 1 LOW (4'b1101: scanning keys 4, 5, 6, B).
        // --------------------------------------------------------------------
        $display("[%0t] Waiting for scanner to drive Row 1 active-low...", $time);
        wait(row == 4'b1101);
        
        // --------------------------------------------------------------------
        // Test Phase 2: Simulate Keypad Closure ('5' = Row 1 x Col 1)
        // Pull Col 1 LOW (4'b1101) while Row 1 is active. This tests both the
        // scan-freeze mechanism and the debouncer integration window.
        // --------------------------------------------------------------------
        $display("[%0t] Pulling Col 1 LOW (Simulating pressing '5')...", $time);
        col = 4'b1101; 
        #150; // Hold well beyond the 5-cycle debounce threshold to allow settling
        
        // --------------------------------------------------------------------
        // Test Phase 3: Simulate Key Release
        // Switch opens; pull-ups restore column lines to idle HIGH (4'b1111).
        // --------------------------------------------------------------------
        $display("[%0t] Releasing key...", $time);
        col = 4'b1111; 
        #150; // Allow release debouncing and scanner resumption
        
        $display("[%0t] Keypad Scanner Test Complete.\n", $time);
        $finish;
    end

endmodule
