`timescale 1ns/1ps

module tb_keypad_scanner;
    reg clk = 0;
    reg rst = 0;
    wire [3:0] row;
    reg [3:0] col = 4'b1111;
    
    wire key_valid;
    wire [3:0] key_data;

    always #4 clk = ~clk;

    keypad_scanner #(.SCAN_DIVIDER(2), .DEBOUNCE_CYCLES(5)) uut (
        .clk(clk),
        .rst(rst),
        .row(row),
        .col(col),
        .key_valid(key_valid),
        .key_data(key_data)
    );

    always @(posedge key_valid) begin
        $display("[%0t] => KEY VALID PULSE! Decoded Data: %h", $time, key_data);
    end

    initial begin
        $display("[%0t] Starting Keypad Scanner Test...", $time);
        rst = 1; #20 rst = 0;
        
        $display("[%0t] Waiting for scanner to drive Row 1 active-low...", $time);
        wait(row == 4'b1101);
        
        $display("[%0t] Pulling Col 1 LOW (Simulating pressing '5')...", $time);
        col = 4'b1101; 
        #150; 
        
        $display("[%0t] Releasing key...", $time);
        col = 4'b1111; 
        #150;
        
        $display("[%0t] Keypad Scanner Test Complete.\n", $time);
        $finish;
    end
endmodule