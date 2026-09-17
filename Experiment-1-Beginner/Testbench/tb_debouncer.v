`timescale 1ns/1ps

module tb_debouncer;
    reg clk = 0;
    reg rst = 0;
    reg noisy_in = 1;
    wire clean_out;

    always #4 clk = ~clk;

    debouncer #(.DEBOUNCE_CYCLES(10)) uut (
        .clk(clk),
        .rst(rst),
        .noisy_in(noisy_in),
        .clean_out(clean_out)
    );

    initial begin
        $display("[%0t] Starting Debouncer Test...", $time);
        $monitor("[%0t] noisy_in = %b | clean_out = %b", $time, noisy_in, clean_out);
        
        rst = 1; #20 rst = 0;
        
        $display("\n[%0t] --- Simulating contact bounce (going LOW) ---", $time);
        noisy_in = 0; #10 noisy_in = 1; #10 noisy_in = 0; 
        #150; 
        $display("[%0t] Signal settled LOW.", $time);
        
        $display("\n[%0t] --- Simulating contact bounce (going HIGH) ---", $time);
        noisy_in = 1; #10 noisy_in = 0; #10 noisy_in = 1; 
        #150;
        $display("[%0t] Signal settled HIGH.", $time);
        
        $display("\n[%0t] Debouncer Test Complete.\n", $time);
        $finish;
    end
endmodule