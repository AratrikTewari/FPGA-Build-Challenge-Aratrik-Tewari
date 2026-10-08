`timescale 1ns / 1ps

module debouncer_tb;
    reg clk;
    reg rst;
    reg btn_in;
    wire btn_out;

    debouncer #(
        .DEBOUNCE_LIMIT(10)
    ) uut (
        .clk(clk),
        .rst(rst),
        .btn_in(btn_in),
        .btn_out(btn_out)
    );

    always #4 clk = ~clk;

    initial begin
        $display("==================================================");
        $display("[Time %0t ns] STARTING DEBOUNCER TEST", $time);
        $display("==================================================");
        
        clk = 0;
        rst = 1;
        btn_in = 0;

        #20;
        rst = 0;
        #20;

        $display("[Time %0t ns] Simulating mechanical contact bounce on PRESS...", $time);
        btn_in = 1; #16; 
        btn_in = 0; #24; 
        btn_in = 1; #32; 
        btn_in = 0; #16; 

        if (btn_out == 0)
            $display("[PASS] Button output cleanly stayed 0 during input noise.");
        else
            $display("[FAIL] Glitch detected! Output went high prematurely.");

        $display("[Time %0t ns] Holding button stable for >10 cycles...", $time);
        btn_in = 1;
        #120; 

        if (btn_out == 1)
            $display("[PASS] Button output successfully registered 1 after stability.");
        else
            $display("[FAIL] Button output failed to transition to 1.");

        $display("[Time %0t ns] Simulating mechanical contact bounce on RELEASE...", $time);
        btn_in = 0; #16;
        btn_in = 1; #24;
        btn_in = 0; #32;
        btn_in = 1; #16;

        if (btn_out == 1)
            $display("[PASS] Button output cleanly stayed 1 during release noise.");
        else
            $display("[FAIL] Glitch detected! Output dropped prematurely.");

        $display("[Time %0t ns] Leaving button stable low...", $time);
        btn_in = 0;
        #120;

        if (btn_out == 0)
            $display("[PASS] Button output successfully returned to 0.");
        else
            $display("[FAIL] Button output failed to clear to 0.");

        $display("==================================================");
        $display("[Time %0t ns] DEBOUNCER TEST COMPLETE", $time);
        $display("==================================================");
        $finish;
    end
endmodule