`timescale 1ns/1ps

module tb_top_module;
    reg clk = 0;
    reg btn_rst = 0;
    
    wire [3:0] kp_row;
    reg [3:0] kp_col = 4'b1111; // Keypad column pull-ups
    
    wire servo_pwm;
    wire led_r, led_g, led_b;
    
    reg [4:0] ps_key_inject = 5'd0;
    wire [3:0] ps_status_out;

    // 125MHz Clock (Period = 8ns)
    always #4 clk = ~clk; 

    // Instantiate UUT with scaled hold cycles (50 cycles = 400ns)
    top_module #(
        .UNLOCK_HOLD_CYCLES(50)
    ) uut (
        .clk           (clk), 
        .btn_rst       (btn_rst),
        .kp_row        (kp_row), 
        .kp_col        (kp_col), 
        .servo_pwm     (servo_pwm), 
        .led_r         (led_r), 
        .led_g         (led_g), 
        .led_b         (led_b),
        .ps_key_inject (ps_key_inject), 
        .ps_status_out (ps_status_out)
    );

    // Synchronous injection task: drives on negative edge to guarantee clean setup/hold
    task inject_ps_key(input [3:0] val);
    begin
        @(negedge clk);
        $display("[%0t] Software injecting key via AXI GPIO: %h", $time, val);
        ps_key_inject <= {1'b1, val};
        
        @(negedge clk);
        ps_key_inject <= 5'd0;
        
        // Wait 4 clock cycles before the next keystroke
        repeat(4) @(negedge clk);
    end
    endtask

    always @(ps_status_out) begin
        $display("[%0t] PS Status Out Changed -> Alarm: %b | RGB: %b", $time, ps_status_out[3], ps_status_out[2:0]);
    end

    // Monitor internal FSM pass buffer
    always @(posedge clk) begin
        if (uut.u_fsm.key_valid) begin
            $display("[%0t] [FSM DEBUG] Sampled key_data=%h | stored_pass=%h | digit_count=%d", 
                     $time, uut.u_fsm.key_data, uut.u_fsm.stored_pass, uut.u_fsm.digit_count);
        end
    end

    initial begin
        $display("[%0t] Starting Top Module Hybrid Test...", $time);
        
        // Apply synchronous reset on negative edge
        @(negedge clk);
        btn_rst = 1;
        repeat(5) @(negedge clk);
        btn_rst = 0;
        repeat(5) @(negedge clk);
        
        $display("\n[%0t] --- Injecting Password (1234) from PS ---", $time);
        inject_ps_key(4'h1);
        inject_ps_key(4'h2);
        inject_ps_key(4'h3);
        inject_ps_key(4'h4);
        
        // Wait long enough to observe unlock and auto-relock
        #800; 
        $display("\n[%0t] Top Module Test Complete.\n", $time);
        $finish;
    end
endmodule