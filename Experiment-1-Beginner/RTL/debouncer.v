module debouncer (
    input clk,       // The 125 MHz system clock
    input rst,       // Reset signal
    input btn_in,    // The raw, bouncy signal from the physical button
    output reg btn_out // The clean, stable signal we will use
);

    // 1,250,000 clock cycles at 125MHz = 10 milliseconds of wait time
    parameter DEBOUNCE_LIMIT = 1250000; 
    
    reg [20:0] counter;
    reg q_reg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            counter <= 0;
            q_reg <= 0;
            btn_out <= 0;
        end else begin
            q_reg <= btn_in;
            
            // If the signal is changing (bouncing), reset the timer
            if (q_reg != btn_in) begin
                counter <= 0; 
            end 
            // If the signal is stable but we haven't reached 10ms yet, keep counting
            else if (counter < DEBOUNCE_LIMIT) begin
                counter <= counter + 1;
            end 
            // If the signal has been stable for 10ms, pass it to the output
            else begin
                btn_out <= q_reg;
            end
        end
    end
endmodule 