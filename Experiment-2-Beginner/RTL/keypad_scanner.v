`timescale 1ns/1ps

module keypad_scanner #(
    parameter integer SCAN_DIVIDER     = 125_000, 
    parameter integer DEBOUNCE_CYCLES  = 1_250_000
) (
    input  wire       clk,
    input  wire       rst,
    output reg  [3:0] row,        
    input  wire [3:0] col,        
    output reg        key_valid,
    output reg  [3:0] key_data
);

    // Debounce each column line
    wire [3:0] col_clean;
    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : DEBOUNCE_COLS
            debouncer #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) u_deb (
                .clk       (clk),
                .rst       (rst),
                .noisy_in  (col[i]),
                .clean_out (col_clean[i])
            );
        end
    endgenerate

    // Stop scanning if any column is actively pulled low (raw or debounced)
    wire key_detected = (col != 4'b1111) || (col_clean != 4'b1111);

    // Clock divider for scanning
    reg [16:0] div_cnt;
    reg        scan_tick;
    always @(posedge clk) begin
        if (rst) begin
            div_cnt   <= 17'd0;
            scan_tick <= 1'b0;
        end else if (!key_detected) begin
            if (div_cnt >= SCAN_DIVIDER - 1) begin
                div_cnt   <= 17'd0;
                scan_tick <= 1'b1;
            end else begin
                div_cnt   <= div_cnt + 17'd1;
                scan_tick <= 1'b0;
            end
        end else begin
            div_cnt   <= 17'd0;
            scan_tick <= 1'b0;
        end
    end

    // Rotate the active-low walking row bit
    always @(posedge clk) begin
        if (rst) begin
            row <= 4'b1110;
        end else if (scan_tick) begin
            case (row)
                4'b1110: row <= 4'b1101; // Walk to Row 1
                4'b1101: row <= 4'b1011; // Walk to Row 2
                4'b1011: row <= 4'b0111; // Walk to Row 3
                4'b0111: row <= 4'b1110; // Wrap to Row 0
                default: row <= 4'b1110;
            endcase
        end
    end

    // Direct matrix decode using {row, col_clean}
    reg [3:0] decoded;
    always @(*) begin
        case ({row, col_clean})
            // Row 0 active (4'b1110): 1, 2, 3, A
            {4'b1110, 4'b1110}: decoded = 4'h1;
            {4'b1110, 4'b1101}: decoded = 4'h2;
            {4'b1110, 4'b1011}: decoded = 4'h3;
            // Row 1 active (4'b1101): 4, 5, 6, B
            {4'b1101, 4'b1110}: decoded = 4'h4;
            {4'b1101, 4'b1101}: decoded = 4'h5;
            {4'b1101, 4'b1011}: decoded = 4'h6;
            // Row 2 active (4'b1011): 7, 8, 9, C
            {4'b1011, 4'b1110}: decoded = 4'h7;
            {4'b1011, 4'b1101}: decoded = 4'h8;
            {4'b1011, 4'b1011}: decoded = 4'h9;
            // Row 3 active (4'b0111): * 0 # D
            {4'b0111, 4'b1101}: decoded = 4'h0;
            default:            decoded = 4'hF;
        endcase
    end

    // Emit a single-cycle valid pulse upon debounced press
    wire debounced_press = (col_clean != 4'b1111);
    reg  prev_pressed;
    always @(posedge clk) begin
        if (rst) begin
            prev_pressed <= 1'b0;
            key_valid    <= 1'b0;
            key_data     <= 4'h0;
        end else begin
            key_valid <= 1'b0;
            if (debounced_press && !prev_pressed && (decoded != 4'hF)) begin
                key_valid <= 1'b1;
                key_data  <= decoded;
            end
            prev_pressed <= debounced_press;
        end
    end

endmodule