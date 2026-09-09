`timescale 1ns / 1ps

module reciprocal #(
    parameter integer IN_W=20,
    parameter integer OUT_W=16,
    parameter integer INT_W=29,
    parameter integer INT_F=27,
    parameter integer LUT_ADDR_W=6
)(
    input  logic clk,
    input  logic rst,
    input  logic start,
    input  logic signed [IN_W-1:0] x_in,
    output logic signed [OUT_W-1:0] reciprocal_out,
    output logic busy,
    output logic done
);

    typedef enum logic [4:0] {
        IDLE, 
        NORMALIZE_DETECT,
        NORMALIZE_SHIFT,
        LUT, 
        LOAD_SEED,
        NR_1_MULT, 
        NR_1_MAG, 
        NR_1_ROUND,
        NR_2,
        NR_3_MULT, 
        NR_3_MAG, 
        NR_3_ROUND,
        DENORM_SHIFT,
        DENORM_MAG,
        DENORM_ROUND,
        DENORM_SIGN,
        DENORM_SAT,
        FINISH
    } state_t;
    state_t state;

    logic signed [IN_W-1:0] x_reg;
    logic signed [5:0] exponent;
    logic [4:0] shift_amt;
    logic signed [INT_W-1:0] x_norm;
    
    logic [LUT_ADDR_W-1:0] lut_addr;
    logic signed [INT_W-1:0] seed;
    logic signed [INT_W-1:0] lut_data;
    logic signed [INT_W-1:0] r_next;

    logic signed [INT_W-1:0] xr_q27_reg, correction_reg;
    logic signed [57:0] prod_xr_reg, prod_nr_reg;

    // Aggressive pipelining registers for denormalization and rounding
    logic signed [57:0] denorm_reg;
    logic signed [57:0] denorm_mag;
    logic               denorm_sign;
    logic signed [57:0] rounded_mag;
    logic               round_sign_reg;
    logic signed [57:0] signed_rounded;
    logic signed [OUT_W-1:0] output_reg;

    logic signed [57:0] rnd_mag;
    logic               rnd_sign;

    logic signed [INT_W-1:0] x_ext_29;
    logic signed [57:0] x_ext_58, seed_ext_58, correction_ext_58, r_next_ext_signed;
    logic signed [29:0] correction_30;

    localparam logic signed [29:0] TWO_Q27_30 = 30'sd268435456;

    reciprocal_lut #(.ADDR_W(LUT_ADDR_W),.DATA_W(INT_W)) lut (
        .addr(lut_addr), .data(lut_data)
    );

    function automatic signed [OUT_W-1:0] saturate_output(input signed [57:0] value);
        reg signed [57:0] max_v, min_v;
        begin
            max_v = (58'sd1 <<< (OUT_W-1)) - 1;
            min_v = -(58'sd1 <<< (OUT_W-1));
            if (value > max_v) saturate_output = {OUT_W{1'b1}};
            else if (value < min_v) saturate_output = '0;
            else saturate_output = value[OUT_W-1:0];
        end
    endfunction

    always_ff @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            x_reg <= '0;
            exponent <= '0;
            shift_amt <= '0;
            x_norm <= '0;
            lut_addr <= '0;
            seed <= '0;
            r_next <= '0;
            xr_q27_reg <= '0;
            correction_reg <= '0; 
            prod_xr_reg <= '0;
            prod_nr_reg <= '0;
            denorm_reg <= '0;
            denorm_mag <= '0;
            denorm_sign <= 1'b0;
            rounded_mag <= '0;
            round_sign_reg <= 1'b0;
            signed_rounded <= '0;
            rnd_mag <= '0;
            rnd_sign <= 1'b0;
            output_reg <= '0;
            reciprocal_out <= '0;
            busy <= 1'b0;
            done <= 1'b0;
        end else begin
            done <= 1'b0;
            case (state)
                IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        x_reg <= x_in;
                        busy <= 1'b1;
                        if (x_in <= 0) begin
                            output_reg <= '0;
                            state <= FINISH;
                        end else begin
                            state <= NORMALIZE_DETECT;
                        end
                    end
                end

                NORMALIZE_DETECT: begin
                    if      (x_reg[18]) begin exponent <=  6; shift_amt <= 9;  end
                    else if (x_reg[17]) begin exponent <=  5; shift_amt <= 10; end
                    else if (x_reg[16]) begin exponent <=  4; shift_amt <= 11; end
                    else if (x_reg[15]) begin exponent <=  3; shift_amt <= 12; end
                    else if (x_reg[14]) begin exponent <=  2; shift_amt <= 13; end
                    else if (x_reg[13]) begin exponent <=  1; shift_amt <= 14; end
                    else if (x_reg[12]) begin exponent <=  0; shift_amt <= 15; end
                    else if (x_reg[11]) begin exponent <= -1; shift_amt <= 16; end
                    else if (x_reg[10]) begin exponent <= -2; shift_amt <= 17; end
                    else if (x_reg[9])  begin exponent <= -3; shift_amt <= 18; end
                    else if (x_reg[8])  begin exponent <= -4; shift_amt <= 19; end
                    else if (x_reg[7])  begin exponent <= -5; shift_amt <= 20; end
                    else if (x_reg[6])  begin exponent <= -6; shift_amt <= 21; end
                    else if (x_reg[5])  begin exponent <= -7; shift_amt <= 22; end
                    else if (x_reg[4])  begin exponent <= -8; shift_amt <= 23; end
                    else if (x_reg[3])  begin exponent <= -9; shift_amt <= 24; end
                    else if (x_reg[2])  begin exponent <= -10; shift_amt <= 25; end
                    else if (x_reg[1])  begin exponent <= -11; shift_amt <= 26; end
                    else                begin exponent <= -12; shift_amt <= 27; end
                    state <= NORMALIZE_SHIFT;
                end

                NORMALIZE_SHIFT: begin
                    x_ext_29 = {9'b0, x_reg};
                    x_norm <= x_ext_29 <<< shift_amt;
                    state <= LUT;
                end

                LUT: begin
                    lut_addr <= x_norm[26:21];
                    state <= LOAD_SEED;
                end

                LOAD_SEED: begin
                    seed <= lut_data;
                    state <= NR_1_MULT;
                end

                NR_1_MULT: begin
                    x_ext_58 = {{29{x_norm[28]}}, x_norm};
                    seed_ext_58 = {{29{seed[28]}}, seed};
                    prod_xr_reg <= x_ext_58 * seed_ext_58;
                    state <= NR_1_MAG;
                end

                NR_1_MAG: begin
                    rnd_sign <= (prod_xr_reg < 0);
                    rnd_mag  <= (prod_xr_reg < 0) ? -prod_xr_reg : prod_xr_reg;
                    state <= NR_1_ROUND;
                end

                NR_1_ROUND: begin
                    logic round_up;
                    round_up = rnd_mag[INT_F-1] & ((|rnd_mag[INT_F-2:0]) | rnd_mag[INT_F]);
                    xr_q27_reg <= rnd_sign ? -((rnd_mag >>> INT_F) + round_up) : ((rnd_mag >>> INT_F) + round_up);
                    state <= NR_2;
                end

                NR_2: begin
                    correction_30 = TWO_Q27_30 - $signed({xr_q27_reg[28], xr_q27_reg});
                    correction_reg <= correction_30[INT_W-1:0];
                    state <= NR_3_MULT;
                end

                NR_3_MULT: begin
                    seed_ext_58 = {{29{seed[28]}}, seed};
                    correction_ext_58 = {{29{correction_reg[28]}}, correction_reg};
                    prod_nr_reg <= seed_ext_58 * correction_ext_58;
                    state <= NR_3_MAG; 
                end

                NR_3_MAG: begin
                    rnd_sign <= (prod_nr_reg < 0);
                    rnd_mag  <= (prod_nr_reg < 0) ? -prod_nr_reg : prod_nr_reg;
                    state <= NR_3_ROUND;
                end

                NR_3_ROUND: begin
                    logic round_up;
                    round_up = rnd_mag[INT_F-1] & ((|rnd_mag[INT_F-2:0]) | rnd_mag[INT_F]);
                    r_next <= rnd_sign ? -((rnd_mag >>> INT_F) + round_up) : ((rnd_mag >>> INT_F) + round_up);
                    state <= DENORM_SHIFT;
                end

                // --- PIPELINED DENORMALIZATION (Max 2-3 logic levels per clock) ---
                DENORM_SHIFT: begin
                    r_next_ext_signed = {{29{r_next[28]}}, r_next};
                    if (exponent >= 0) denorm_reg <= r_next_ext_signed >>> exponent;
                    else               denorm_reg <= r_next_ext_signed <<< (-exponent);
                    state <= DENORM_MAG;
                end

                DENORM_MAG: begin
                    denorm_sign <= (denorm_reg < 0);
                    denorm_mag  <= (denorm_reg < 0) ? -denorm_reg : denorm_reg;
                    state <= DENORM_ROUND;
                end

                DENORM_ROUND: begin
                    logic round_up;
                    round_up = denorm_mag[14] & ((|denorm_mag[13:0]) | denorm_mag[15]);
                    rounded_mag    <= (denorm_mag >>> 15) + round_up;
                    round_sign_reg <= denorm_sign;
                    state <= DENORM_SIGN;
                end

                DENORM_SIGN: begin
                    signed_rounded <= round_sign_reg ? -$signed(rounded_mag) : $signed(rounded_mag);
                    state <= DENORM_SAT;
                end

                DENORM_SAT: begin
                    output_reg <= saturate_output(signed_rounded);
                    state <= FINISH;
                end

                FINISH: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    reciprocal_out <= output_reg;
                    state <= IDLE;
                end

                default: begin
                    state <= IDLE;
                    busy <= 1'b0;
                end
            endcase
        end
    end
endmodule