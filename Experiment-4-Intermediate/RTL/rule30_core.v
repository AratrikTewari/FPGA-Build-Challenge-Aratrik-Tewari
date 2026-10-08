// ============================================================================
// rule30_core.v
// Elementary Cellular Automaton Rule 30 PRNG core, circular boundary.
//
//   next(i) = left(i) XOR ( center(i) OR right(i) )
//   left(i)  = state[(i+1) mod WIDTH]
//   right(i) = state[(i-1) mod WIDTH]
//
// This MUST match hardware/ip_repo/.../tb/golden_model.py bit-for-bit.
// ============================================================================
module rule30_core #(
    parameter integer WIDTH = 32
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  load_seed,
    input  wire [WIDTH-1:0]      seed_in,
    output reg  [WIDTH-1:0]      prng_out
);

    wire [WIDTH-1:0] next_state;

    genvar i;
    generate
        for (i = 0; i < WIDTH; i = i + 1) begin : ca_gen
            wire left_bit, center_bit, right_bit;

            // Circular boundary: index arithmetic wraps using modulo WIDTH
            assign left_bit   = prng_out[(i + 1) % WIDTH];
            assign center_bit = prng_out[i];
            assign right_bit  = prng_out[(i - 1 + WIDTH) % WIDTH];

            // Rule 30: left XOR (center OR right)
            assign next_state[i] = left_bit ^ (center_bit | right_bit);
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prng_out <= {WIDTH{1'b0}};
        end else if (load_seed) begin
            prng_out <= seed_in;
        end else begin
            prng_out <= next_state;
        end
    end

endmodule
