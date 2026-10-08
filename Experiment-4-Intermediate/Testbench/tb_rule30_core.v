// ============================================================================
// tb_rule30_core.v
// Self-checking testbench: loads expected states produced by golden_model.py
// (via $readmemh) and compares them against rule30_core's output every cycle.
//
// Run flow (see hardware/ip_repo/rule30_axi_v1_0/tb/README or scripts):
//   1) python3 golden_model.py --seed 0xDEADBEEF --generations 32 \
//        --gen_vectors expected.hex
//   2) iverilog -o sim ../hdl/rule30_core.v tb_rule30_core.v
//   3) vvp sim
// ============================================================================
`timescale 1ns/1ps

module tb_rule30_core;

    localparam WIDTH       = 32;
    localparam GENERATIONS = 32;
    localparam SEED        = 32'hDEADBEEF;

    reg                   clk = 0;
    reg                   rst_n = 0;
    reg                   load_seed = 0;
    reg  [WIDTH-1:0]      seed_in = 0;
    wire [WIDTH-1:0]      prng_out;

    reg  [WIDTH-1:0]      expected [0:GENERATIONS];
    integer               gen_idx;
    integer               errors;

    rule30_core #(.WIDTH(WIDTH)) dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .load_seed (load_seed),
        .seed_in   (seed_in),
        .prng_out  (prng_out)
    );

    // 100 MHz-equivalent clock (period value is arbitrary for functional sim)
    always #5 clk = ~clk;

    initial begin
        errors = 0;

        // Load the golden-model expected states (see instructions above to generate)
        $readmemh("expected.hex", expected);

        // Reset
        rst_n = 0;
        load_seed = 0;
        @(posedge clk); @(posedge clk);
        rst_n = 1;

        // Load the seed (generation 0)
        @(negedge clk);
        load_seed = 1;
        seed_in   = SEED;
        @(posedge clk);
        @(negedge clk);
        load_seed = 0;

        // Generation 0 check (right after seed load)
        if (prng_out !== expected[0]) begin
            $display("MISMATCH gen 0: got 0x%08x expected 0x%08x", prng_out, expected[0]);
            errors = errors + 1;
        end else begin
            $display("OK       gen 0: 0x%08x", prng_out);
        end

        // Step through the remaining generations, one per clock
        for (gen_idx = 1; gen_idx <= GENERATIONS; gen_idx = gen_idx + 1) begin
            @(posedge clk);
            @(negedge clk); // settle
            if (prng_out !== expected[gen_idx]) begin
                $display("MISMATCH gen %0d: got 0x%08x expected 0x%08x",
                          gen_idx, prng_out, expected[gen_idx]);
                errors = errors + 1;
            end else begin
                $display("OK       gen %0d: 0x%08x", gen_idx, prng_out);
            end
        end

        if (errors == 0)
            $display("\nPASS: all %0d generations match golden model.", GENERATIONS + 1);
        else
            $display("\nFAIL: %0d mismatches out of %0d generations.", errors, GENERATIONS + 1);

        $finish;
    end

endmodule
