// ============================================================================
// tb_rule30_core_harsh.v
// Multi-seed regression for the Rule 30 core with 3 extra seeds and 64
// generations each. This intentionally stresses the CA beyond the original
// 32-generation check and fails loudly on the first mismatch with the exact
// generation number.
// ============================================================================
`timescale 1ns/1ps

module tb_rule30_core_harsh;

    localparam WIDTH       = 32;
    localparam GENERATIONS = 64;

    reg                   clk = 0;
    reg                   rst_n = 0;
    reg                   load_seed = 0;
    reg  [WIDTH-1:0]      seed_in = 0;
    wire [WIDTH-1:0]      prng_out;

    reg  [WIDTH-1:0]      expected [0:GENERATIONS];
    integer               gen_idx;
    integer               errors;
    integer               seed_idx;

    rule30_core #(.WIDTH(WIDTH)) dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .load_seed (load_seed),
        .seed_in   (seed_in),
        .prng_out  (prng_out)
    );

    always #5 clk = ~clk;

    initial begin
        $display("Starting harsh regression checks...");

        for (seed_idx = 0; seed_idx < 3; seed_idx = seed_idx + 1) begin
            errors = 0;

            case (seed_idx)
                0: $readmemh("expected_harsh_seed_00000001.hex", expected);
                1: $readmemh("expected_harsh_seed_a5a5a5a5.hex", expected);
                default: $readmemh("expected_harsh_seed_12345678.hex", expected);
            endcase

            rst_n = 0;
            load_seed = 0;
            seed_in = 0;
            repeat (2) @(posedge clk);
            rst_n = 1;

            case (seed_idx)
                0: seed_in = 32'h00000001;
                1: seed_in = 32'hA5A5A5A5;
                default: seed_in = 32'h12345678;
            endcase

            @(negedge clk);
            load_seed = 1;
            @(posedge clk);
            @(negedge clk);
            load_seed = 0;

            if (prng_out !== expected[0]) begin
                $display("HARSH FAIL: seed=0x%08x gen 0 got=0x%08x expected=0x%08x",
                         seed_in, prng_out, expected[0]);
                $fatal(1, "Mismatch at generation 0");
            end

            for (gen_idx = 1; gen_idx <= GENERATIONS; gen_idx = gen_idx + 1) begin
                @(posedge clk);
                @(negedge clk);
                if (prng_out !== expected[gen_idx]) begin
                    $display("HARSH FAIL: seed=0x%08x divergence at gen %0d got=0x%08x expected=0x%08x",
                             seed_in, gen_idx, prng_out, expected[gen_idx]);
                    $fatal(1, "Mismatch at generation %0d", gen_idx);
                end
            end

            $display("HARSH PASS: seed=0x%08x all %0d generations matched.", seed_in, GENERATIONS + 1);
        end

        $display("PASS: all harsh regression checks matched for 3 seeds over 64 generations.");
        $finish;
    end

endmodule
