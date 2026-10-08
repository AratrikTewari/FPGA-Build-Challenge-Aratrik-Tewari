// ============================================================================
// tb_rule30_axi_bfm.v
// AXI4-Lite bus-functional-model testbench for rule30_axi_v1_0_S00_AXI.v.
// Exercises the actual register map by driving AWADDR/WDATA and ARADDR
// handshakes directly instead of bypassing the bus and instantiating the core.
// ============================================================================
`timescale 1ns/1ps

module tb_rule30_axi_bfm;

    localparam integer C_S_AXI_DATA_WIDTH = 32;
    localparam integer C_S_AXI_ADDR_WIDTH = 4;
    localparam [31:0] SEED = 32'hDEADBEEF;
    localparam [31:0] EXPECTED_GEN1 = 32'h10a92088;

    reg                                      s00_axi_aclk = 0;
    reg                                      s00_axi_aresetn = 0;
    reg  [C_S_AXI_ADDR_WIDTH-1:0]            s00_axi_awaddr = 0;
    reg  [2:0]                               s00_axi_awprot = 0;
    reg                                      s00_axi_awvalid = 0;
    wire                                     s00_axi_awready;
    reg  [C_S_AXI_DATA_WIDTH-1:0]            s00_axi_wdata = 0;
    reg  [(C_S_AXI_DATA_WIDTH/8)-1:0]        s00_axi_wstrb = 4'hF;
    reg                                      s00_axi_wvalid = 0;
    wire                                     s00_axi_wready;
    wire [1:0]                               s00_axi_bresp;
    wire                                     s00_axi_bvalid;
    reg                                      s00_axi_bready = 0;
    reg  [C_S_AXI_ADDR_WIDTH-1:0]            s00_axi_araddr = 0;
    reg  [2:0]                               s00_axi_arprot = 0;
    reg                                      s00_axi_arvalid = 0;
    wire                                     s00_axi_arready;
    wire [C_S_AXI_DATA_WIDTH-1:0]            s00_axi_rdata;
    wire [1:0]                               s00_axi_rresp;
    wire                                     s00_axi_rvalid;
    reg                                      s00_axi_rready = 0;

    reg [31:0] read_value = 0;

    rule30_axi_v1_0 #(
        .C_S00_AXI_DATA_WIDTH (C_S_AXI_DATA_WIDTH),
        .C_S00_AXI_ADDR_WIDTH (C_S_AXI_ADDR_WIDTH)
    ) dut (
        .s00_axi_aclk    (s00_axi_aclk),
        .s00_axi_aresetn (s00_axi_aresetn),
        .s00_axi_awaddr  (s00_axi_awaddr),
        .s00_axi_awprot  (s00_axi_awprot),
        .s00_axi_awvalid (s00_axi_awvalid),
        .s00_axi_awready (s00_axi_awready),
        .s00_axi_wdata   (s00_axi_wdata),
        .s00_axi_wstrb   (s00_axi_wstrb),
        .s00_axi_wvalid  (s00_axi_wvalid),
        .s00_axi_wready  (s00_axi_wready),
        .s00_axi_bresp   (s00_axi_bresp),
        .s00_axi_bvalid  (s00_axi_bvalid),
        .s00_axi_bready  (s00_axi_bready),
        .s00_axi_araddr  (s00_axi_araddr),
        .s00_axi_arprot  (s00_axi_arprot),
        .s00_axi_arvalid (s00_axi_arvalid),
        .s00_axi_arready (s00_axi_arready),
        .s00_axi_rdata   (s00_axi_rdata),
        .s00_axi_rresp   (s00_axi_rresp),
        .s00_axi_rvalid  (s00_axi_rvalid),
        .s00_axi_rready  (s00_axi_rready),
        .leds            ()
    );

    always #5 s00_axi_aclk = ~s00_axi_aclk;

    task automatic write_seed;
        input [C_S_AXI_ADDR_WIDTH-1:0] addr;
        input [C_S_AXI_DATA_WIDTH-1:0] data;
        begin
            @(negedge s00_axi_aclk);
            s00_axi_awaddr  = addr;
            s00_axi_awvalid = 1'b1;
            s00_axi_wdata   = data;
            s00_axi_wvalid  = 1'b1;
            s00_axi_bready  = 1'b1;

            while (!(s00_axi_awready && s00_axi_wready)) begin
                @(posedge s00_axi_aclk);
            end

            #1;
            s00_axi_awvalid = 1'b0;
            s00_axi_wvalid  = 1'b0;

            while (!s00_axi_bvalid) begin
                @(posedge s00_axi_aclk);
            end

            #1;
            s00_axi_bready = 1'b0;
        end
    endtask

    task automatic read_data;
        input [C_S_AXI_ADDR_WIDTH-1:0] addr;
        output [C_S_AXI_DATA_WIDTH-1:0] data;
        begin
            @(negedge s00_axi_aclk);
            s00_axi_araddr  = addr;
            s00_axi_arvalid = 1'b1;
            s00_axi_rready  = 1'b1;

            while (!s00_axi_arready) begin
                @(posedge s00_axi_aclk);
            end

            while (!s00_axi_rvalid) begin
                @(posedge s00_axi_aclk);
            end

            #1 data = s00_axi_rdata;
            s00_axi_arvalid = 1'b0;
            @(negedge s00_axi_aclk);
            s00_axi_rready  = 1'b0;
        end
    endtask

    initial begin
        s00_axi_aresetn = 0;
        s00_axi_bready = 0;
        s00_axi_rready = 0;
        repeat (3) @(posedge s00_axi_aclk);
        s00_axi_aresetn = 1;

        write_seed(4'h0, SEED);

        // The first read address cycle after the write captures generation 1.
        read_data(4'h4, read_value);

        if (read_value !== EXPECTED_GEN1) begin
            $display("AXI BFM FAIL: read back 0x%08x from 0x04, expected 0x%08x (gen 1)",
                     read_value, EXPECTED_GEN1);
            $fatal(1, "AXI readback mismatch at generation 1");
        end

        $display("AXI BFM PASS: seed write to 0x00 and readback from 0x04 matched golden-model gen 1.");
        $finish;
    end

endmodule
