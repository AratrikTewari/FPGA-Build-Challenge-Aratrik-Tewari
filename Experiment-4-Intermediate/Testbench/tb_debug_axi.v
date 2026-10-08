`timescale 1ns/1ps
module tb_debug_axi;
    localparam integer C_S_AXI_DATA_WIDTH = 32;
    localparam integer C_S_AXI_ADDR_WIDTH = 4;
    reg s00_axi_aclk=0; reg s00_axi_aresetn=0;
    reg [C_S_AXI_ADDR_WIDTH-1:0] s00_axi_awaddr=0; reg [2:0] s00_axi_awprot=0; reg s00_axi_awvalid=0; wire s00_axi_awready;
    reg [C_S_AXI_DATA_WIDTH-1:0] s00_axi_wdata=0; reg [(C_S_AXI_DATA_WIDTH/8)-1:0] s00_axi_wstrb=4'hF; reg s00_axi_wvalid=0; wire s00_axi_wready;
    wire [1:0] s00_axi_bresp; wire s00_axi_bvalid; reg s00_axi_bready=0;
    reg [C_S_AXI_ADDR_WIDTH-1:0] s00_axi_araddr=0; reg [2:0] s00_axi_arprot=0; reg s00_axi_arvalid=0; wire s00_axi_arready;
    wire [C_S_AXI_DATA_WIDTH-1:0] s00_axi_rdata; wire [1:0] s00_axi_rresp; wire s00_axi_rvalid; reg s00_axi_rready=0;
    rule30_axi_v1_0 #( .C_S00_AXI_DATA_WIDTH(C_S_AXI_DATA_WIDTH), .C_S00_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH) ) dut (
        .s00_axi_aclk(s00_axi_aclk), .s00_axi_aresetn(s00_axi_aresetn),
        .s00_axi_awaddr(s00_axi_awaddr), .s00_axi_awprot(s00_axi_awprot), .s00_axi_awvalid(s00_axi_awvalid), .s00_axi_awready(s00_axi_awready),
        .s00_axi_wdata(s00_axi_wdata), .s00_axi_wstrb(s00_axi_wstrb), .s00_axi_wvalid(s00_axi_wvalid), .s00_axi_wready(s00_axi_wready),
        .s00_axi_bresp(s00_axi_bresp), .s00_axi_bvalid(s00_axi_bvalid), .s00_axi_bready(s00_axi_bready),
        .s00_axi_araddr(s00_axi_araddr), .s00_axi_arprot(s00_axi_arprot), .s00_axi_arvalid(s00_axi_arvalid), .s00_axi_arready(s00_axi_arready),
        .s00_axi_rdata(s00_axi_rdata), .s00_axi_rresp(s00_axi_rresp), .s00_axi_rvalid(s00_axi_rvalid), .s00_axi_rready(s00_axi_rready)
    );
    always #5 s00_axi_aclk = ~s00_axi_aclk;
    initial begin
        repeat (3) @(posedge s00_axi_aclk);
        s00_axi_aresetn = 1;
        s00_axi_awaddr = 4'h0; s00_axi_awvalid=1; s00_axi_wdata=32'hDEADBEEF; s00_axi_wvalid=1; s00_axi_bready=1;
        repeat (5) begin @(posedge s00_axi_aclk); $display("cycle=%0d awready=%b wready=%b bvalid=%b rvalid=%b rdata=%h", $time/10, s00_axi_awready, s00_axi_wready, s00_axi_bvalid, s00_axi_rvalid, s00_axi_rdata); end
        s00_axi_awvalid=0; s00_axi_wvalid=0; s00_axi_bready=0;
        s00_axi_araddr=4'h4; s00_axi_arvalid=1; s00_axi_rready=1;
        repeat (12) begin @(posedge s00_axi_aclk); $display("cycle=%0d arready=%b rvalid=%b rdata=%h prng=%h", $time/10, s00_axi_arready, s00_axi_rvalid, s00_axi_rdata, dut.rule30_axi_v1_0_S00_AXI_inst.u_rule30_core.prng_out); end
        $finish;
    end
endmodule
