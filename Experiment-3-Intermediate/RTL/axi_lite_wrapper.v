`timescale 1ns / 1ps

// ============================================================================
// Module:        axi_lite_wrapper
// Project:       FPGA Build Challenge - Experiment 3
// Target Device: AMD Xilinx Zynq-7000 SoC
//
// System Context & Top-Level Integration:
//   This module serves as the AXI4-Lite wrapper for the Q4.12 sigmoid_pwl core.
//   It allows a processing system (like Zynq ARM) to interface with the sigmoid
//   core through standard AXI memory-mapped registers.
//
// Architectural Hierarchy:
//   1. Register Interface: Handles read/write operations over AXI4-Lite.
//   2. Sigmoid Core Wrapper: Connects internal AXI registers to the valid_in, 
//      din, dout, and valid_out signals of the pipelined sigmoid_pwl core.
// ============================================================================
module axi_lite_wrapper #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 4
)(
    input wire s_axi_aclk, input wire s_axi_aresetn,
    input wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    input wire s_axi_awvalid, output reg s_axi_awready,
    input wire [C_S_AXI_DATA_WIDTH-1:0] s_axi_wdata,
    input wire [C_S_AXI_DATA_WIDTH/8-1:0] s_axi_wstrb,
    input wire s_axi_wvalid, output reg s_axi_wready,
    output reg [1:0] s_axi_bresp, output reg s_axi_bvalid, input wire s_axi_bready,
    input wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    input wire s_axi_arvalid, output reg s_axi_arready,
    output reg [C_S_AXI_DATA_WIDTH-1:0] s_axi_rdata,
    output reg [1:0] s_axi_rresp, output reg s_axi_rvalid, input wire s_axi_rready
);
    reg [31:0] slv_reg0;
    reg [31:0] slv_reg1;
    reg result_valid, busy, start_pulse;

    // AXI4-Lite allows AW and W to arrive in either order.
    reg aw_captured, w_captured;
    reg [C_S_AXI_ADDR_WIDTH-1:0] awaddr_latched;
    reg [31:0] wdata_latched;
    reg [3:0] wstrb_latched;
    integer byte_index;

    wire signed [15:0] core_din = slv_reg0[15:0];
    wire signed [15:0] core_dout;
    wire core_valid;

    sigmoid_pwl core_inst (
        .clk(s_axi_aclk), .rst_n(s_axi_aresetn),
        .din_q4_12(core_din), .valid_in(start_pulse),
        .dout_q4_12(core_dout), .valid_out(core_valid)
    );

    // Capture independent write address/data channels, then apply the write.
    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            s_axi_awready <= 1'b0; s_axi_wready <= 1'b0;
            s_axi_bvalid <= 1'b0; s_axi_bresp <= 2'b00;
            aw_captured <= 1'b0; w_captured <= 1'b0;
            awaddr_latched <= {C_S_AXI_ADDR_WIDTH{1'b0}};
            wdata_latched <= 32'd0; wstrb_latched <= 4'd0;
            slv_reg0 <= 32'd0; start_pulse <= 1'b0;
        end else begin
            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0;
            start_pulse <= 1'b0;
            if (!aw_captured && !s_axi_bvalid && s_axi_awvalid) begin
                s_axi_awready <= 1'b1;
                aw_captured <= 1'b1;
                awaddr_latched <= s_axi_awaddr;
            end
            if (!w_captured && !s_axi_bvalid && s_axi_wvalid) begin
                s_axi_wready <= 1'b1;
                w_captured <= 1'b1;
                wdata_latched <= s_axi_wdata;
                wstrb_latched <= s_axi_wstrb;
            end
            if (aw_captured && w_captured && !s_axi_bvalid) begin
                if (awaddr_latched[3:2] == 2'b00) begin
                    for (byte_index = 0; byte_index < 4; byte_index = byte_index + 1)
                        if (wstrb_latched[byte_index])
                            slv_reg0[byte_index*8 +: 8] <= wdata_latched[byte_index*8 +: 8];
                    if (wstrb_latched[2] && wdata_latched[16]) start_pulse <= 1'b1;
                end
                aw_captured <= 1'b0;
                w_captured <= 1'b0;
                s_axi_bvalid <= 1'b1;
                s_axi_bresp <= 2'b00;
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // Result/status registers. A new command clears stale valid status.
    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            slv_reg1 <= 32'd0; result_valid <= 1'b0; busy <= 1'b0;
        end else begin
            if (start_pulse) begin
                result_valid <= 1'b0;
                busy <= 1'b1;
            end
            if (core_valid) begin
                slv_reg1 <= {16'd0, core_dout};
                result_valid <= 1'b1;
                busy <= 1'b0;
            end
        end
    end

    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            s_axi_arready <= 1'b0; s_axi_rvalid <= 1'b0;
            s_axi_rresp <= 2'b00; s_axi_rdata <= 32'd0;
        end else begin
            s_axi_arready <= 1'b0;
            if (!s_axi_rvalid && s_axi_arvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rresp <= 2'b00;
                case (s_axi_araddr[3:2])
                    2'b00: s_axi_rdata <= slv_reg0;
                    2'b01: s_axi_rdata <= slv_reg1;
                    2'b10: s_axi_rdata <= {30'd0, busy, result_valid};
                    default: s_axi_rdata <= 32'd0;
                endcase
                s_axi_rvalid <= 1'b1;
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end
endmodule

