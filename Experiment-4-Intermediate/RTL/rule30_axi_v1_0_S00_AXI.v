// ============================================================================
// rule30_axi_v1_0_S00_AXI.v
// AXI4-Lite slave wrapper around rule30_core.
//
// Address map (from hardware/build perspective; also see docs/address_map.md):
//   0x00  SEED_REG   (write-only from software's perspective)
//         Writing here pulses load_seed for one clock and loads seed_in.
//   0x04  DATA_REG   (read-only from software's perspective)
//         Reading here returns the CA's current 32-bit state.
//
// This is Vivado's standard "Create and Package New IP -> AXI4 Peripheral"
// template with the write/read data paths replaced to drive rule30_core.
// If you regenerate the template in Vivado, keep the AXI protocol logic
// (the FSM-ish awready/wready/bready/arready/rvalid handshake below) intact
// and only touch the two marked USER LOGIC sections.
// ============================================================================
module rule30_axi_v1_0_S00_AXI #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 4
)(
    input  wire                                 S_AXI_ACLK,
    input  wire                                 S_AXI_ARESETN,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]        S_AXI_AWADDR,
    input  wire [2:0]                           S_AXI_AWPROT,
    input  wire                                 S_AXI_AWVALID,
    output wire                                 S_AXI_AWREADY,

    input  wire [C_S_AXI_DATA_WIDTH-1:0]        S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0]    S_AXI_WSTRB,
    input  wire                                 S_AXI_WVALID,
    output wire                                 S_AXI_WREADY,

    output wire [1:0]                           S_AXI_BRESP,
    output wire                                 S_AXI_BVALID,
    input  wire                                 S_AXI_BREADY,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]        S_AXI_ARADDR,
    input  wire [2:0]                           S_AXI_ARPROT,
    input  wire                                 S_AXI_ARVALID,
    output wire                                 S_AXI_ARREADY,

    output wire [C_S_AXI_DATA_WIDTH-1:0]        S_AXI_RDATA,
    output wire [1:0]                           S_AXI_RRESP,
    output wire                                 S_AXI_RVALID,
    input  wire                                 S_AXI_RREADY,
    
    // User Ports
    output wire [3:0]                           leds
);

    // ------------------------------------------------------------------
    // AXI protocol bookkeeping (standard Vivado template pattern)
    // ------------------------------------------------------------------
    reg                              axi_awready;
    reg                              axi_wready;
    reg  [1:0]                       axi_bresp;
    reg                              axi_bvalid;
    reg [C_S_AXI_ADDR_WIDTH-1:0]     axi_awaddr;

    reg                              axi_arready;
    reg [C_S_AXI_DATA_WIDTH-1:0]     axi_rdata;
    reg  [1:0]                       axi_rresp;
    reg                              axi_rvalid;
    reg [C_S_AXI_ADDR_WIDTH-1:0]     axi_araddr;

    wire                        load_seed;
    wire [C_S_AXI_DATA_WIDTH-1:0] seed_in;
    wire [C_S_AXI_DATA_WIDTH-1:0] prng_out;

    localparam ADDR_LSB       = (C_S_AXI_DATA_WIDTH/32) + 1;
    localparam OPT_MEM_ADDR_BITS = 1; // 2 registers => 2 addressable words

    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BRESP   = axi_bresp;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RRESP   = axi_rresp;
    assign S_AXI_RVALID  = axi_rvalid;

    // ---- Write address channel ----
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_awready <= 1'b0;
            axi_awaddr  <= 0;
        end else if (!axi_awready && S_AXI_AWVALID && S_AXI_WVALID) begin
            axi_awready <= 1'b1;
            axi_awaddr  <= S_AXI_AWADDR;
        end else begin
            axi_awready <= 1'b0;
        end
    end

    // ---- Write data channel ----
    wire slv_reg_wren = axi_wready && S_AXI_WVALID && axi_awready && S_AXI_AWVALID;

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN)
            axi_wready <= 1'b0;
        else if (!axi_wready && S_AXI_WVALID && S_AXI_AWVALID)
            axi_wready <= 1'b1;
        else
            axi_wready <= 1'b0;
    end

    // ---- Write response channel ----
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_bvalid <= 1'b0;
            axi_bresp  <= 2'b0;
        end else if (axi_awready && S_AXI_AWVALID && !axi_bvalid && axi_wready && S_AXI_WVALID) begin
            axi_bvalid <= 1'b1;
            axi_bresp  <= 2'b0; // OKAY
        end else if (S_AXI_BREADY && axi_bvalid) begin
            axi_bvalid <= 1'b0;
        end
    end

    // ---- Read address channel ----
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_arready <= 1'b0;
            axi_araddr  <= 0;
        end else if (!axi_arready && S_AXI_ARVALID) begin
            axi_arready <= 1'b1;
            axi_araddr  <= S_AXI_ARADDR;
        end else begin
            axi_arready <= 1'b0;
        end
    end

    // ---- Read data channel ----
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_rvalid <= 1'b0;
            axi_rresp  <= 2'b0;
            axi_rdata  <= 0;
        end else if (axi_arready && S_AXI_ARVALID && !axi_rvalid) begin
            axi_rvalid <= 1'b1;
            axi_rresp  <= 2'b0; // OKAY
            case (S_AXI_ARADDR[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])
                2'b01:   axi_rdata <= prng_out; // 0x04 DATA_REG
                default: axi_rdata <= {C_S_AXI_DATA_WIDTH{1'b0}};
            endcase
        end else if (S_AXI_RVALID && S_AXI_RREADY) begin
            axi_rvalid <= 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // USER LOGIC #1: instantiate the verified Rule 30 core
    // ------------------------------------------------------------------
    // SEED_REG is offset 0x00 -> word address 0
    assign load_seed = slv_reg_wren && (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB] == 2'b00);
    assign seed_in    = S_AXI_WDATA;

    rule30_core #(.WIDTH(C_S_AXI_DATA_WIDTH)) u_rule30_core (
        .clk       (S_AXI_ACLK),
        .rst_n     (S_AXI_ARESETN),
        .load_seed (load_seed),
        .seed_in   (seed_in),
        .prng_out  (prng_out)
    );
    
    // Output lower 4 bits of the PRNG to the LEDs
    assign leds = prng_out[3:0];

endmodule
