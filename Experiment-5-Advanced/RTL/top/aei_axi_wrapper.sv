// ============================================================================
// AEI Engine AXI4-Lite wrapper
// ============================================================================

module aei_axi_wrapper #(
    parameter integer W = 20,
    parameter integer F = 12,
    parameter integer ACC_W = 44,
    parameter integer INV_W = 16,
    parameter integer NUM_TARGETS = 8
)(
    input  logic        s_axi_aclk,
    input  logic        s_axi_aresetn,
    input  logic [11:0] s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,
    input  logic [11:0] s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready
);

    localparam integer ID_W = $clog2(NUM_TARGETS);

    logic        aw_hold;
    logic [11:0] awaddr_reg;
    logic        w_hold;
    logic [31:0] wdata_reg;
    logic [3:0]  wstrb_reg;

    logic frame_start_reg;
    logic measurement_valid_reg;
    logic load_enable_reg;
    logic write_enable_reg;
    logic [ID_W-1:0] load_target_reg;
    logic signed [W-1:0] z_in_reg [0:1];
    logic signed [W-1:0] x_load_reg [0:3];
    logic signed [W-1:0] P_load_reg [0:3][0:3];
    logic signed [W-1:0] x_write_reg [0:3];
    logic signed [W-1:0] P_write_reg [0:3][0:3];
    logic signed [W-1:0] F_mat_reg [0:3][0:3];
    logic signed [W-1:0] Q_reg     [0:3][0:3];
    logic signed [W-1:0] R_diag_reg[0:1];
    logic signed [W-1:0] prediction_horizon_reg;
    logic signed [W-1:0] collision_threshold_reg;

    logic aei_busy;
    logic aei_frame_done;
    logic [ID_W-1:0] aei_target_id;
    logic signed [W-1:0] aei_x_out [0:3];
    logic signed [W-1:0] aei_P_out [0:3][0:3];
    logic aei_load_done;
    logic aei_collision_busy;
    logic aei_collision_done;
    logic aei_collision_detected;
    logic aei_collision_matrix [0:NUM_TARGETS-1][0:NUM_TARGETS-1];

    logic sticky_frame_done;
    logic sticky_collision_done;
    logic sticky_collision_detected;
    logic sticky_load_done;

    always_comb begin
        s_axi_awready = !aw_hold && !s_axi_bvalid;
        s_axi_wready  = !w_hold  && !s_axi_bvalid;
        s_axi_arready = !s_axi_rvalid;
    end

    integer i;
    integer j;

    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            aw_hold <= 1'b0;
            awaddr_reg <= '0;
            w_hold <= 1'b0;
            wdata_reg <= '0;
            wstrb_reg <= '0;
            s_axi_bvalid <= 1'b0;
            s_axi_bresp <= 2'b00;
            frame_start_reg <= 1'b0;
            measurement_valid_reg <= 1'b0;
            load_enable_reg <= 1'b0;
            write_enable_reg <= 1'b0;
            load_target_reg <= '0;
            prediction_horizon_reg <= '0;
            collision_threshold_reg <= '0;
            z_in_reg[0] <= '0;
            z_in_reg[1] <= '0;
            R_diag_reg[0] <= '0;
            R_diag_reg[1] <= '0;

            for (i = 0; i < 4; i = i + 1) begin
                x_load_reg[i] <= '0;
                x_write_reg[i] <= '0;
                for (j = 0; j < 4; j = j + 1) begin
                    P_load_reg[i][j] <= '0;
                    P_write_reg[i][j] <= '0;
                    F_mat_reg[i][j] <= '0;
                    Q_reg[i][j] <= '0;
                end
            end
        end
        else begin
            frame_start_reg      <= 1'b0;
            measurement_valid_reg <= 1'b0;
            load_enable_reg      <= 1'b0;
            write_enable_reg     <= 1'b0;

            if (s_axi_awvalid && s_axi_awready) begin
                aw_hold <= 1'b1;
                awaddr_reg <= s_axi_awaddr;
            end

            if (s_axi_wvalid && s_axi_wready) begin
                w_hold <= 1'b1;
                wdata_reg <= s_axi_wdata;
                wstrb_reg <= s_axi_wstrb;
            end

            if (aw_hold && w_hold && !s_axi_bvalid) begin
                case (awaddr_reg)
                    12'h000: begin
                        if (wstrb_reg[0]) begin
                            if (wdata_reg[0]) frame_start_reg <= 1'b1;
                            if (wdata_reg[1]) measurement_valid_reg <= 1'b1;
                            if (wdata_reg[2]) load_enable_reg <= 1'b1;
                            if (wdata_reg[3]) write_enable_reg <= 1'b1;
                        end
                    end
                    12'h008: load_target_reg <= wdata_reg[ID_W-1:0];
                    12'h00C: prediction_horizon_reg <= wdata_reg[W-1:0];
                    12'h010: collision_threshold_reg <= wdata_reg[W-1:0];
                    12'h014: z_in_reg[0] <= wdata_reg[W-1:0];
                    12'h018: z_in_reg[1] <= wdata_reg[W-1:0];
                    12'h01C: R_diag_reg[0] <= wdata_reg[W-1:0];
                    12'h020: R_diag_reg[1] <= wdata_reg[W-1:0];

                    12'h100, 12'h104, 12'h108, 12'h10C, 12'h110, 12'h114, 12'h118, 12'h11C, 
                    12'h120, 12'h124, 12'h128, 12'h12C, 12'h130, 12'h134, 12'h138, 12'h13C: begin
                        F_mat_reg[(awaddr_reg - 12'h100) >> 4][(awaddr_reg - 12'h100) >> 2 & 2'h3] <= wdata_reg[W-1:0];
                    end

                    12'h140, 12'h144, 12'h148, 12'h14C, 12'h150, 12'h154, 12'h158, 12'h15C,
                    12'h160, 12'h164, 12'h168, 12'h16C, 12'h170, 12'h174, 12'h178, 12'h17C: begin
                        Q_reg[(awaddr_reg - 12'h140) >> 4][(awaddr_reg - 12'h140) >> 2 & 2'h3] <= wdata_reg[W-1:0];
                    end

                    12'h200, 12'h204, 12'h208, 12'h20C: begin
                        x_load_reg[(awaddr_reg - 12'h200) >> 2] <= wdata_reg[W-1:0];
                    end

                    12'h210, 12'h214, 12'h218, 12'h21C, 12'h220, 12'h224, 12'h228, 12'h22C,
                    12'h230, 12'h234, 12'h238, 12'h23C, 12'h240, 12'h244, 12'h248, 12'h24C: begin
                        P_load_reg[(awaddr_reg - 12'h210) >> 4][(awaddr_reg - 12'h210) >> 2 & 2'h3] <= wdata_reg[W-1:0];
                    end

                    12'h250, 12'h254, 12'h258, 12'h25C: begin
                        x_write_reg[(awaddr_reg - 12'h250) >> 2] <= wdata_reg[W-1:0];
                    end

                    12'h260, 12'h264, 12'h268, 12'h26C, 12'h270, 12'h274, 12'h278, 12'h27C,
                    12'h280, 12'h284, 12'h288, 12'h28C, 12'h290, 12'h294, 12'h298, 12'h29C: begin
                        P_write_reg[(awaddr_reg - 12'h260) >> 4][(awaddr_reg - 12'h260) >> 2 & 2'h3] <= wdata_reg[W-1:0];
                    end

                    default: begin end
                endcase
                aw_hold <= 1'b0;
                w_hold <= 1'b0;
                s_axi_bvalid <= 1'b1;
                s_axi_bresp <= 2'b00;
            end
            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
        end
    end

    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            sticky_frame_done         <= 1'b0;
            sticky_collision_done     <= 1'b0;
            sticky_collision_detected <= 1'b0;
            sticky_load_done          <= 1'b0;
        end
        else begin
            if (aei_frame_done) sticky_frame_done <= 1'b1;
            if (aei_collision_done) begin
                sticky_collision_done <= 1'b1;
                if (aei_collision_detected) sticky_collision_detected <= 1'b1;
            end
            if (aei_load_done) sticky_load_done <= 1'b1;

            if (aw_hold && w_hold && !s_axi_bvalid &&
                (awaddr_reg == 12'h000) && wstrb_reg[0] && wdata_reg[4]) begin
                sticky_frame_done <= 1'b0;
                sticky_collision_done <= 1'b0;
                sticky_collision_detected <= 1'b0;
                sticky_load_done <= 1'b0;
            end
        end
    end

    logic [11:0] araddr_reg;

    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_rvalid <= 1'b0;
            s_axi_rresp <= 2'b00;
            s_axi_rdata <= 32'h00000000;
            araddr_reg <= '0;
        end
        else begin
            if (s_axi_arvalid && s_axi_arready) begin
                araddr_reg <= s_axi_araddr;
                s_axi_rvalid <= 1'b1;
                s_axi_rresp <= 2'b00;
                s_axi_rdata <= 32'h00000000;

                case (s_axi_araddr)
                    12'h004: begin
                        s_axi_rdata[0] <= aei_busy;
                        s_axi_rdata[1] <= sticky_frame_done;
                        s_axi_rdata[2] <= aei_collision_busy;
                        s_axi_rdata[3] <= sticky_collision_done;
                        s_axi_rdata[4] <= sticky_collision_detected;
                        s_axi_rdata[5] <= sticky_load_done;
                    end
                    12'h008: s_axi_rdata[ID_W-1:0] <= load_target_reg;
                    12'h00C: s_axi_rdata[W-1:0] <= prediction_horizon_reg;
                    12'h010: s_axi_rdata[W-1:0] <= collision_threshold_reg;
                    12'h014: s_axi_rdata[W-1:0] <= z_in_reg[0];
                    12'h018: s_axi_rdata[W-1:0] <= z_in_reg[1];
                    12'h01C: s_axi_rdata[W-1:0] <= R_diag_reg[0];
                    12'h020: s_axi_rdata[W-1:0] <= R_diag_reg[1];

                    12'h100, 12'h104, 12'h108, 12'h10C, 12'h110, 12'h114, 12'h118, 12'h11C,
                    12'h120, 12'h124, 12'h128, 12'h12C, 12'h130, 12'h134, 12'h138, 12'h13C: begin
                        s_axi_rdata[W-1:0] <= F_mat_reg[(s_axi_araddr - 12'h100) >> 4][((s_axi_araddr - 12'h100) >> 2) & 2'h3];
                    end

                    12'h140, 12'h144, 12'h148, 12'h14C, 12'h150, 12'h154, 12'h158, 12'h15C,
                    12'h160, 12'h164, 12'h168, 12'h16C, 12'h170, 12'h174, 12'h178, 12'h17C: begin
                        s_axi_rdata[W-1:0] <= Q_reg[(s_axi_araddr - 12'h140) >> 4][((s_axi_araddr - 12'h140) >> 2) & 2'h3];
                    end

                    12'h200, 12'h204, 12'h208, 12'h20C: begin
                        s_axi_rdata[W-1:0] <= x_load_reg[(s_axi_araddr - 12'h200) >> 2];
                    end

                    12'h210, 12'h214, 12'h218, 12'h21C, 12'h220, 12'h224, 12'h228, 12'h22C,
                    12'h230, 12'h234, 12'h238, 12'h23C, 12'h240, 12'h244, 12'h248, 12'h24C: begin
                        s_axi_rdata[W-1:0] <= P_load_reg[(s_axi_araddr - 12'h210) >> 4][((s_axi_araddr - 12'h210) >> 2) & 2'h3];
                    end

                    12'h250, 12'h254, 12'h258, 12'h25C: begin
                        s_axi_rdata[W-1:0] <= x_write_reg[(s_axi_araddr - 12'h250) >> 2];
                    end

                    12'h260, 12'h264, 12'h268, 12'h26C, 12'h270, 12'h274, 12'h278, 12'h27C,
                    12'h280, 12'h284, 12'h288, 12'h28C, 12'h290, 12'h294, 12'h298, 12'h29C: begin
                        s_axi_rdata[W-1:0] <= P_write_reg[(s_axi_araddr - 12'h260) >> 4][((s_axi_araddr - 12'h260) >> 2) & 2'h3];
                    end

                    12'h300, 12'h304, 12'h308, 12'h30C: begin
                        s_axi_rdata[W-1:0] <= aei_x_out[(s_axi_araddr - 12'h300) >> 2];
                    end

                    12'h310, 12'h314, 12'h318, 12'h31C, 12'h320, 12'h324, 12'h328, 12'h32C,
                    12'h330, 12'h334, 12'h338, 12'h33C, 12'h340, 12'h344, 12'h348, 12'h34C: begin
                        s_axi_rdata[W-1:0] <= aei_P_out[(s_axi_araddr - 12'h310) >> 4][((s_axi_araddr - 12'h310) >> 2) & 2'h3];
                    end

                    12'h350: s_axi_rdata[ID_W-1:0] <= aei_target_id;

                    12'h400, 12'h404, 12'h408, 12'h40C, 12'h410, 12'h414, 12'h418, 12'h41C,
                    12'h420, 12'h424, 12'h428, 12'h42C, 12'h430, 12'h434, 12'h438, 12'h43C,
                    12'h440, 12'h444, 12'h448, 12'h44C, 12'h450, 12'h454, 12'h458, 12'h45C,
                    12'h460, 12'h464, 12'h468, 12'h46C, 12'h470, 12'h474, 12'h478, 12'h47C,
                    12'h480, 12'h484, 12'h488, 12'h48C, 12'h490, 12'h494, 12'h498, 12'h49C,
                    12'h4A0, 12'h4A4, 12'h4A8, 12'h4AC, 12'h4B0, 12'h4B4, 12'h4B8, 12'h4BC,
                    12'h4C0, 12'h4C4, 12'h4C8, 12'h4CC, 12'h4D0, 12'h4D4, 12'h4D8, 12'h4DC,
                    12'h4E0, 12'h4E4, 12'h4E8, 12'h4EC, 12'h4F0, 12'h4F4, 12'h4F8, 12'h4FC: begin
                        s_axi_rdata[0] <= aei_collision_matrix[(s_axi_araddr - 12'h400) >> 5][((s_axi_araddr - 12'h400) >> 2) & 3'h7];
                    end

                    default: s_axi_rdata <= 32'h00000000;
                endcase
            end
            if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
        end
    end

    aei_system_top #(
        .W(W),
        .F(F),
        .ACC_W(ACC_W),
        .INV_W(INV_W),
        .NUM_TARGETS(NUM_TARGETS)
    ) u_aei_system_top (
        .clk(s_axi_aclk),
        .rst(!s_axi_aresetn),
        .frame_start(frame_start_reg),
        .measurement_valid(measurement_valid_reg),
        .z_in(z_in_reg),
        .load_enable(load_enable_reg),
        .load_target(load_target_reg),
        .x_load(x_load_reg),
        .P_load(P_load_reg),
        .load_done(aei_load_done),
        .write_enable(write_enable_reg),
        .x_write(x_write_reg),
        .P_write(P_write_reg),
        .F_mat(F_mat_reg),
        .Q(Q_reg),
        .R_diag(R_diag_reg),
        .prediction_horizon(prediction_horizon_reg),
        .collision_threshold(collision_threshold_reg),
        .busy(aei_busy),
        .frame_done(aei_frame_done),
        .target_id(aei_target_id),
        .x_out(aei_x_out),
        .P_out(aei_P_out),
        .collision_busy(aei_collision_busy),
        .collision_done(aei_collision_done),
        .collision_detected(aei_collision_detected),
        .collision_matrix(aei_collision_matrix)
    );

endmodule