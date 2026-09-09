`timescale 1ns / 1ps

module kalman_engine #(
    parameter integer W     = 20,
    parameter integer F     = 12,
    parameter integer ACC_W = 44,
    parameter integer INV_W = 16
)(
    input  logic clk,
    input  logic rst,
    input  logic start,

    input  logic maneuver_in, 

    input  logic signed [W-1:0] x_in [0:3],
    input  logic signed [W-1:0] P_in [0:3][0:3],
    input  logic signed [W-1:0] z [0:1],

    input  logic signed [W-1:0] F_mat [0:3][0:3],
    input  logic signed [W-1:0] Q [0:3][0:3],
    input  logic signed [W-1:0] R_diag [0:1],

    output logic signed [W-1:0] x_out [0:3],
    output logic signed [W-1:0] P_out [0:3][0:3],

    output logic maneuver_out, 
    output logic busy,
    output logic done
);

    // ================================================================
    // Internal datapath signals
    // ================================================================

    logic signed [W-1:0] x_pred [0:3];
    logic signed [W-1:0] P_pred [0:3][0:3];

    logic signed [W-1:0] innovation [0:1];
    logic signed [W-1:0] S [0:1];

    logic signed [INV_W-1:0] invS;

    logic signed [W-1:0] PHt [0:3][0:1];
    logic signed [W-1:0] K [0:3][0:1];

    logic signed [W-1:0] x_updated [0:3];

    logic signed [W-1:0] KH [0:3][0:3];
    logic signed [W-1:0] I_KH [0:3][0:3];

    logic signed [W-1:0] P_updated [0:3][0:3];

    // ================================================================
    // ISOLATION REGISTER (Fixes Critical Path)
    // ================================================================
    (* keep = "true", max_fanout = "16" *) logic signed [INV_W-1:0] invS_isolated;

    always_ff @(posedge clk) begin
        if (rst) begin
            invS_isolated <= '0;
        end else begin
            invS_isolated <= invS;
        end
    end

    // ================================================================
    // NIS Combinational Wires & Registers
    // ================================================================
    logic signed [39:0] inn_x_sq_wire;
    logic signed [39:0] inn_y_sq_wire;
    logic signed [W-1:0] nis_val_wire;

    assign inn_x_sq_wire = innovation[0] * innovation[0];
    assign inn_y_sq_wire = innovation[1] * innovation[1];
    assign nis_val_wire  = nis_raw_reg[W+12-1 : 12];

    logic signed [W-1:0] inn_x_sq;
    logic signed [W-1:0] inn_y_sq;
    logic signed [W-1:0] inn_sum;
    logic signed [35:0] nis_raw_reg;   

    localparam signed [W-1:0] GAMMA_THRESH = 20'sd37724; 

    // ================================================================
    // Latched inputs
    // ================================================================
    logic maneuver_reg;
    logic signed [W-1:0] x_reg [0:3];
    logic signed [W-1:0] P_reg [0:3][0:3];
    logic signed [W-1:0] z_reg [0:1];

    logic signed [W-1:0] F_reg [0:3][0:3];
    logic signed [W-1:0] Q_reg [0:3][0:3];
    logic signed [W-1:0] R_reg [0:1];

    // ================================================================
    // Start signals & Submodule status
    // ================================================================
    logic start_state_predict;
    logic start_covariance_predict;
    logic start_measurement_update;
    logic start_innovation_covariance;
    logic start_reciprocal;
    logic start_ph_transpose;
    logic start_kalman_gain;
    logic start_state_correction;
    logic start_kh_matrix;
    logic start_I_minus_KH;
    logic start_covariance_update;

    logic state_predict_done;
    logic covariance_predict_done;
    logic measurement_update_done;
    logic innovation_covariance_done;
    logic reciprocal_done;
    logic ph_transpose_done;
    logic kalman_gain_done;
    logic state_correction_done;
    logic kh_matrix_done;
    logic I_minus_KH_done;
    logic covariance_update_done;

    logic state_predict_seen;
    logic covariance_predict_seen;
    logic measurement_update_seen;
    logic innovation_covariance_seen;
    logic ph_transpose_seen;
    logic state_correction_seen;
    logic kh_matrix_seen;

    // ================================================================
    // FSM
    // ================================================================
    typedef enum logic [3:0] {
        IDLE,
        PREDICT,
        WAIT_PREDICT,
        WAIT_MEASUREMENT,
        WAIT_RECIPROCAL,
        CALC_NIS_1, 
        CALC_NIS_2, 
        CALC_NIS_3, 
        CALC_NIS_4, 
        WAIT_GAIN,
        WAIT_STATE_AND_KH,
        WAIT_I_MINUS_KH,
        WAIT_COVARIANCE,
        FINISH
    } state_t;

    state_t state;
    integer i, j;

    // ================================================================
    // SUBMODULE INSTANTIATIONS
    // ================================================================
    state_predict #(.W(W), .F(F), .ACC_W(ACC_W)) u_state_predict (
        .clk(clk), .rst(rst), .start(start_state_predict),
        .x_in(x_reg), .F_mat(F_reg), .x_out(x_pred),
        .busy(), .done(state_predict_done)
    );

    covariance_predict #(.W(W), .F(F), .ACC_W(ACC_W)) u_covariance_predict (
        .clk(clk), .rst(rst), .start(start_covariance_predict),
        .maneuver_flag(maneuver_reg), 
        .P_in(P_reg), .F_mat(F_reg), .Q_in(Q_reg), .P_pred(P_pred),
        .busy(), .done(covariance_predict_done)
    );

    measurement_update #(.W(W)) u_measurement_update (
        .clk(clk), .rst(rst), .start(start_measurement_update),
        .x_pred(x_pred), .z(z_reg), .innovation(innovation),
        .busy(), .done(measurement_update_done)
    );

    innovation_covariance #(.W(W)) u_innovation_covariance (
        .clk(clk), .rst(rst), .start(start_innovation_covariance),
        .P_pred(P_pred), .R_diag(R_reg), .S(S),
        .busy(), .done(innovation_covariance_done)
    );

    reciprocal #(.IN_W(W), .OUT_W(INV_W), .INT_W(29), .INT_F(27), .LUT_ADDR_W(6)) u_reciprocal (
        .clk(clk), .rst(rst), .start(start_reciprocal),
        .x_in(S[0]), .reciprocal_out(invS),
        .busy(), .done(reciprocal_done)
    );

    ph_transpose #(.W(W)) u_ph_transpose (
        .clk(clk), .rst(rst), .start(start_ph_transpose),
        .P_pred(P_pred), .PHt(PHt),
        .busy(), .done(ph_transpose_done)
    );

    kalman_gain #(.W(W), .INV_W(INV_W), .F(F)) u_kalman_gain (
        .clk(clk), .rst(rst), .start(start_kalman_gain),
        .PHt(PHt), .invS(invS_isolated), 
        .K(K),
        .busy(), .done(kalman_gain_done)
    );

    state_correction #(.W(W), .F(F), .ACC_W(ACC_W)) u_state_correction (
        .clk(clk), .rst(rst), .start(start_state_correction),
        .x_pred(x_pred), .K(K), .innovation(innovation), .x_updated(x_updated),
        .busy(), .done(state_correction_done)
    );

    kh_matrix #(.W(W)) u_kh_matrix (
        .clk(clk), .rst(rst), .start(start_kh_matrix),
        .K(K), .KH(KH),
        .busy(), .done(kh_matrix_done)
    );

    I_minus_KH #(.W(W), .IDENTITY(4096)) u_I_minus_KH (
        .clk(clk), .rst(rst), .start(start_I_minus_KH),
        .KH(KH), .I_KH(I_KH),
        .busy(), .done(I_minus_KH_done)
    );

    covariance_update #(.W(W), .F(F), .ACC_W(ACC_W)) u_covariance_update (
        .clk(clk), .rst(rst), .start(start_covariance_update),
        .I_KH(I_KH), .P_pred(P_pred), .P_updated(P_updated),
        .busy(), .done(covariance_update_done)
    );

    // ================================================================
    // CONTROLLER
    // ================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            maneuver_out <= 1'b0;

            start_state_predict <= 1'b0;
            start_covariance_predict <= 1'b0;
            start_measurement_update <= 1'b0;
            start_innovation_covariance <= 1'b0;
            start_reciprocal <= 1'b0;
            start_ph_transpose <= 1'b0;
            start_kalman_gain <= 1'b0;
            start_state_correction <= 1'b0;
            start_kh_matrix <= 1'b0;
            start_I_minus_KH <= 1'b0;
            start_covariance_update <= 1'b0;

            state_predict_seen <= 1'b0;
            covariance_predict_seen <= 1'b0;
            measurement_update_seen <= 1'b0;
            innovation_covariance_seen <= 1'b0;
            ph_transpose_seen <= 1'b0;
            state_correction_seen <= 1'b0;
            kh_matrix_seen <= 1'b0;
            
            maneuver_reg <= 1'b0;
            inn_x_sq <= '0;
            inn_y_sq <= '0;
            inn_sum <= '0;
            nis_raw_reg <= '0; 

            for (i = 0; i < 4; i = i + 1) begin
                x_reg[i] <= '0;
                x_out[i] <= '0;
                for (j = 0; j < 4; j = j + 1) begin
                    P_reg[i][j] <= '0;
                    F_reg[i][j] <= '0;
                    Q_reg[i][j] <= '0;
                    P_out[i][j] <= '0;
                end
            end
            z_reg[0] <= '0;
            z_reg[1] <= '0;
            R_reg[0] <= '0;
            R_reg[1] <= '0;
        end else begin
            done <= 1'b0;
            start_state_predict <= 1'b0;
            start_covariance_predict <= 1'b0;
            start_measurement_update <= 1'b0;
            start_innovation_covariance <= 1'b0;
            start_reciprocal <= 1'b0;
            start_ph_transpose <= 1'b0;
            start_kalman_gain <= 1'b0;
            start_state_correction <= 1'b0;
            start_kh_matrix <= 1'b0;
            start_I_minus_KH <= 1'b0;
            start_covariance_update <= 1'b0;

            case (state)
                IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        maneuver_reg <= maneuver_in; 
                        x_reg[0] <= x_in[0]; x_reg[1] <= x_in[1]; x_reg[2] <= x_in[2]; x_reg[3] <= x_in[3];
                        z_reg[0] <= z[0]; z_reg[1] <= z[1];
                        R_reg[0] <= R_diag[0]; R_reg[1] <= R_diag[1];

                        for (i = 0; i < 4; i = i + 1) begin
                            for (j = 0; j < 4; j = j + 1) begin
                                P_reg[i][j] <= P_in[i][j];
                                F_reg[i][j] <= F_mat[i][j];
                                Q_reg[i][j] <= Q[i][j];
                            end
                        end
                        state <= PREDICT;
                    end
                end

                PREDICT: begin
                    state_predict_seen <= 1'b0;
                    covariance_predict_seen <= 1'b0;
                    start_state_predict <= 1'b1;
                    start_covariance_predict <= 1'b1;
                    state <= WAIT_PREDICT;
                end

                WAIT_PREDICT: begin
                    if (state_predict_done)      state_predict_seen <= 1'b1;
                    if (covariance_predict_done) covariance_predict_seen <= 1'b1;

                    if ((state_predict_seen || state_predict_done) && 
                        (covariance_predict_seen || covariance_predict_done)) begin
                        measurement_update_seen    <= 1'b0;
                        innovation_covariance_seen <= 1'b0;
                        ph_transpose_seen          <= 1'b0;
                        start_measurement_update   <= 1'b1;
                        start_innovation_covariance<= 1'b1;
                        start_ph_transpose         <= 1'b1;
                        state <= WAIT_MEASUREMENT;
                    end
                end

                WAIT_MEASUREMENT: begin
                    if (measurement_update_done)    measurement_update_seen <= 1'b1;
                    if (innovation_covariance_done) innovation_covariance_seen <= 1'b1;
                    if (ph_transpose_done)          ph_transpose_seen <= 1'b1;

                    if ((measurement_update_seen || measurement_update_done) &&
                        (innovation_covariance_seen || innovation_covariance_done) &&
                        (ph_transpose_seen || ph_transpose_done)) begin
                        start_reciprocal <= 1'b1;
                        state <= WAIT_RECIPROCAL;
                    end
                end

                WAIT_RECIPROCAL: begin
                    if (reciprocal_done) begin
                        state <= CALC_NIS_1; 
                    end
                end
                
                CALC_NIS_1: begin
                    inn_x_sq <= inn_x_sq_wire[W+F-1 : F];
                    inn_y_sq <= inn_y_sq_wire[W+F-1 : F];
                    state <= CALC_NIS_2;
                end

                CALC_NIS_2: begin
                    inn_sum <= inn_x_sq + inn_y_sq;
                    state <= CALC_NIS_3;
                end

                CALC_NIS_3: begin
                    nis_raw_reg <= inn_sum * invS_isolated; 
                    state <= CALC_NIS_4;
                end

                CALC_NIS_4: begin
                    if (nis_val_wire > GAMMA_THRESH) maneuver_out <= 1'b1;
                    else maneuver_out <= 1'b0;
                    
                    start_kalman_gain <= 1'b1;
                    state <= WAIT_GAIN;
                end

                WAIT_GAIN: begin
                    if (kalman_gain_done) begin
                        state_correction_seen <= 1'b0;
                        kh_matrix_seen <= 1'b0;
                        start_state_correction <= 1'b1;
                        start_kh_matrix <= 1'b1;
                        state <= WAIT_STATE_AND_KH;
                    end
                end

                WAIT_STATE_AND_KH: begin
                    if (state_correction_done) state_correction_seen <= 1'b1;
                    if (kh_matrix_done)        kh_matrix_seen <= 1'b1;

                    if ((state_correction_seen || state_correction_done) && 
                        (kh_matrix_seen || kh_matrix_done)) begin
                        start_I_minus_KH <= 1'b1;
                        state <= WAIT_I_MINUS_KH;
                    end
                end

                WAIT_I_MINUS_KH: begin
                    if (I_minus_KH_done) begin
                        start_covariance_update <= 1'b1;
                        state <= WAIT_COVARIANCE;
                    end
                end

                WAIT_COVARIANCE: begin
                    if (covariance_update_done) begin
                        x_out[0] <= x_updated[0];
                        x_out[1] <= x_updated[1];
                        x_out[2] <= x_updated[2];
                        x_out[3] <= x_updated[3];
                        for (i = 0; i < 4; i = i + 1) begin
                            for (j = 0; j < 4; j = j + 1) begin
                                P_out[i][j] <= P_updated[i][j];
                            end
                        end
                        state <= FINISH;
                    end
                end

                FINISH: begin
                    busy <= 1'b0;
                    done <= 1'b1;
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