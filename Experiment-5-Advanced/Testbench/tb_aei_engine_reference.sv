`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_aei_engine_reference.sv
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Golden reference testbench for the AEI engine, comparing RTL outputs cycle-by-cycle against a bit-accurate behavioral model.
//
// Architectural Hierarchy:
//   - Instantiated within the broader Experiment 5 RTL/Verification ecosystem.
//   - Synthesizable for PL (Programmable Logic) deployment unless marked as TB.
// ============================================================================
module tb_aei_engine_reference;

    localparam integer W = 20;
    localparam integer F = 12;
    localparam integer ACC_W = 44;
    localparam integer INV_W = 16;
    localparam integer NUM_TARGETS = 8;
    localparam integer Q_SCALE = 4096;
    localparam integer ID_W = $clog2(NUM_TARGETS);

    logic clk = 1'b0;
    logic rst = 1'b1;

    logic frame_start;
    logic measurement_valid;
    logic signed [W-1:0] z_in [0:1];

    logic load_enable;
    logic [ID_W-1:0] load_target;
    logic signed [W-1:0] x_load [0:3];
    logic signed [W-1:0] P_load [0:3][0:3];
    logic load_done;

    logic write_enable;
    logic signed [W-1:0] x_write [0:3];
    logic signed [W-1:0] P_write [0:3][0:3];

    logic signed [W-1:0] F_mat [0:3][0:3];
    logic signed [W-1:0] Q_mat [0:3][0:3];
    logic signed [W-1:0] R_diag [0:1];

    logic signed [W-1:0] prediction_horizon;
    logic signed [W-1:0] collision_threshold;

    logic busy;
    logic frame_done;
    logic [ID_W-1:0] target_id;

    logic signed [W-1:0] x_out [0:3];
    logic signed [W-1:0] P_out [0:3][0:3];

    logic collision_busy;
    logic collision_done;
    logic collision_detected;
    logic collision_matrix [0:NUM_TARGETS-1][0:NUM_TARGETS-1];

    aei_engine_top #(
        .W(W), .F(F), .ACC_W(ACC_W), .INV_W(INV_W),
        .NUM_TARGETS(NUM_TARGETS)
    ) dut (
        .clk(clk), .rst(rst),
        .frame_start(frame_start),
        .measurement_valid(measurement_valid),
        .z_in(z_in),
        .load_enable(load_enable),
        .load_target(load_target),
        .x_load(x_load),
        .P_load(P_load),
        .load_done(load_done),
        .write_enable(write_enable),
        .x_write(x_write),
        .P_write(P_write),
        .F_mat(F_mat),
        .Q(Q_mat),
        .R_diag(R_diag),
        .prediction_horizon(prediction_horizon),
        .collision_threshold(collision_threshold),
        .busy(busy),
        .frame_done(frame_done),
        .target_id(target_id),
        .x_out(x_out),
        .P_out(P_out),
        .collision_busy(collision_busy),
        .collision_done(collision_done),
        .collision_detected(collision_detected),
        .collision_matrix(collision_matrix)
    );

    always #5 clk = ~clk;

    task automatic clear_arrays;
        integer i, j;
        begin
            for (i = 0; i < 4; i = i + 1) begin
                x_load[i] = '0;
                x_write[i] = '0;
                for (j = 0; j < 4; j = j + 1) begin
                    P_load[i][j] = '0;
                    P_write[i][j] = '0;
                    F_mat[i][j] = '0;
                    Q_mat[i][j] = '0;
                end
            end
            z_in[0] = '0;
            z_in[1] = '0;
            R_diag[0] = '0;
            R_diag[1] = '0;
        end
    endtask

    task automatic configure;
        begin
            F_mat[0][0] = Q_SCALE;
            F_mat[1][1] = Q_SCALE;
            F_mat[2][2] = Q_SCALE;
            F_mat[3][3] = Q_SCALE;
            F_mat[0][2] = 41;
            F_mat[1][3] = 41;

            Q_mat[0][0] = 1;
            Q_mat[1][1] = 1;
            Q_mat[2][2] = 1;
            Q_mat[3][3] = 1;

            R_diag[0] = 1;
            R_diag[1] = 1;

            prediction_horizon = 8192;
            collision_threshold = 8192;
        end
    endtask

    task automatic load_one(
        input integer tid,
        input integer px,
        input integer py,
        input integer vx,
        input integer vy
    );
        integer i, j;
        begin
            for (i = 0; i < 4; i = i + 1) begin
                x_load[i] = '0;
                for (j = 0; j < 4; j = j + 1)
                    P_load[i][j] = '0;
            end

            x_load[0] = px * Q_SCALE;
            x_load[1] = py * Q_SCALE;
            x_load[2] = vx * Q_SCALE;
            x_load[3] = vy * Q_SCALE;

            P_load[0][0] = Q_SCALE;
            P_load[1][1] = Q_SCALE;
            P_load[2][2] = Q_SCALE;
            P_load[3][3] = Q_SCALE;

            @(negedge clk);
            load_target = tid;
            load_enable = 1'b1;
            @(negedge clk);
            load_enable = 1'b0;
            wait (load_done === 1'b1);
            @(negedge clk);
        end
    endtask

    task automatic send_measurement(
        input integer px,
        input integer py
    );
        begin
            z_in[0] = px * Q_SCALE;
            z_in[1] = py * Q_SCALE;
            @(negedge clk);
            measurement_valid = 1'b1;
            @(negedge clk);
            measurement_valid = 1'b0;
        end
    endtask

    task automatic run_frame;
        integer t;
        integer px, py;
        begin
            @(negedge clk);
            frame_start = 1'b1;
            @(negedge clk);
            frame_start = 1'b0;

            for (t = 0; t < NUM_TARGETS; t = t + 1) begin
                wait (busy === 1'b1);
                wait (target_id === t[ID_W-1:0]);

                // Deterministic measurements intentionally differ from
                // initial states so that the complete Kalman datapath is exercised.
                px = (t + 1) * 10 + ((t % 3) - 1);
                py = (t + 1) * 8  + ((t % 2) ? 1 : -1);
                send_measurement(px, py);
            end

            wait (frame_done === 1'b1);
            wait (collision_done === 1'b1);
            #1;
        end
    endtask

    integer fd;
    integer i, j;

    initial begin
        clear_arrays();
        configure();

        frame_start = 1'b0;
        measurement_valid = 1'b0;
        load_enable = 1'b0;
        load_target = '0;
        write_enable = 1'b0;

        repeat (3) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        // Eight deterministic, non-identical targets.
        load_one(0, 10,  8,  2, -1);
        load_one(1, 22, 17, -3,  2);
        load_one(2, 31, 24,  1,  1);
        load_one(3, 43, 31, -2, -2);
        load_one(4, 54, 42,  3,  0);
        load_one(5, 66, 49, -1,  2);
        load_one(6, 77, 58,  2, -2);
        load_one(7, 89, 66, -2,  1);

        run_frame();

        fd = $fopen("aei_rtl_reference_dump.txt", "w");
        if (fd == 0) begin
            $display("ERROR: unable to open aei_rtl_reference_dump.txt");
            $finish;
        end

        $fdisplay(fd, "AEI_RTL_REFERENCE_V1");
        $fdisplay(fd, "W %0d F %0d TARGETS %0d", W, F, NUM_TARGETS);
        $fdisplay(fd, "PREDICTION_HORIZON %0d", prediction_horizon);
        $fdisplay(fd, "COLLISION_THRESHOLD %0d", collision_threshold);

        for (i = 0; i < NUM_TARGETS; i = i + 1) begin
            $fdisplay(fd, "TARGET %0d", i);
            $fdisplay(fd, "X %0d %0d %0d %0d",
                $signed(dut.final_target_state[i][0]),
                $signed(dut.final_target_state[i][1]),
                $signed(dut.final_target_state[i][2]),
                $signed(dut.final_target_state[i][3]));
            for (j = 0; j < 4; j = j + 1)
                $fdisplay(fd, "P%0d %0d %0d %0d %0d",
                    j,
                    $signed(dut.frame_P_out[j][0]),
                    $signed(dut.frame_P_out[j][1]),
                    $signed(dut.frame_P_out[j][2]),
                    $signed(dut.frame_P_out[j][3]));
        end

        $fdisplay(fd, "COLLISION_DETECTED %0d", collision_detected);
        for (i = 0; i < NUM_TARGETS; i = i + 1) begin
            $fwrite(fd, "MATRIX %0d", i);
            for (j = 0; j < NUM_TARGETS; j = j + 1)
                $fwrite(fd, " %0d", collision_matrix[i][j]);
            $fwrite(fd, "\n");
        end
        $fclose(fd);

        $display("REFERENCE DUMP COMPLETE");
        $display("RTL results written to aei_rtl_reference_dump.txt");
        $finish;
    end

endmodule
