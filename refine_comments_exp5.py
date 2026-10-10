import os
import glob
import re

file_details = {
    'aei_engine_top.sv': 'Top-level wrapper for the AEI (Algebraic Equivalent Indicator) engine. Manages AXI interfaces and orchestrates data flow into the core tracking engines.',
    'aei_engine.sv': 'Core computational logic for the AEI tracking algorithm. Processes radar/sensor measurements to compute object kinematics and trajectory estimates.',
    'aei_system_regression.sv': 'System-level regression testing harness for the AEI architecture. Provides self-checking stimulus generation for corner cases.',
    'aei_system_top.sv': 'Topmost system integration file wrapping the AEI engine with PS-PL boundary components (e.g. AXI interconnects, clock domain crossing).',
    'collision_engine.sv': 'Dedicated hardware accelerator for predicting physical intersections between tracked targets based on their estimated state vectors and covariance ellipsoids.',
    'frame_engine.sv': 'Manages coordinate transformations and frame-of-reference rotations (e.g. sensor frame to world frame) for incoming measurement vectors.',
    'measurement_scheduler.sv': 'Arbitrates and schedules asynchronous incoming sensor measurements, dispatching them to the Kalman filter engines to prevent pipeline stalls.',
    'system_corner_regression.sv': 'Specialized regression suite to test boundary conditions, pipeline backpressure, arithmetic overflows, and matrix singularity exceptions.',
    'hw_server.py': 'Python-based hardware server mimicking the FPGA memory map over Ethernet/UART. Used for hardware-in-the-loop (HIL) testing and data logging.',
    'live_tracking_pynq.py': 'Jupyter/Python integration script for the PYNQ framework. Allocates contiguous memory buffers, configures the AEI IP, and visualizes live target tracks.',
    'tb_aei_engine_reference.sv': 'Golden reference testbench for the AEI engine, comparing RTL outputs cycle-by-cycle against a bit-accurate behavioral model.',
    'tb_aei_engine_top.sv': 'Integration testbench for the aei_engine_top module, stimulating AXI transactions to mimic the Zynq ARM processor.',
    'tb_aei_engine.sv': 'Unit testbench for the core AEI algorithm block, injecting standard trajectories and verifying tracking convergence.',
    'tb_aei_system_regression.sv': 'Automated regression testbench wrapper executing a suite of stress-tests across the entire AEI system.',
    'tb_aei_system_top.sv': 'Full-system testbench verifying the interactions between the PS VIP (Verification IP) and the PL AEI wrappers.',
    'tb_collision_engine.sv': 'Unit testbench for the collision_engine. Injects intersecting and diverging trajectories to verify true/false positive rates.',
    'tb_collision_predictor.sv': 'Verifies the forward-time projection logic of the collision detection system, predicting TTI (Time to Intersection).',
    'tb_covariance_predict.sv': 'Unit testbench for the Kalman filter covariance prediction step (P = F*P*F^T + Q).',
    'tb_covariance_update.sv': 'Unit testbench for the Kalman filter covariance update step (P = (I - K*H)*P).',
    'tb_frame_engine.sv': 'Unit testbench for the frame transformation engine, injecting known polar/Cartesian coordinates and verifying rotation matrices.',
    'tb_I_minus_KH.sv': 'Unit testbench verifying the specific matrix subtraction and multiplication for the (I - K*H) Kalman term.',
    'tb_innovation_covariance.sv': 'Unit testbench for the Innovation Covariance matrix computation (S = H*P*H^T + R).',
    'tb_kalman_engine.sv': 'Testbench for the consolidated Kalman Filter engine, executing full Predict-Update cycles over noisy stimulus data.',
    'tb_kalman_gain.sv': 'Unit testbench for the Kalman Gain computation (K = P*H^T * S^-1), including matrix inversion checks.',
    'tb_kh_matrix.sv': 'Unit testbench for the intermediate K*H matrix multiplication block.',
    'tb_measurement_scheduler.sv': 'Unit testbench for the measurement scheduler, generating heavy sensor burst traffic to test FIFO and arbitration logic.',
    'tb_measurement_update.sv': 'Unit testbench for the Kalman state update step (x = x + K*y).',
    'tb_multi_target_collision_detector.sv': 'Testbench for N-body collision detection, scaling the collision logic to handle multiple simultaneous targets.',
    'tb_multi_target_engine.sv': 'Testbench for the multi-target tracking supervisor, ensuring target IDs and states are independently maintained.',
    'tb_ph_transpose.sv': 'Unit testbench for matrix transpose and multiplication operations (P * H^T) used heavily in the KF.',
    'tb_reciprocal.sv': 'Unit testbench for the hardware division/reciprocal accelerator (e.g. Newton-Raphson or LUT-based), essential for matrix inversion.',
    'tb_state_correction.sv': 'Unit testbench for applying the innovation vector (residual) to correct the a priori state estimate.',
    'tb_state_predict.sv': 'Unit testbench for the forward state projection block (x = F*x) based on the kinematic model.',
    'tb_system_corner_regression.sv': 'Regression testbench specifically focused on corner-case math errors (e.g. divide by zero, saturation).',
    'tb_target_memory_load.sv': 'Unit testbench verifying DMA/AXI bursts for loading external target configurations into BRAM/URAM.',
    'tb_target_memory.sv': 'Unit testbench for the multi-banked memory subsystem holding target state vectors and covariance matrices.',
    'tb_target_scheduler.sv': 'Unit testbench for the scheduling logic that decides which target to update next based on observation priority.'
}

def get_comment(filename):
    desc = file_details.get(filename, "Advanced component of the Experiment 5 multi-target tracking and collision detection architecture.")
    
    if filename.endswith('.py'):
        return f'''# ============================================================================
# Module:        {filename}
# Project:       FPGA Build Challenge - Experiment 5 (Advanced)
# Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
#
# System Context & Top-Level Integration:
#   {desc}
#
# Architectural Hierarchy:
#   - Part of the Software/Simulation stack for the Advanced Kalman Filter project.
#   - Interfaces with the Zynq Processing System (PS) via AXI memory-mapped I/O.
# ============================================================================
'''
    else:
        return f'''// ============================================================================
// Module:        {filename}
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   {desc}
//
// Architectural Hierarchy:
//   - Instantiated within the broader Experiment 5 RTL/Verification ecosystem.
//   - Synthesizable for PL (Programmable Logic) deployment unless marked as TB.
// ============================================================================
'''

files = glob.glob('Experiment-5-Advanced/RTL/*.*') + \
        glob.glob('Experiment-5-Advanced/Simulation/*.*') + \
        glob.glob('Experiment-5-Advanced/Testbench/*.*')

for filepath in files:
    filename = os.path.basename(filepath)
    if not (filename.endswith('.sv') or filename.endswith('.v') or filename.endswith('.py')):
        continue

    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
    
    content = re.sub(r'(?s)// ============================================================================.*?// ============================================================================\n', '', content)
    content = re.sub(r'(?s)# ============================================================================.*?# ============================================================================\n', '', content)
    
    if filename.endswith('.sv') or filename.endswith('.v'):
        content = re.sub(r'^`timescale.*?\n', '', content).lstrip()
        new_content = '`timescale 1ns / 1ps\n\n' + get_comment(filename) + content
    else:
        new_content = get_comment(filename) + content.lstrip()
        
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(new_content)
