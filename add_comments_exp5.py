import os
import glob

v_comment = """`timescale 1ns / 1ps

// ============================================================================
// Module:        {filename}
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Advanced FPGA architecture implementation featuring Kalman filters,
//   collision detection, and multi-target tracking.
//
// Architectural Hierarchy:
//   Part of the Experiment 5 RTL / Simulation / Testbench ecosystem.
// ============================================================================

"""

py_comment = """# ============================================================================
# Module:        {filename}
# Project:       FPGA Build Challenge - Experiment 5 (Advanced)
# Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
#
# System Context & Top-Level Integration:
#   This Python script is part of the Advanced tracking ecosystem,
#   used for reference modeling, tracking, or generating test vectors.
# ============================================================================

"""

def process_file(filepath):
    filename = os.path.basename(filepath)
    ext = os.path.splitext(filename)[1]
    
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
        
    if ext in ['.v', '.sv']:
        lines = content.split('\n')
        new_lines = []
        for line in lines:
            if line.startswith('`timescale'):
                continue
            new_lines.append(line)
        content = '\n'.join(new_lines).strip()
        new_content = v_comment.format(filename=filename.replace(ext, '')) + content + '\n'
    elif ext == '.py':
        new_content = py_comment.format(filename=filename) + content
    else:
        return False
        
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(new_content)
        
    return True

files = glob.glob('Experiment-5-Advanced/RTL/*.*') + \
        glob.glob('Experiment-5-Advanced/Simulation/*.*') + \
        glob.glob('Experiment-5-Advanced/Testbench/*.*')

for filepath in files:
    if filepath.endswith('.sv') or filepath.endswith('.v') or filepath.endswith('.py'):
        process_file(filepath)
