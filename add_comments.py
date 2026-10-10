import os
import glob

v_comment = """`timescale 1ns / 1ps

// ============================================================================
// Module:        {filename}
// Project:       FPGA Build Challenge - Experiment 4
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   This module is part of the Rule 30 Cellular Automaton implementation,
//   responsible for generating pseudo-random sequences.
//
// Architectural Hierarchy:
//   Part of the Experiment 4 RTL / Simulation / Testbench ecosystem.
// ============================================================================

"""

py_comment = """# ============================================================================
# Module:        {filename}
# Project:       FPGA Build Challenge - Experiment 4
# Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
#
# System Context & Top-Level Integration:
#   This Python script is part of the Rule 30 Cellular Automaton ecosystem,
#   used for reference modeling or generating test vectors.
# ============================================================================

"""

xdc_comment = """# ============================================================================
# Constraints:    {filename}
# Project:       FPGA Build Challenge - Experiment 4
# Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
#
# System Context & Top-Level Integration:
#   Physical and timing constraints for the Rule 30 implementation.
# ============================================================================

"""

def process_file(filepath):
    filename = os.path.basename(filepath)
    ext = os.path.splitext(filename)[1]
    
    with open(filepath, 'r') as f:
        content = f.read()
        
    if ext == '.v':
        lines = content.split('\n')
        new_lines = []
        for line in lines:
            if line.startswith('`timescale'):
                continue
            new_lines.append(line)
        content = '\n'.join(new_lines).strip()
        new_content = v_comment.format(filename=filename.replace('.v', '')) + content + '\n'
    elif ext == '.py':
        new_content = py_comment.format(filename=filename) + content
    elif ext == '.xdc':
        new_content = xdc_comment.format(filename=filename) + content
    else:
        return False
        
    with open(filepath, 'w') as f:
        f.write(new_content)
        
    return True

files = glob.glob('Experiment-4-Intermediate/RTL/*.*') + \
        glob.glob('Experiment-4-Intermediate/Simulation/*.*') + \
        glob.glob('Experiment-4-Intermediate/Testbench/*.*')

for filepath in files:
    if filepath.endswith('.v') or filepath.endswith('.py') or filepath.endswith('.xdc'):
        process_file(filepath)
