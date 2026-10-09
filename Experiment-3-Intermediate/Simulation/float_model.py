# ============================================================================
# Module:        float_model.py
# Project:       FPGA Build Challenge - Experiment 3
# Target Device: AMD Xilinx Zynq-7000 SoC
#
# System Context & Top-Level Integration:
#   This Python module serves as the ideal floating-point reference model for
#   the sigmoid activation function. It provides a baseline to evaluate the 
#   quantization errors and approximation limits of the fixed-point implementation.
#
# Architectural Hierarchy:
#   1. Ideal Sigmoid Calculation: Implements the exact math.exp based sigmoid.
#   2. Error Analysis: Evaluates maximum deviation when comparing standard float
#      computations against quantized and PWL variants.
# ============================================================================
"""Floating-point golden reference for the sigmoid accelerator.

The hardware core will approximate this logistic sigmoid with piecewise
linear segments.  This file deliberately retains the exact mathematical
function so approximation error can be measured independently.
"""
import numpy as np


def golden_function(x: np.ndarray) -> np.ndarray:
    """Return the exact logistic sigmoid, ``1 / (1 + exp(-x))``.

    The split form avoids needless overflow warnings for large-magnitude
    values while preserving NumPy scalar and vector input support.
    """
    values = np.asarray(x, dtype=np.float64)
    positive = values >= 0.0
    result = np.empty_like(values)
    result[positive] = 1.0 / (1.0 + np.exp(-values[positive]))
    exp_x = np.exp(values[~positive])
    result[~positive] = exp_x / (1.0 + exp_x)
    return result


if __name__ == "__main__":
    x = np.array([-8.0, -4.0, -2.0, -1.0, 0.0, 1.0, 2.0, 4.0, 8.0])
    y = golden_function(x)
    for xi, yi in zip(x, y):
        print(f"x={xi:+6.3f}  sigmoid(x)={yi:.8f}")

