# ============================================================================
# Module:        fixed_point_model.py
# Project:       FPGA Build Challenge - Experiment 3
# Target Device: AMD Xilinx Zynq-7000 SoC
#
# System Context & Top-Level Integration:
#   This Python module serves as the fixed-point reference model for the 
#   Q4.12 piecewise-linear (PWL) sigmoid core. It validates mathematical 
#   correctness before RTL implementation.
#
# Architectural Hierarchy:
#   1. Fixed-Point Conversion: Utilities for floating-point to Q-format.
#   2. PWL Approximation Model: Python equivalent of the Verilog implementation,
#      featuring bit-exact logic mapping.
#   3. Test Vector Generation: Outputs deterministic stimuli for RTL testbenches.
# ============================================================================
"""
STEP 2 — Fixed-point / hardware-shape model.

This mirrors EXACTLY what the Verilog will do: same thresholds, same
shift amounts, same additive constants, same Q4.12 rounding. If this
model's error vs. float_model.py is acceptable, the RTL (step 4) has
no excuse to disagree — any mismatch there is a hardware bug, not an
algorithm limitation.
"""
import numpy as np
from float_model import golden_function

Q_FRAC_BITS = 12
Q_ONE = 1 << Q_FRAC_BITS  # 4096
Q_INPUT_MIN = -8.0
Q_INPUT_MAX = (0x7FFF / Q_ONE)

# Segment boundaries and intercepts in Q4.12.  Every gain is a right shift,
# so the accelerator needs only comparators, adders, and wiring shifts.
SATURATION_THRESHOLD = 0x5000  # 5.0
MID_THRESHOLD = 0x2600         # 2.375
LOW_THRESHOLD = 0x1000         # 1.0


def float_to_fixed(x: float) -> int:
    """Convert a representable value to its unsigned 16-bit Q4.12 encoding."""
    if not Q_INPUT_MIN <= x <= Q_INPUT_MAX:
        raise ValueError(f"{x} is outside the Q4.12 range [{Q_INPUT_MIN}, {Q_INPUT_MAX}]")
    return int(round(x * Q_ONE)) & 0xFFFF


def fixed_to_float(x: int) -> float:
    if x >= 32768:
        x -= 65536
    return x / Q_ONE


def plan_pwl_fixed(x_fixed: int) -> int:
    """
    Piecewise-linear approximation of sigmoid, hardware-shape (shifts + adds).
    Mirrors hardware/src/core/sigmoid_pwl.v exactly — keep these two in sync.
    """
    sign = 1 if x_fixed >= 32768 else 0
    abs_x = (65536 - x_fixed) if sign else x_fixed
    abs_x &= 0xFFFF

    if abs_x >= SATURATION_THRESHOLD:  # |x| >= 5.0
        y_pos = 0x1000            # 1.0
    elif abs_x >= MID_THRESHOLD:  # |x| >= 2.375 -> y = x/32 + 0.84375
        y_pos = (abs_x >> 5) + 3456
    elif abs_x >= LOW_THRESHOLD:  # |x| >= 1.0 -> y = x/8 + 0.625
        y_pos = (abs_x >> 3) + 2560
    else:                         # |x| < 1.0     -> y = x/4 + 0.5
        y_pos = (abs_x >> 2) + 2048

    result = (0x1000 - y_pos) if sign else y_pos
    return result & 0xFFFF


if __name__ == "__main__":
    # Includes every segment boundary and a dense full demo-domain sweep.
    x_vals = np.unique(np.concatenate((
        np.linspace(-6.0, 6.0, 24_001),
        np.array([-5.0, -2.375, -1.0, 0.0, 1.0, 2.375, 5.0]),
    )))
    max_err = -1.0
    max_err_x = 0.0
    for x in x_vals:
        y_true = golden_function(x)
        y_hw = fixed_to_float(plan_pwl_fixed(float_to_fixed(x)))
        err = abs(y_true - y_hw)
        if err > max_err:
            max_err = float(err)
            max_err_x = float(x)

    print("PWL sigmoid: 4 symmetric segments, Q4.12, shift/add only")
    print(f"Sweep: {len(x_vals)} points over [-6.0, +6.0]")
    print(f"Max absolute error: {max_err:.6f} at x={max_err_x:+.4f}")
    if max_err > 0.02:
        print("WARNING: error exceeds 0.02; add PWL segments for a more believable demo.")
    else:
        print("PASS: error is at or below 0.02, suitable for a visual sigmoid demo.")

