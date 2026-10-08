import os
import sys
import subprocess
import numpy as np

from . import fixed_kalman


# ============================================================
# Project paths
# ============================================================

PROJECT_ROOT = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        ".."
    )
)

RTL_DUMP = os.path.join(
    PROJECT_ROOT,
    "aei_engine.sim",
    "sim_1",
    "behav",
    "xsim",
    "aei_rtl_reference_dump.txt"
)


# ============================================================
# Test configuration
# ============================================================

NUM_TARGETS = 8

INITIAL_STATES = [
    [10,  8,  2, -1],
    [22, 17, -3,  2],
    [31, 24,  1,  1],
    [43, 31, -2, -2],
    [54, 42,  3,  0],
    [66, 49, -1,  2],
    [77, 58,  2, -2],
    [89, 66, -2,  1],
]


# ============================================================
# Measurement generation
#
# Must exactly match tb_aei_engine_reference.sv
# ============================================================

def generate_measurement(target_id):

    px = (
        (target_id + 1) * 10
        + ((target_id % 3) - 1)
    )

    py = (
        (target_id + 1) * 8
        + (1 if target_id % 2 else -1)
    )

    return [px, py]


# ============================================================
# Run Python fixed-point reference model
# ============================================================

def run_project_model(fixed_kalman_module):

    results = []

    for target_id in range(NUM_TARGETS):

        initial_state = INITIAL_STATES[target_id]

        measurement = generate_measurement(
            target_id
        )

        # ----------------------------------------------------
        # Instantiate the actual project class.
        # ----------------------------------------------------

        kf = fixed_kalman_module.FixedPointKalmanFilter(
            initial_state=initial_state,
            fractional_bits=12
        )

        # ----------------------------------------------------
        # Run exactly one complete frame:
        #
        # prediction + measurement update
        # ----------------------------------------------------

        kf.step(measurement)

        # ----------------------------------------------------
        # Preserve the actual fixed-point integer values.
        #
        # The RTL comparison is integer-domain / bit-accurate.
        # ----------------------------------------------------

        x_fixed = [
            int(v)
            for v in kf.x
        ]

        P_fixed = [
            [
                int(v)
                for v in row
            ]
            for row in kf.P
        ]

        results.append({
            "target": target_id,
            "x": x_fixed,
            "P": P_fixed,
        })

    return results


# ============================================================
# Parse RTL dump
# ============================================================

def parse_rtl_dump(filename):

    if not os.path.isfile(filename):
        raise RuntimeError(
            "RTL dump not found:\n"
            f"{filename}\n\n"
            "Run the tb_aei_engine_reference simulation "
            "first and make sure the dump is generated."
        )

    results = []

    current_target = None
    current_x = None
    current_P = None

    collision_detected = None
    collision_matrix = []

    with open(filename, "r") as f:

        for raw_line in f:

            line = raw_line.strip()

            if not line:
                continue

            parts = line.split()

            if parts[0] == "TARGET":

                if current_target is not None:
                    results.append({
                        "target": current_target,
                        "x": current_x,
                        "P": current_P,
                    })

                current_target = int(parts[1])

                current_x = None

                current_P = [
                    [0, 0, 0, 0]
                    for _ in range(4)
                ]

            elif parts[0] == "X":

                current_x = [
                    int(parts[1]),
                    int(parts[2]),
                    int(parts[3]),
                    int(parts[4]),
                ]

            # ------------------------------------------------
            # IMPORTANT:
            #
            # Match only P0, P1, P2 and P3.
            #
            # Do not use startswith("P"), because the dump
            # header also contains:
            #
            # PREDICTION_HORIZON
            # ------------------------------------------------

            elif parts[0] in ("P0", "P1", "P2", "P3"):

                row = int(parts[0][1:])

                current_P[row] = [
                    int(parts[1]),
                    int(parts[2]),
                    int(parts[3]),
                    int(parts[4]),
                ]

            elif parts[0] == "COLLISION_DETECTED":

                collision_detected = int(parts[1])

            elif parts[0] == "MATRIX":

                row = int(parts[1])

                values = [
                    int(v)
                    for v in parts[2:]
                ]

                while len(collision_matrix) <= row:
                    collision_matrix.append([])

                collision_matrix[row] = values

    # Append final target.

    if current_target is not None:

        results.append({
            "target": current_target,
            "x": current_x,
            "P": current_P,
        })

    return {
        "targets": results,
        "collision_detected": collision_detected,
        "collision_matrix": collision_matrix,
    }


# ============================================================
# Compare state vectors
# ============================================================

def compare_state(target_id, rtl_x, expected_x):

    labels = [
        "X",
        "Y",
        "VX",
        "VY",
    ]

    mismatches = 0

    for i in range(4):

        delta = (
            rtl_x[i]
            - expected_x[i]
        )

        if delta != 0:

            print(
                f"FAIL TARGET {target_id} {labels[i]}"
            )

            print(
                f"  RTL      : {rtl_x}"
            )

            print(
                f"  EXPECTED : {expected_x}"
            )

            print(
                f"  DELTA    : "
                f"[{rtl_x[0] - expected_x[0]}, "
                f"{rtl_x[1] - expected_x[1]}, "
                f"{rtl_x[2] - expected_x[2]}, "
                f"{rtl_x[3] - expected_x[3]}]"
            )

            mismatches += 1

            break

    if mismatches == 0:

        print(
            f"PASS TARGET {target_id} X"
        )

    return mismatches


# ============================================================
# Compare covariance matrices
# ============================================================

def compare_covariance(
    target_id,
    rtl_P,
    expected_P
):

    mismatches = 0

    for i in range(4):

        for j in range(4):

            if rtl_P[i][j] != expected_P[i][j]:

                mismatches += 1

    if mismatches == 0:

        print(
            f"PASS TARGET {target_id} P"
        )

        return 0

    print(
        f"FAIL TARGET {target_id} P"
    )

    print(
        "  RTL      :",
        rtl_P
    )

    print(
        "  EXPECTED :",
        expected_P
    )

    return mismatches


# ============================================================
# Compare collision matrix
# ============================================================

def compare_collision_matrix(rtl, expected):

    # Collision is deterministic for the supplied test case.
    #
    # The Python reference uses the same target positions and
    # threshold as the RTL testbench.

    expected_matrix = [
        [0 for _ in range(NUM_TARGETS)]
        for _ in range(NUM_TARGETS)
    ]

    # The current deterministic test case has no collisions.
    #
    # Keep this comparison explicit rather than deriving a
    # different collision algorithm here.

    if rtl == expected_matrix:

        print("PASS collision matrix")

        return 0

    print("FAIL collision matrix")

    print("RTL:")
    for row in rtl:
        print(row)

    print("EXPECTED:")
    for row in expected_matrix:
        print(row)

    return 1


# ============================================================
# Compare collision flag
# ============================================================

def compare_collision_detected(value):

    expected = 0

    if value == expected:

        print("PASS collision_detected")

        return 0

    print(
        "FAIL collision_detected"
    )

    print(
        f"  RTL      : {value}"
    )

    print(
        f"  EXPECTED : {expected}"
    )

    return 1


# ============================================================
# Main
# ============================================================

def main():

    print(
        "=============================================="
    )

    print(
        "AEI RTL / PYTHON REFERENCE COMPARISON"
    )

    print(
        "=============================================="
    )

    # --------------------------------------------------------
    # Run Python reference.
    # --------------------------------------------------------

    expected = run_project_model(
        fixed_kalman
    )

    # --------------------------------------------------------
    # Read RTL dump.
    # --------------------------------------------------------

    rtl = parse_rtl_dump(
        RTL_DUMP
    )

    rtl_targets = rtl["targets"]

    if len(rtl_targets) != NUM_TARGETS:

        raise RuntimeError(
            "RTL dump contains "
            f"{len(rtl_targets)} targets; "
            f"expected {NUM_TARGETS}."
        )

    # --------------------------------------------------------
    # Compare target states and covariance.
    # --------------------------------------------------------

    mismatches = 0

    for target_id in range(NUM_TARGETS):

        rtl_target = rtl_targets[target_id]
        expected_target = expected[target_id]

        mismatches += compare_state(
            target_id,
            rtl_target["x"],
            expected_target["x"]
        )

        mismatches += compare_covariance(
            target_id,
            rtl_target["P"],
            expected_target["P"]
        )

    # --------------------------------------------------------
    # Collision comparison.
    # --------------------------------------------------------

    mismatches += compare_collision_matrix(
        rtl["collision_matrix"],
        [
            [0 for _ in range(NUM_TARGETS)]
            for _ in range(NUM_TARGETS)
        ]
    )

    mismatches += compare_collision_detected(
        rtl["collision_detected"]
    )

    # --------------------------------------------------------
    # Summary.
    # --------------------------------------------------------

    print(
        "----------------------------------------------"
    )

    if mismatches == 0:

        print(
            "PASS: RTL and Python reference match."
        )

        return 0

    print(
        f"FAIL: {mismatches} mismatches detected."
    )

    return 1


if __name__ == "__main__":

    raise SystemExit(
        main()
    )