import numpy as np

from reference.fixed_kalman import FixedPointKalmanFilter
from reference.measurement_model import measure_state


# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------

FRACTIONAL_BITS = 12
STEPS = 200

targets = [
    [10.0, 20.0, 5.0, -2.0],
    [30.0, 10.0, -3.0, 4.0],
    [50.0, 50.0, 2.0, 1.0],
    [70.0, 20.0, -4.0, 3.0],
    [20.0, 80.0, 1.0, -5.0],
    [80.0, 70.0, -2.0, -1.0],
    [40.0, 30.0, 3.0, 2.0],
    [60.0, 90.0, -1.0, -3.0],
]


# ------------------------------------------------------------
# Create filters
# ------------------------------------------------------------

filters = []

for target in targets:

    initial_state = [
        target[0],
        target[1],
        0.0,
        0.0
    ]

    filters.append(
        FixedPointKalmanFilter(
            initial_state,
            fractional_bits=FRACTIONAL_BITS
        )
    )


# ------------------------------------------------------------
# Track ranges
# ------------------------------------------------------------

s00_min = float("inf")
s00_max = float("-inf")

s11_min = float("inf")
s11_max = float("-inf")

s01_min = float("inf")
s01_max = float("-inf")

s10_min = float("inf")
s10_max = float("-inf")

max_off_diagonal = 0.0


# ------------------------------------------------------------
# Run simulation
# ------------------------------------------------------------

for step in range(STEPS):

    for target_id, target in enumerate(targets):

        kf = filters[target_id]

        # ----------------------------------------------------
        # Prediction
        # ----------------------------------------------------

        kf.predict()

        P = kf.to_float(kf.P)

        H = kf.to_float(kf.H)
        R = kf.to_float(kf.R)

        # ----------------------------------------------------
        # Innovation covariance
        #
        # S = H P H^T + R
        # ----------------------------------------------------

        S = H @ P @ H.T + R

        s00 = S[0, 0]
        s01 = S[0, 1]
        s10 = S[1, 0]
        s11 = S[1, 1]

        # ----------------------------------------------------
        # Update ranges
        # ----------------------------------------------------

        s00_min = min(s00_min, s00)
        s00_max = max(s00_max, s00)

        s11_min = min(s11_min, s11)
        s11_max = max(s11_max, s11)

        s01_min = min(s01_min, s01)
        s01_max = max(s01_max, s01)

        s10_min = min(s10_min, s10)
        s10_max = max(s10_max, s10)

        max_off_diagonal = max(
            max_off_diagonal,
            abs(s01),
            abs(s10)
        )

        # ----------------------------------------------------
        # Perform normal Kalman update
        # ----------------------------------------------------

        measurement = measure_state(target)

        kf.update(measurement)

        # ----------------------------------------------------
        # Update true target
        # ----------------------------------------------------

        target[0] += target[2] * 0.01
        target[1] += target[3] * 0.01


# ------------------------------------------------------------
# Print results
# ------------------------------------------------------------

print()
print("=== Q12 Innovation Covariance S ===")
print()

print(
    f"S[0,0] : min = {s00_min: .10f}, "
    f"max = {s00_max: .10f}"
)

print(
    f"S[0,1] : min = {s01_min: .10f}, "
    f"max = {s01_max: .10f}"
)

print(
    f"S[1,0] : min = {s10_min: .10f}, "
    f"max = {s10_max: .10f}"
)

print(
    f"S[1,1] : min = {s11_min: .10f}, "
    f"max = {s11_max: .10f}"
)

print()
print("=== Off-Diagonal Check ===")
print()

print(
    f"Maximum |off-diagonal| = "
    f"{max_off_diagonal:.12f}"
)

print()

if max_off_diagonal == 0.0:

    print("RESULT: S is exactly diagonal.")
    print()
    print("We can replace the general 2x2 inverse with:")
    print()
    print("    inv_S00 = 1 / S00")
    print("    inv_S11 = 1 / S11")
    print()
    print("No determinant calculation is required.")
    print("No general 2x2 matrix inverse is required.")

else:

    print("RESULT: S is not exactly diagonal.")
    print()
    print("The general 2x2 inverse must be retained.")

    if max_off_diagonal < 0.001:
        print()
        print(
            "However, the off-diagonal terms are very small."
        )
        print(
            "A specialized approximation may still be possible."
        )

