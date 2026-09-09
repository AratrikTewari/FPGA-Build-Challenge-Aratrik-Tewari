import numpy as np

# ============================================================
# Configuration
# ============================================================

DT = 0.01

Q = np.array([
    [0.01, 0.00, 0.00, 0.00],
    [0.00, 0.01, 0.00, 0.00],
    [0.00, 0.00, 0.10, 0.00],
    [0.00, 0.00, 0.00, 0.10]
])

R = np.array([
    [1.0, 0.0],
    [0.0, 1.0]
])

F = np.array([
    [1.0, 0.0, DT, 0.0],
    [0.0, 1.0, 0.0, DT],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]
])

H = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0]
])


# ============================================================
# Range tracking
# ============================================================

ranges = {}


def track(name, value):
    value = np.asarray(value)

    if name not in ranges:
        ranges[name] = {
            "min": np.min(value),
            "max": np.max(value),
            "max_abs": np.max(np.abs(value))
        }
    else:
        ranges[name]["min"] = min(
            ranges[name]["min"],
            np.min(value)
        )

        ranges[name]["max"] = max(
            ranges[name]["max"],
            np.max(value)
        )

        ranges[name]["max_abs"] = max(
            ranges[name]["max_abs"],
            np.max(np.abs(value))
        )


# ============================================================
# Initial state
# ============================================================

x = np.array([
    30.0,
    10.0,
    0.0,
    0.0
])

P = np.eye(4) * 10.0


# ============================================================
# Targets
# ============================================================

targets = [
    [10.0, 20.0, 5.0, -2.0],
    [30.0, 10.0, -3.0, 4.0],
    [50.0, 50.0, 2.0, 1.0],
    [70.0, 20.0, -4.0, 3.0],
    [20.0, 80.0, 1.0, -5.0],
    [80.0, 70.0, -2.0, -1.0],
    [40.0, 30.0, 3.0, 2.0],
    [60.0, 90.0, -1.0, -3.0]
]


# ============================================================
# Run all targets
# ============================================================

for target in targets:

    true_state = np.array(target, dtype=float)

    x = np.array([
        30.0,
        10.0,
        0.0,
        0.0
    ])

    P = np.eye(4) * 10.0

    for step in range(200):

        # ----------------------------------------------------
        # True motion
        # ----------------------------------------------------

        true_state = F @ true_state

        # ----------------------------------------------------
        # Measurement
        # ----------------------------------------------------

        measurement = (
            H @ true_state
            + np.random.normal(0.0, 1.0, 2)
        )

        # ----------------------------------------------------
        # Prediction
        # ----------------------------------------------------

        x_pred = F @ x

        FP = F @ P
        FPFt = FP @ F.T

        P_pred = FPFt + Q

        # ----------------------------------------------------
        # Innovation
        # ----------------------------------------------------

        Hx = H @ x_pred

        innovation = measurement - Hx

        HP = H @ P_pred
        PHt = P_pred @ H.T

        S = HP @ H.T + R

        # ----------------------------------------------------
        # Reciprocal / inverse
        #
        # S is diagonal for this system.
        # ----------------------------------------------------

        S_inv = np.linalg.inv(S)

        # ----------------------------------------------------
        # Kalman gain
        # ----------------------------------------------------

        K = PHt @ S_inv

        Kinnovation = K @ innovation

        # ----------------------------------------------------
        # State update
        # ----------------------------------------------------

        x_updated = x_pred + Kinnovation

        KH = K @ H

        I_KH = np.eye(4) - KH

        P_updated = I_KH @ P_pred

        # ----------------------------------------------------
        # Track important values
        # ----------------------------------------------------

        track("x_pred", x_pred)
        track("F @ P", FP)
        track("F @ P @ F.T", FPFt)
        track("P_pred + Q", P_pred)

        track("measurement", measurement)
        track("innovation", innovation)

        track("H @ P", HP)
        track("P @ H.T", PHt)

        track("S", S)
        track("S inverse", S_inv)

        track("K", K)
        track("K @ innovation", Kinnovation)

        track("x_updated", x_updated)

        track("K @ H", KH)
        track("I - K @ H", I_KH)

        track("P_updated", P_updated)

        # ----------------------------------------------------
        # Update
        # ----------------------------------------------------

        x = x_updated
        P = P_updated


# ============================================================
# Print results
# ============================================================

print()
print("=== Hardware Width Verification ===")
print()

print(
    f"{'Signal':<22}"
    f"{'Min':>14}"
    f"{'Max':>14}"
    f"{'Max |value|':>16}"
)

print("-" * 68)

for name, data in ranges.items():

    print(
        f"{name:<22}"
        f"{data['min']:>14.6f}"
        f"{data['max']:>14.6f}"
        f"{data['max_abs']:>16.6f}"
    )


# ============================================================
# Fixed-point limits
# ============================================================

print()
print("=== Proposed Fixed-Point Limits ===")
print()

# Signed 20-bit Q8.12
state_bits = 20
state_fractional = 12

state_min = -(2 ** (state_bits - 1)) / (2 ** state_fractional)
state_max = (
    (2 ** (state_bits - 1)) - 1
) / (2 ** state_fractional)

print(
    f"20-bit Q8.12 range: "
    f"{state_min:.6f} to {state_max:.6f}"
)


# Signed 16-bit Q0.12
recip_bits = 16
recip_fractional = 12

recip_min = -(2 ** (recip_bits - 1)) / (2 ** recip_fractional)
recip_max = (
    (2 ** (recip_bits - 1)) - 1
) / (2 ** recip_fractional)

print(
    f"16-bit Q0.12 range: "
    f"{recip_min:.6f} to {recip_max:.6f}"
)


# 40-bit signed product
product_bits = 40

product_max = 2 ** (product_bits - 1) - 1

print(
    f"40-bit signed raw maximum: "
    f"{product_max}"
)


# 44-bit accumulator
accumulator_bits = 44

accumulator_max = 2 ** (accumulator_bits - 1) - 1

print(
    f"44-bit signed raw maximum: "
    f"{accumulator_max}"
)


print()
print("Verification complete.")

