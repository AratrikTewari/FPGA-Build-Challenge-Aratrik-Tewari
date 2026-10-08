import numpy as np

from reference.kalman import KalmanFilter, F, H, Q, R
from reference.measurement_model import measure_state


DT = 0.01
STEPS = 200

initial_targets = [
    [10.0, 20.0, 5.0, -2.0],
    [30.0, 10.0, -3.0, 4.0],
    [50.0, 50.0, 2.0, 1.0],
    [70.0, 20.0, -4.0, 3.0],
    [20.0, 80.0, 1.0, -5.0],
    [80.0, 70.0, -2.0, -1.0],
    [40.0, 30.0, 3.0, 2.0],
    [60.0, 90.0, -1.0, -3.0],
]


true_states = [state.copy() for state in initial_targets]

filters = []

for state in initial_targets:
    filters.append(
        KalmanFilter([state[0], state[1], 0.0, 0.0])
    )


ranges = {}


def record(name, value):
    value = np.asarray(value, dtype=np.float64)

    if name not in ranges:
        ranges[name] = {
            "min": np.full(value.shape, np.inf),
            "max": np.full(value.shape, -np.inf)
        }

    ranges[name]["min"] = np.minimum(
        ranges[name]["min"],
        value
    )

    ranges[name]["max"] = np.maximum(
        ranges[name]["max"],
        value
    )


for step in range(STEPS):

    for target_id in range(len(true_states)):

        true_state = true_states[target_id]
        kf = filters[target_id]

        measurement = measure_state(true_state)

        # Prediction
        x = kf.x
        P = kf.P

        x_pred = F @ x

        FP = F @ P
        P_pred = FP @ F.T
        P_pred_Q = P_pred + Q

        record("x_pred", x_pred)
        record("F @ P", FP)
        record("F @ P @ F.T", P_pred)
        record("P_pred + Q", P_pred_Q)

        # Use predicted covariance for update
        P = P_pred_Q

        # Innovation
        innovation = measurement - H @ x_pred

        HP = H @ P
        S = HP @ H.T + R

        PHt = P @ H.T

        S_inv = np.linalg.inv(S)

        K = PHt @ S_inv

        K_innovation = K @ innovation

        x_updated = x_pred + K_innovation

        I = np.eye(4)

        KH = K @ H
        I_KH = I - KH
        P_updated = I_KH @ P

        # Record important intermediate values
        record("measurement", measurement)
        record("innovation", innovation)
        record("H @ P", HP)
        record("P @ H.T", PHt)
        record("S", S)
        record("S inverse", S_inv)
        record("K", K)
        record("K @ innovation", K_innovation)
        record("x_updated", x_updated)
        record("K @ H", KH)
        record("I - K @ H", I_KH)
        record("P_updated", P_updated)

        # Update the real floating-point filter
        kf.x = x_updated
        kf.P = P_updated

        # Update true state
        true_state[0] += true_state[2] * DT
        true_state[1] += true_state[3] * DT


print("=== Intermediate Numerical Ranges ===")
print()

for name, data in ranges.items():

    minimum = np.min(data["min"])
    maximum = np.max(data["max"])

    print(
        f"{name:<20} "
        f"min = {minimum: .10f}   "
        f"max = {maximum: .10f}"
    )


print()
print("=== Largest Absolute Values ===")
print()

for name, data in ranges.items():

    maximum_absolute = max(
        abs(np.min(data["min"])),
        abs(np.max(data["max"]))
    )

    print(
        f"{name:<20} "
        f"max |value| = {maximum_absolute:.10f}"
    )