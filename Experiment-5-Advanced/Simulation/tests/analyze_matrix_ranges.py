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


# Range trackers
p_min = np.full((4, 4), np.inf)
p_max = np.full((4, 4), -np.inf)

k_min = np.full((4, 2), np.inf)
k_max = np.full((4, 2), -np.inf)


for step in range(STEPS):

    for target_id in range(len(true_states)):

        true_state = true_states[target_id]

        measurement = measure_state(true_state)

        kf = filters[target_id]

        # Prediction
        kf.predict()

        # Calculate Kalman gain before update
        S = H @ kf.P @ H.T + R
        K = kf.P @ H.T @ np.linalg.inv(S)

        # Record P range
        p_min = np.minimum(p_min, kf.P)
        p_max = np.maximum(p_max, kf.P)

        # Record K range
        k_min = np.minimum(k_min, K)
        k_max = np.maximum(k_max, K)

        # Complete update
        kf.update(measurement)

        # Update true state
        true_state[0] += true_state[2] * DT
        true_state[1] += true_state[3] * DT


print("=== Covariance P Range ===")
print()

for row in range(4):
    for col in range(4):
        print(
            f"P[{row},{col}] : "
            f"min = {p_min[row, col]: .8f}, "
            f"max = {p_max[row, col]: .8f}"
        )


print()
print("=== Kalman Gain K Range ===")
print()

for row in range(4):
    for col in range(2):
        print(
            f"K[{row},{col}] : "
            f"min = {k_min[row, col]: .8f}, "
            f"max = {k_max[row, col]: .8f}"
        )


print()
print("=== Constant Matrices ===")
print()

print("F =")
print(F)

print()
print("H =")
print(H)

print()
print("Q =")
print(Q)

print()
print("R =")
print(R)