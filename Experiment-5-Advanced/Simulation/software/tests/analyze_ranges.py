from reference.kalman import KalmanFilter
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


minimum = [float("inf")] * 4
maximum = [float("-inf")] * 4


for step in range(STEPS):

    for target_id in range(len(true_states)):

        true_state = true_states[target_id]

        measurement = measure_state(true_state)

        estimate = filters[target_id].step(measurement)

        for i in range(4):
            minimum[i] = min(minimum[i], estimate[i])
            maximum[i] = max(maximum[i], estimate[i])

        true_state[0] += true_state[2] * DT
        true_state[1] += true_state[3] * DT


names = ["x", "y", "vx", "vy"]

print("=== Estimated State Ranges ===")

for i in range(4):
    print(
        f"{names[i]:>2}: "
        f"min = {minimum[i]:10.4f}, "
        f"max = {maximum[i]:10.4f}"
    )