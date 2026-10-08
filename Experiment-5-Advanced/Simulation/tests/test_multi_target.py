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
    initial_estimate = [state[0], state[1], 0.0, 0.0]
    filters.append(KalmanFilter(initial_estimate))


position_error_sums = [0.0] * len(true_states)
velocity_error_sums = [0.0] * len(true_states)


for step in range(STEPS):

    for target_id in range(len(true_states)):

        true_state = true_states[target_id]

        measurement = measure_state(true_state)

        estimate = filters[target_id].step(measurement)

        position_error = (
            (estimate[0] - true_state[0]) ** 2
            + (estimate[1] - true_state[1]) ** 2
        ) ** 0.5

        velocity_error = (
            (estimate[2] - true_state[2]) ** 2
            + (estimate[3] - true_state[3]) ** 2
        ) ** 0.5

        position_error_sums[target_id] += position_error
        velocity_error_sums[target_id] += velocity_error

        true_state[0] += true_state[2] * DT
        true_state[1] += true_state[3] * DT


print("=== Multi-Target Summary ===")

for target_id in range(len(true_states)):

    average_position_error = (
        position_error_sums[target_id] / STEPS
    )

    average_velocity_error = (
        velocity_error_sums[target_id] / STEPS
    )

    print(
        f"Target {target_id}: "
        f"Average position error = "
        f"{average_position_error:.4f}, "
        f"Average velocity error = "
        f"{average_velocity_error:.4f}"
    )
