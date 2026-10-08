import matplotlib.pyplot as plt

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


true_history = [[] for _ in initial_targets]
measurement_history = [[] for _ in initial_targets]
estimate_history = [[] for _ in initial_targets]


for step in range(STEPS):

    for target_id in range(len(true_states)):

        true_state = true_states[target_id]

        measurement = measure_state(true_state)
        estimate = filters[target_id].step(measurement)

        true_history[target_id].append(
            [true_state[0], true_state[1]]
        )

        measurement_history[target_id].append(
            measurement
        )

        estimate_history[target_id].append(
            [estimate[0], estimate[1]]
        )

        true_state[0] += true_state[2] * DT
        true_state[1] += true_state[3] * DT


plt.figure(figsize=(10, 8))

for target_id in range(len(initial_targets)):

    true_x = [p[0] for p in true_history[target_id]]
    true_y = [p[1] for p in true_history[target_id]]

    measurement_x = [p[0] for p in measurement_history[target_id]]
    measurement_y = [p[1] for p in measurement_history[target_id]]

    estimate_x = [p[0] for p in estimate_history[target_id]]
    estimate_y = [p[1] for p in estimate_history[target_id]]

    plt.plot(
        true_x,
        true_y,
        label=f"Target {target_id} True"
    )

    plt.scatter(
        measurement_x,
        measurement_y,
        s=8,
        alpha=0.25
    )

    plt.plot(
        estimate_x,
        estimate_y,
        linestyle="--",
        label=f"Target {target_id} Estimate"
    )


plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("8-Target Kalman Filter Tracking")
plt.grid(True)
plt.legend(fontsize=8)
plt.tight_layout()

plt.show()