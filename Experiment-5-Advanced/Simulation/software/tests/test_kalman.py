from reference.kalman import KalmanFilter
from reference.measurement_model import measure_state


DT = 0.01
STEPS = 200

true_state = [30.0, 10.0, -3.0, 4.0]

kf = KalmanFilter([30.0, 10.0, 0.0, 0.0])


position_error_sum = 0.0
velocity_error_sum = 0.0


for i in range(STEPS):

    measurement = measure_state(true_state)

    estimate = kf.step(measurement)

    position_error = (
        (estimate[0] - true_state[0]) ** 2
        + (estimate[1] - true_state[1]) ** 2
    ) ** 0.5

    velocity_error = (
        (estimate[2] - true_state[2]) ** 2
        + (estimate[3] - true_state[3]) ** 2
    ) ** 0.5

    position_error_sum += position_error
    velocity_error_sum += velocity_error

    if i % 20 == 0 or i == STEPS - 1:
        print(f"Step {i:3d}")
        print(f"  True:        {true_state}")
        print(f"  Measurement: {measurement}")
        print(f"  Estimate:    {estimate}")
        print(f"  Position error: {position_error:.4f}")
        print(f"  Velocity error: {velocity_error:.4f}")
        print()

    true_state[0] += true_state[2] * DT
    true_state[1] += true_state[3] * DT


print("=== Summary ===")

print(
    f"Average position error: "
    f"{position_error_sum / STEPS:.4f}"
)

print(
    f"Average velocity error: "
    f"{velocity_error_sum / STEPS:.4f}"
)
