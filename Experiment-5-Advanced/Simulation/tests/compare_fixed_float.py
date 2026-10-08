import numpy as np

from reference.kalman import KalmanFilter
from reference.fixed_kalman import FixedPointKalmanFilter
from reference.measurement_model import measure_state


DT = 0.01
STEPS = 200

INITIAL_STATE = [30.0, 10.0, -3.0, 4.0]

FORMATS = [8, 12, 16]


# ============================================================
# Generate ONE common measurement sequence
# ============================================================

def generate_measurements():

    true_state = INITIAL_STATE.copy()

    measurements = []

    for step in range(STEPS):

        measurement = measure_state(true_state)

        measurements.append(
            np.array(measurement, dtype=float)
        )

        true_state[0] += true_state[2] * DT
        true_state[1] += true_state[3] * DT

    return measurements


# ============================================================
# Run one fixed-point format using the same measurements
# ============================================================

def run_test(fractional_bits, measurements):

    float_kf = KalmanFilter(
        [30.0, 10.0, 0.0, 0.0]
    )

    fixed_kf = FixedPointKalmanFilter(
        [30.0, 10.0, 0.0, 0.0],
        fractional_bits=fractional_bits
    )

    position_difference_sum = 0.0
    velocity_difference_sum = 0.0

    max_position_difference = 0.0
    max_velocity_difference = 0.0

    for measurement in measurements:

        float_estimate = float_kf.step(
            measurement
        )

        fixed_estimate = fixed_kf.step(
            measurement
        )

        position_difference = (
            (float_estimate[0] - fixed_estimate[0]) ** 2
            +
            (float_estimate[1] - fixed_estimate[1]) ** 2
        ) ** 0.5

        velocity_difference = (
            (float_estimate[2] - fixed_estimate[2]) ** 2
            +
            (float_estimate[3] - fixed_estimate[3]) ** 2
        ) ** 0.5

        position_difference_sum += (
            position_difference
        )

        velocity_difference_sum += (
            velocity_difference
        )

        max_position_difference = max(
            max_position_difference,
            position_difference
        )

        max_velocity_difference = max(
            max_velocity_difference,
            velocity_difference
        )

    return (
        position_difference_sum / STEPS,
        velocity_difference_sum / STEPS,
        max_position_difference,
        max_velocity_difference
    )


# ============================================================
# Main
# ============================================================

print("=== Fixed-Point Precision Sweep ===")
print()

# Generate measurements exactly once.
measurements = generate_measurements()

print(
    f"{'Format':<10}"
    f"{'Resolution':<15}"
    f"{'Avg Pos Diff':<18}"
    f"{'Avg Vel Diff':<18}"
    f"{'Max Pos Diff':<18}"
    f"{'Max Vel Diff':<18}"
)

print("-" * 95)


for fractional_bits in FORMATS:

    avg_pos, avg_vel, max_pos, max_vel = run_test(
        fractional_bits,
        measurements
    )

    resolution = 2 ** (-fractional_bits)

    print(
        f"Q8.{fractional_bits:<6}"
        f"{resolution:<15.8f}"
        f"{avg_pos:<18.6f}"
        f"{avg_vel:<18.6f}"
        f"{max_pos:<18.6f}"
        f"{max_vel:<18.6f}"
    )


print()
print("Sweep complete.")
