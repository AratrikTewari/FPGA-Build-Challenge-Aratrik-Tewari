import random


NOISE_STD = 1.0
RANDOM_SEED = 42

random.seed(RANDOM_SEED)


def measure_state(state):
    x, y, vx, vy = state

    x_measured = x + random.gauss(0, NOISE_STD)
    y_measured = y + random.gauss(0, NOISE_STD)

    return [x_measured, y_measured]


if __name__ == "__main__":
    state = [30.0, 10.0, -3.0, 4.0]

    for i in range(5):
        measurement = measure_state(state)
        print(f"Measurement {i}: {measurement}")
