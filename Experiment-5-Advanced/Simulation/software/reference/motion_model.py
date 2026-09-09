DT = 0.01


def predict_state(state):
    x, y, vx, vy = state

    x_next = x + vx * DT
    y_next = y + vy * DT
    vx_next = vx
    vy_next = vy

    return [x_next, y_next, vx_next, vy_next]

####################################################################

if __name__ == "__main__":
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

    for i, state in enumerate(targets):
        next_state = predict_state(state)
        print(f"Target {i}: {state} -> {next_state}")