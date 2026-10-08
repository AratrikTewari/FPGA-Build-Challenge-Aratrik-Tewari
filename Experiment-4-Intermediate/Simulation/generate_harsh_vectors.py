from pathlib import Path


def next_state(state: int, width: int = 32) -> int:
    result = 0
    for i in range(width):
        center = (state >> i) & 1
        left = (state >> ((i + 1) % width)) & 1
        right = (state >> ((i - 1) % width)) & 1
        bit = left ^ (center | right)
        result |= (bit << i)
    return result & ((1 << width) - 1)


def run(seed: int, generations: int, width: int = 32):
    state = seed & ((1 << width) - 1)
    yield state
    for _ in range(generations):
        state = next_state(state, width)
        yield state


for seed in [0x00000001, 0xA5A5A5A5, 0x12345678]:
    path = Path(f"expected_harsh_seed_{seed:08x}.hex")
    with path.open("w") as f:
        for state in run(seed, 64):
            f.write(f"{state:08x}\n")
    print(f"Wrote {path}")
