#!/usr/bin/env python3
"""
golden_model.py
----------------
Pure-Python bit-exact reference model of the Rule 30 CA PRNG core.
This is the SOURCE OF TRUTH for the hardware design. Write this first,
verify it does what you expect, then make the Verilog match it exactly.

Rule 30:  next(i) = left(i) XOR ( center(i) OR right(i) )
Boundary: circular (the register wraps around: bit WIDTH-1's "left"
          neighbor is bit 0, and bit 0's "right" neighbor is bit WIDTH-1)

Usage:
    python3 golden_model.py                       # prints a demo trace
    python3 golden_model.py --gen_vectors N        # writes tb vectors for Verilog
"""

import argparse
import sys

WIDTH = 32


def next_state(state: int, width: int = WIDTH) -> int:
    """Compute one Rule 30 generation with circular boundary conditions."""
    result = 0
    for i in range(width):
        center = (state >> i) & 1
        left = (state >> ((i + 1) % width)) & 1     # neighbor with higher index
        right = (state >> ((i - 1) % width)) & 1    # neighbor with lower index
        bit = left ^ (center | right)
        result |= (bit << i)
    return result & ((1 << width) - 1)


def run(seed: int, generations: int, width: int = WIDTH):
    """Yield `generations` states starting from `seed` (seed is generation 0)."""
    state = seed & ((1 << width) - 1)
    yield state
    for _ in range(generations):
        state = next_state(state, width)
        yield state


def print_trace(seed: int, generations: int, width: int = WIDTH):
    print(f"Rule 30 CA trace | WIDTH={width} | seed=0x{seed:08X}\n")
    for gen, state in enumerate(run(seed, generations, width)):
        bits = format(state, f"0{width}b")
        print(f"gen {gen:3d} | 0x{state:08X} | {bits}")


def write_verilog_vectors(seed: int, generations: int, path: str, width: int = WIDTH):
    """
    Write a $readmemh-compatible hex file: one state per line, generation 0
    (the seed) through generation N. The Verilog testbench loads this and
    compares it cycle-by-cycle against the RTL's prng_out.
    """
    with open(path, "w") as f:
        for state in run(seed, generations, width):
            f.write(f"{state:08x}\n")
    print(f"Wrote {generations + 1} expected states to {path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=lambda x: int(x, 0), default=0xDEADBEEF)
    ap.add_argument("--generations", type=int, default=32)
    ap.add_argument("--width", type=int, default=WIDTH)
    ap.add_argument("--gen_vectors", metavar="PATH",
                     help="Write expected-state hex vectors to this file for the Verilog testbench")
    args = ap.parse_args()

    if args.gen_vectors:
        write_verilog_vectors(args.seed, args.generations, args.gen_vectors, args.width)
    else:
        print_trace(args.seed, args.generations, args.width)
