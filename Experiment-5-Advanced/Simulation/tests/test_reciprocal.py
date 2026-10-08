import numpy as np


FRACTIONAL_BITS = 12
SCALE = 1 << FRACTIONAL_BITS


def to_fixed(value):
    return int(round(value * SCALE))


def to_float(value):
    return value / SCALE


def reciprocal_newton(x, iterations):
    """
    Newton-Raphson reciprocal approximation.

    r_next = r * (2 - x*r)

    We start from the exact reciprocal deliberately so that
    this function lets us study convergence independently of
    LUT-seed error.
    """

    r = 1.0 / x

    for _ in range(iterations):
        r = r * (2.0 - x * r)

    return r


def reciprocal_from_seed(x, seed, iterations):
    """
    Newton-Raphson reciprocal using a supplied initial seed.
    """

    r = seed

    for _ in range(iterations):
        r = r * (2.0 - x * r)

    return r


def generate_lut(lut_bits):
    """
    Generate a reciprocal LUT covering normalized inputs
    from 1.0 up to, but not including, 2.0.

    The LUT stores Q12 reciprocal values.
    """

    entries = 1 << lut_bits

    lut = []

    for address in range(entries):

        normalized_x = 1.0 + (
            address + 0.5
        ) / entries

        reciprocal = 1.0 / normalized_x

        fixed_value = to_fixed(reciprocal)

        lut.append(
            to_float(fixed_value)
        )

    return lut


def lookup_seed(x, lut, lut_bits):
    """
    Normalize x to:

        x = normalized_x * 2^exponent

    with:

        1 <= normalized_x < 2

    Then obtain an approximate reciprocal from the LUT.
    """

    exponent = int(np.floor(np.log2(x)))

    normalized_x = x / (2.0 ** exponent)

    entries = 1 << lut_bits

    address = int(
        (normalized_x - 1.0) * entries
    )

    if address < 0:
        address = 0

    if address >= entries:
        address = entries - 1

    seed_normalized = lut[address]

    # Undo normalization:
    seed = seed_normalized / (2.0 ** exponent)

    return seed


def test_lut_configuration(
    lut_bits,
    iterations,
    test_values
):

    lut = generate_lut(lut_bits)

    errors = []

    for x in test_values:

        exact = 1.0 / x

        seed = lookup_seed(
            x,
            lut,
            lut_bits
        )

        result = reciprocal_from_seed(
            x,
            seed,
            iterations
        )

        # Quantize final result to Q12
        result_fixed = to_fixed(result)
        result_q12 = to_float(result_fixed)

        error = abs(
            result_q12 - exact
        )

        errors.append(error)

    return max(errors), np.mean(errors)


# ------------------------------------------------------------
# Test range
# ------------------------------------------------------------

# Our measured S range was approximately:
#
#     1.1367 ... 11.0110
#
# Use many test points to cover this range.

test_values = np.linspace(
    1.13671875,
    11.0109863281,
    10000
)


# ------------------------------------------------------------
# Sweep LUT sizes and iterations
# ------------------------------------------------------------

print()
print("=== Reciprocal Hardware Sweep ===")
print()

print(
    "LUT bits   Entries   Iterations   "
    "Max error       Average error"
)

print("-" * 65)


configurations = [
    (4, 1),
    (4, 2),
    (6, 1),
    (6, 2),
    (8, 1),
    (8, 2),
    (10, 1),
    (10, 2),
]


for lut_bits, iterations in configurations:

    max_error, avg_error = test_lut_configuration(
        lut_bits,
        iterations,
        test_values
    )

    entries = 1 << lut_bits

    print(
        f"{lut_bits:8d} "
        f"{entries:9d} "
        f"{iterations:12d} "
        f"{max_error: .10f} "
        f"{avg_error: .10f}"
    )


# ------------------------------------------------------------
# Demonstrate convergence
# ------------------------------------------------------------

print()
print("=== Example Newton-Raphson Convergence ===")
print()

examples = [
    1.13671875,
    2.0,
    5.0,
    10.0,
    11.0109863281
]

lut_bits = 6
lut = generate_lut(lut_bits)

for x in examples:

    exact = 1.0 / x

    seed = lookup_seed(
        x,
        lut,
        lut_bits
    )

    r1 = reciprocal_from_seed(
        x,
        seed,
        1
    )

    r2 = reciprocal_from_seed(
        x,
        seed,
        2
    )

    r1_q12 = to_float(
        to_fixed(r1)
    )

    r2_q12 = to_float(
        to_fixed(r2)
    )

    print(f"x = {x:.10f}")
    print(f"  Exact reciprocal : {exact:.10f}")
    print(f"  LUT seed         : {seed:.10f}")
    print(
        f"  After 1 NR       : {r1_q12:.10f} "
        f"error = {abs(r1_q12-exact):.10f}"
    )
    print(
        f"  After 2 NR       : {r2_q12:.10f} "
        f"error = {abs(r2_q12-exact):.10f}"
    )
    print()

