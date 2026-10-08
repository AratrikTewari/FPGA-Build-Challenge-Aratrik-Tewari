import numpy as np


# ============================================================
# Fixed-point configuration
# ============================================================

FRACTIONAL_BITS = 12
SCALE = 1 << FRACTIONAL_BITS

PRODUCT_BITS = 40
ACCUMULATOR_BITS = 44


# ============================================================
# Fixed-point conversion
# ============================================================

def to_fixed(value):
    return int(np.round(value * SCALE))


def from_fixed(value):
    return value / SCALE


# ============================================================
# Signed integer limits
# ============================================================

def signed_limits(bits):
    minimum = -(1 << (bits - 1))
    maximum = (1 << (bits - 1)) - 1
    return minimum, maximum


PRODUCT_MIN, PRODUCT_MAX = signed_limits(PRODUCT_BITS)
ACC_MIN, ACC_MAX = signed_limits(ACCUMULATOR_BITS)


# ============================================================
# Range tracker
# ============================================================

ranges = {}


def track(name, value):

    value = int(value)

    if name not in ranges:
        ranges[name] = {
            "min": value,
            "max": value,
            "max_abs": abs(value)
        }

    else:
        ranges[name]["min"] = min(
            ranges[name]["min"],
            value
        )

        ranges[name]["max"] = max(
            ranges[name]["max"],
            value
        )

        ranges[name]["max_abs"] = max(
            ranges[name]["max_abs"],
            abs(value)
        )


# ============================================================
# Fixed-point multiplication
#
# Q8.12 × Q8.12
#
# Raw product:
#       20 bits × 20 bits = 40 bits
#
# Product contains 24 fractional bits.
# ============================================================

def fixed_multiply(a, b):

    product = a * b

    track("raw product", product)

    return product


# ============================================================
# Fixed-point MAC
#
# The inputs are fixed-point integers.
#
# We accumulate raw products BEFORE shifting.
# ============================================================

def fixed_mac(vector_a, vector_b):

    accumulator = 0

    for a, b in zip(vector_a, vector_b):

        product = fixed_multiply(a, b)

        accumulator += product

        track("MAC accumulator", accumulator)

    return accumulator


# ============================================================
# Generate deterministic test vectors
#
# These include the actual ranges observed in the Kalman
# simulation, plus boundary-oriented values.
# ============================================================

values = [

    -90.120647,
    -79.390892,
    -72.180748,
    -19.653635,
    -10.512581,
    -6.588309,

    -5.772158,
    -1.0,
    -0.1,
    0.0,

    0.1,
    1.0,
    5.0,
    6.588309,
    10.512581,
    11.011000,

    72.180748,
    79.390892,
    90.120647,
    92.588410
]


fixed_values = [
    to_fixed(value)
    for value in values
]


# ============================================================
# Test all pairwise multiplications
# ============================================================

for a in fixed_values:

    for b in fixed_values:

        fixed_multiply(a, b)


# ============================================================
# Test MAC operations representative of matrix operations
# ============================================================

for i in range(len(fixed_values) - 3):

    vector_a = fixed_values[i:i + 4]
    vector_b = fixed_values[-(i + 4):]

    fixed_mac(vector_a, vector_b)


# ============================================================
# Test important Kalman matrix dimensions
#
# 4-term dot products:
#       4 × 4 matrix multiplication
#
# 2-term dot products:
#       measurement / innovation paths
# ============================================================

for i in range(0, len(fixed_values) - 7, 4):

    vector_a = fixed_values[i:i + 4]
    vector_b = fixed_values[i + 4:i + 8]

    fixed_mac(vector_a, vector_b)


# ============================================================
# Print results
# ============================================================

print()
print("=== Raw Fixed-Point MAC Width Verification ===")
print()

print(
    f"{'Quantity':<22}"
    f"{'Min':>18}"
    f"{'Max':>18}"
    f"{'Max |value|':>20}"
)

print("-" * 80)


for name, data in ranges.items():

    print(
        f"{name:<22}"
        f"{data['min']:>18}"
        f"{data['max']:>18}"
        f"{data['max_abs']:>20}"
    )


# ============================================================
# Convert important ranges back to real values
# ============================================================

print()
print("=== Real-Value Interpretation ===")
print()

for name, data in ranges.items():

    if name == "raw product":

        # Raw product has 24 fractional bits
        scale = 1 << (2 * FRACTIONAL_BITS)

    else:

        # MAC accumulator also contains 24 fractional bits
        scale = 1 << (2 * FRACTIONAL_BITS)

    minimum = data["min"] / scale
    maximum = data["max"] / scale
    maximum_abs = data["max_abs"] / scale

    print(
        f"{name:<22}"
        f"min = {minimum:>14.6f}   "
        f"max = {maximum:>14.6f}   "
        f"max |value| = {maximum_abs:>14.6f}"
    )


# ============================================================
# Hardware limits
# ============================================================

print()
print("=== Hardware Limits ===")
print()

print(
    f"20-bit signed input range: "
    f"{signed_limits(20)[0]} to {signed_limits(20)[1]}"
)

print(
    f"40-bit signed product range: "
    f"{PRODUCT_MIN} to {PRODUCT_MAX}"
)

print(
    f"44-bit signed accumulator range: "
    f"{ACC_MIN} to {ACC_MAX}"
)


# ============================================================
# Check whether observed values fit
# ============================================================

product_required = ranges["raw product"]["max_abs"]
accumulator_required = ranges["MAC accumulator"]["max_abs"]


print()
print("=== Width Checks ===")
print()

if product_required <= PRODUCT_MAX:
    print("40-bit product: PASS")
else:
    print("40-bit product: FAIL")


if accumulator_required <= ACC_MAX:
    print("44-bit accumulator: PASS")
else:
    print("44-bit accumulator: FAIL")


# ============================================================
# Calculate minimum required signed widths
# ============================================================

def required_signed_bits(max_abs):

    bits = 1

    while max_abs > (1 << (bits - 1)) - 1:
        bits += 1

    return bits


product_required_bits = required_signed_bits(product_required)
accumulator_required_bits = required_signed_bits(accumulator_required)


print()
print("=== Minimum Required Width ===")
print()

print(
    f"Raw product requires at least "
    f"{product_required_bits} signed bits"
)

print(
    f"MAC accumulator requires at least "
    f"{accumulator_required_bits} signed bits"
)


print()
print("Verification complete.")
