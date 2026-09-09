import math


FRACTIONAL_BITS = 8
SCALE = 2 ** FRACTIONAL_BITS


def float_to_fixed(value):
    return round(value * SCALE)


def fixed_to_float(value):
    return value / SCALE


def quantization_error(value):
    fixed_value = float_to_fixed(value)
    reconstructed = fixed_to_float(fixed_value)

    return abs(value - reconstructed)


test_values = [
    0.0,
    1.0,
    -1.0,
    10.0,
    10.125,
    30.0,
    30.1234,
    -5.7122,
    6.4033,
    -6.5883,
    6.5295,
    80.3484,
    90.1015,
]


print("=== Q8.8 Fixed-Point Test ===")
print()

print(f"Fractional bits : {FRACTIONAL_BITS}")
print(f"Scale           : {SCALE}")
print(f"Resolution      : {1 / SCALE}")
print()

max_error = 0.0

for value in test_values:

    fixed_value = float_to_fixed(value)
    reconstructed = fixed_to_float(fixed_value)
    error = quantization_error(value)

    max_error = max(max_error, error)

    print(
        f"Float: {value:10.6f}  "
        f"Fixed: {fixed_value:6d}  "
        f"Back: {reconstructed:10.6f}  "
        f"Error: {error:.8f}"
    )


print()
print(f"Maximum quantization error: {max_error:.8f}")
print(f"Maximum theoretical error: {1 / (2 * SCALE):.8f}")

