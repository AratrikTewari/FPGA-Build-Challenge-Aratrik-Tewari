from reference.reciprocal import FixedPointReciprocal


reciprocal = FixedPointReciprocal(
    fractional_bits=12
)


test_values = [
    1.13671875,
    2.0,
    3.0,
    5.0,
    7.5,
    10.0,
    11.0109863281
]


print()
print("=== Q12 Reciprocal Reference Test ===")
print()

print(
    "Input          Exact          Approximation"
    "       Error"
)

print("-" * 65)


max_error = 0.0


for x in test_values:

    exact = 1.0 / x

    approximation = (
        reciprocal.reciprocal_float(x)
    )

    error = abs(
        approximation - exact
    )

    max_error = max(
        max_error,
        error
    )

    print(
        f"{x:10.6f}   "
        f"{exact:12.8f}   "
        f"{approximation:12.8f}   "
        f"{error:.10f}"
    )


print()
print(
    f"Maximum error: {max_error:.10f}"
)

print(
    f"Q12 LSB:       {1/4096:.10f}"
)

