import math


class FixedPointReciprocal:
    """
    Bit-accurate fixed-point reciprocal reference model.

    Main hardware configuration:

        Input:
            20-bit signed Q8.12

        Internal reciprocal datapath:
            Normalized mantissa : Q2.27
            LUT seed            : Q2.27
            LUT entries         : 64
            NR iterations       : 1
            NR products         : 58-bit

        Output:
            16-bit signed Q0.12

    The class also supports other fractional-bit values for
    software precision sweeps.

    For a generic fractional_bits value, output_bits is chosen
    automatically so that values such as 1.0 remain representable.

    Architecture:

        input
          |
          v
        exponent detection
          |
          v
        normalization to [1, 2)
          |
          v
        6-bit LUT
          |
          v
        Newton-Raphson
          |
          v
        exponent denormalization
          |
          v
        output conversion
    """

    # ========================================================
    # Initialization
    # ========================================================

    def __init__(
        self,
        fractional_bits=12,
        internal_fractional_bits=27,
        lut_bits=6,
        output_bits=None
    ):

        self.fractional_bits = fractional_bits
        self.internal_fractional_bits = internal_fractional_bits
        self.lut_bits = lut_bits

        self.scale = 1 << fractional_bits
        self.internal_scale = 1 << internal_fractional_bits

        self.lut_size = 1 << lut_bits

        # ----------------------------------------------------
        # Output width
        #
        # Hardware:
        #     Q0.12 -> 16 bits
        #
        # Generic software sweep:
        #     ensure 1.0 is representable
        #
        # For Q16 this requires at least 17 signed bits.
        # ----------------------------------------------------

        if output_bits is None:

            self.output_bits = max(
                16,
                fractional_bits + 1
            )

        else:

            self.output_bits = output_bits

        self.lut = self._generate_lut()

    # ========================================================
    # Fixed-point conversion
    # ========================================================

    def to_fixed(self, value):

        return int(
            round(value * self.scale)
        )

    def to_float(self, value):

        return value / self.scale

    def to_internal(self, value):

        return int(
            round(value * self.internal_scale)
        )

    def internal_to_float(self, value):

        return value / self.internal_scale

    # ========================================================
    # Round-to-nearest, ties-to-even
    # ========================================================

    @staticmethod
    def round_shift(value, shift):

        # No shift required.

        if shift == 0:
            return value

        # Negative shift means left shift.

        if shift < 0:
            return value << (-shift)

        sign = -1 if value < 0 else 1
        magnitude = abs(value)

        quotient = magnitude >> shift

        remainder_mask = (
            (1 << shift) - 1
        )

        remainder = (
            magnitude & remainder_mask
        )

        half = (
            1 << (shift - 1)
        )

        # Greater than half -> round upward.

        if remainder > half:

            quotient += 1

        # Exactly half -> ties-to-even.

        elif remainder == half:

            if quotient & 1:
                quotient += 1

        return sign * quotient

    # ========================================================
    # Generate reciprocal LUT
    # ========================================================

    def _generate_lut(self):

        lut = []

        for address in range(self.lut_size):

            # ------------------------------------------------
            # Midpoint of LUT interval.
            #
            # normalized_x lies in [1, 2).
            # ------------------------------------------------

            normalized_x = (
                1.0
                + (address + 0.5)
                / self.lut_size
            )

            reciprocal = (
                1.0 / normalized_x
            )

            fixed_value = int(
                round(
                    reciprocal
                    * self.internal_scale
                )
            )

            lut.append(
                fixed_value
            )

        return lut

    # ========================================================
    # Determine exponent
    # ========================================================

    def _get_exponent(self, x_fixed):

        """
        x_fixed represents:

            x = x_fixed / 2^fractional_bits

        Find exponent e such that:

            x = m * 2^e

        with:

            1 <= m < 2
        """

        if x_fixed <= 0:

            raise ValueError(
                "Reciprocal input must be positive."
            )

        exponent = (
            x_fixed.bit_length()
            - 1
            - self.fractional_bits
        )

        return exponent

    # ========================================================
    # Normalize input
    # ========================================================

    def _normalize(self, x_fixed):

        """
        Convert x into:

            x = m * 2^e

        where:

            1 <= m < 2

        and m is represented in Q2.27.
        """

        exponent = self._get_exponent(
            x_fixed
        )

        # ----------------------------------------------------
        # x = x_fixed / 2^fractional_bits
        #
        # m = x / 2^exponent
        #
        # Therefore:
        #
        # m_fixed =
        #
        #     x_fixed * 2^27
        #     ----------------
        #     2^fractional_bits * 2^exponent
        # ----------------------------------------------------

        shift = (
            self.internal_fractional_bits
            - self.fractional_bits
            - exponent
        )

        normalized_fixed = (
            self.round_shift(
                x_fixed,
                -shift
            )
            if shift < 0
            else
            x_fixed << shift
        )

        # ----------------------------------------------------
        # Guard against a rounding boundary producing exactly
        # 2.0. In that case renormalize to 1.0 and increment
        # the exponent.
        # ----------------------------------------------------

        two_internal = (
            2 * self.internal_scale
        )

        if normalized_fixed >= two_internal:

            normalized_fixed = (
                self.round_shift(
                    normalized_fixed,
                    1
                )
            )

            exponent += 1

        return exponent, normalized_fixed

    # ========================================================
    # LUT lookup
    # ========================================================

    def _lookup_seed(self, normalized_fixed):

        """
        normalized_fixed is Q2.27 and represents:

            1 <= m < 2

        The fractional portion selects the LUT entry.
        """

        fractional_part = (
            normalized_fixed
            - self.internal_scale
        )

        if fractional_part < 0:
            fractional_part = 0

        address = (
            fractional_part
            * self.lut_size
        ) >> self.internal_fractional_bits

        # ----------------------------------------------------
        # Protect against boundary conditions.
        # ----------------------------------------------------

        if address < 0:
            address = 0

        if address >= self.lut_size:
            address = self.lut_size - 1

        return self.lut[address]

    # ========================================================
    # Internal multiplication
    # ========================================================

    def _multiply_internal(self, a, b):

        """
        Q2.27 × Q2.27

        Raw product:

            Q4.54

        Returned result:

            Q2.27

        Raw multiplication therefore requires:

            29 × 29 = 58 bits
        """

        product = (
            a * b
        )

        return self.round_shift(
            product,
            self.internal_fractional_bits
        )

    # ========================================================
    # Newton-Raphson
    # ========================================================

    def _newton_raphson(
        self,
        normalized_x,
        seed
    ):

        """
        Newton-Raphson reciprocal iteration:

            r_next = r * (2 - x*r)
        """

        # ----------------------------------------------------
        # x * r
        #
        # Q2.27 × Q2.27
        # -> Q2.27
        # ----------------------------------------------------

        xr = self._multiply_internal(
            normalized_x,
            seed
        )

        # ----------------------------------------------------
        # 2 - x*r
        # ----------------------------------------------------

        two_fixed = (
            2 * self.internal_scale
        )

        correction = (
            two_fixed
            - xr
        )

        # ----------------------------------------------------
        # r * correction
        # ----------------------------------------------------

        result = self._multiply_internal(
            seed,
            correction
        )

        return result

    # ========================================================
    # Denormalize reciprocal
    # ========================================================

    def _denormalize(
        self,
        reciprocal_normalized,
        exponent
    ):

        """
        If:

            x = m * 2^e

        then:

            1/x = (1/m) * 2^-e
        """

        if exponent >= 0:

            return self.round_shift(
                reciprocal_normalized,
                exponent
            )

        else:

            return (
                reciprocal_normalized
                << (-exponent)
            )

    # ========================================================
    # Convert internal format to output format
    # ========================================================

    def _convert_to_output(self, value):

        """
        Convert:

            Q2.27

        to:

            Q0.fractional_bits

        using the configured output width.
        """

        shift = (
            self.internal_fractional_bits
            - self.fractional_bits
        )

        result = self.round_shift(
            value,
            shift
        )

        # ----------------------------------------------------
        # Signed output limits.
        # ----------------------------------------------------

        max_value = (
            (1 << (self.output_bits - 1))
            - 1
        )

        min_value = (
            -(1 << (self.output_bits - 1))
        )

        # ----------------------------------------------------
        # Saturation.
        # ----------------------------------------------------

        if result > max_value:

            result = max_value

        elif result < min_value:

            result = min_value

        return result

    # ========================================================
    # Main reciprocal
    # ========================================================

    def reciprocal(self, x_fixed):

        """
        Compute reciprocal of a fixed-point input.

        Input representation:

            x = x_fixed / 2^fractional_bits

        Output representation:

            result = reciprocal * 2^fractional_bits
        """

        if x_fixed <= 0:

            raise ValueError(
                "Reciprocal input must be positive."
            )

        # ----------------------------------------------------
        # Normalize:
        #
        # x = m * 2^e
        #
        # 1 <= m < 2
        # ----------------------------------------------------

        exponent, normalized_x = (
            self._normalize(
                x_fixed
            )
        )

        # ----------------------------------------------------
        # LUT seed
        # ----------------------------------------------------

        seed = self._lookup_seed(
            normalized_x
        )

        # ----------------------------------------------------
        # One Newton-Raphson iteration
        # ----------------------------------------------------

        reciprocal_normalized = (
            self._newton_raphson(
                normalized_x,
                seed
            )
        )

        # ----------------------------------------------------
        # Undo normalization:
        #
        # 1/x = (1/m) * 2^-e
        # ----------------------------------------------------

        reciprocal_internal = (
            self._denormalize(
                reciprocal_normalized,
                exponent
            )
        )

        # ----------------------------------------------------
        # Convert Q2.27 to output format.
        # ----------------------------------------------------

        result_fixed = (
            self._convert_to_output(
                reciprocal_internal
            )
        )

        return result_fixed

    # ========================================================
    # Floating-point convenience interface
    # ========================================================

    def reciprocal_float(self, x):

        x_fixed = self.to_fixed(
            x
        )

        result_fixed = self.reciprocal(
            x_fixed
        )

        return self.to_float(
            result_fixed
        )
