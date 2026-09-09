import numpy as np

from .reciprocal import FixedPointReciprocal


class FixedPointKalmanFilter:

    def __init__(
        self,
        initial_state,
        fractional_bits=12
    ):
        self.fractional_bits = fractional_bits
        self.scale = 1 << fractional_bits

        # ----------------------------------------------------
        # State
        # x = [x, y, vx, vy]
        # ----------------------------------------------------

        self.x = self.to_fixed_array(initial_state)

        # ----------------------------------------------------
        # System matrices
        # ----------------------------------------------------

        dt = 0.01

        self.F = self.to_fixed_matrix([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        self.H = self.to_fixed_matrix([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])

        # Store Q_base for adaptive scaling
        self.Q_base = self.to_fixed_matrix([
            [0.01, 0.0, 0.0, 0.0],
            [0.0, 0.01, 0.0, 0.0],
            [0.0, 0.0, 0.1, 0.0],
            [0.0, 0.0, 0.0, 0.1]
        ])
        self.Q = np.copy(self.Q_base)

        self.R = self.to_fixed_matrix([
            [1.0, 0.0],
            [0.0, 1.0]
        ])

        # ----------------------------------------------------
        # Initial covariance
        # ----------------------------------------------------

        self.P = self.to_fixed_matrix([
            [10.0, 0.0, 0.0, 0.0],
            [0.0, 10.0, 0.0, 0.0],
            [0.0, 0.0, 10.0, 0.0],
            [0.0, 0.0, 0.0, 10.0]
        ])

        # ----------------------------------------------------
        # Reciprocal & Adaptation Config
        # ----------------------------------------------------

        self.reciprocal = FixedPointReciprocal(
            fractional_bits=fractional_bits
        )
        
        # Gamma 9.21 in Q8.12
        self.gamma_fixed = self.to_fixed(9.21)

    # ========================================================
    # Fixed-point conversion
    # ========================================================

    def to_fixed(self, value):
        return int(round(value * self.scale))

    def to_float(self, value):
        return value / self.scale

    def to_fixed_array(self, values):
        return np.array(
            [self.to_fixed(v) for v in values],
            dtype=np.int64
        )

    def to_fixed_matrix(self, matrix):
        return np.array(
            [
                [self.to_fixed(v) for v in row]
                for row in matrix
            ],
            dtype=np.int64
        )

    # ========================================================
    # Fixed-point multiplication
    # ========================================================

    def multiply(self, a, b):
        return (a * b) >> self.fractional_bits

    # ========================================================
    # Matrix multiplication
    # ========================================================

    def matmul(self, A, B):

        rows = A.shape[0]
        cols = B.shape[1]
        inner = A.shape[1]

        result = np.zeros(
            (rows, cols),
            dtype=np.int64
        )

        for i in range(rows):
            for j in range(cols):

                accumulator = 0

                for k in range(inner):
                    accumulator += (
                        int(A[i, k]) * int(B[k, j])
                    )

                result[i, j] = (
                    accumulator >> self.fractional_bits
                )

        return result

    # ========================================================
    # Prediction
    # ========================================================

    def predict(self):

        # x_pred = F x
        self.x = self.matmul(
            self.F,
            self.x.reshape(4, 1)
        ).reshape(4)

        # P_pred = F P F^T + Q
        FP = self.matmul(self.F, self.P)
        FPFt = self.matmul(FP, self.F.T)

        self.P = FPFt + self.Q

        return self.x

    # ========================================================
    # Measurement update
    # ========================================================

    def update(self, measurement):

        z = self.to_fixed_array(
            measurement
        ).reshape(2, 1)

        Hx = self.matmul(
            self.H,
            self.x.reshape(4, 1)
        )

        innovation = z - Hx

        HP = self.matmul(self.H, self.P)
        HPHt = self.matmul(HP, self.H.T)

        S = HPHt + self.R

        S_value = int(S[0, 0])
        invS = self.reciprocal.reciprocal(S_value)

        # ----------------------------------------------------
        # Adaptive Process Noise (NIS Check)
        # NIS = (y_x^2 + y_y^2) * invS
        # ----------------------------------------------------
        inn_x = int(innovation[0, 0])
        inn_y = int(innovation[1, 0])
        
        # Calculate y^2
        y2_x = self.multiply(inn_x, inn_x)
        y2_y = self.multiply(inn_y, inn_y)
        
        # Calculate NIS (Note: invS is Q0.12)
        nis_fixed = self.multiply((y2_x + y2_y), invS)
        
        if nis_fixed > self.gamma_fixed:
            # Scale Q_base by 4x if maneuver detected
            self.Q = self.Q_base * 4 
        else:
            self.Q = np.copy(self.Q_base)

        # ----------------------------------------------------
        # Standard Update Continues
        # ----------------------------------------------------
        PHt = self.matmul(self.P, self.H.T)

        K = np.zeros((4, 2), dtype=np.int64)
        for i in range(4):
            for j in range(2):
                K[i, j] = self.multiply(int(PHt[i, j]), int(invS))

        Ky = self.matmul(K, innovation)
        self.x = self.x + Ky.reshape(4)

        I = np.eye(4, dtype=np.int64) * self.scale
        KH = self.matmul(K, self.H)
        I_minus_KH = I - KH

        self.P = self.matmul(I_minus_KH, self.P)

        return self.x

    # ========================================================
    # Complete Kalman step
    # ========================================================

    def step(self, measurement):
        self.predict()
        self.update(measurement)
        return self.get_state_float()

    # ========================================================
    # Risk Cone Projection (Fixed-Point)
    # ========================================================
    
    def project_risk_cone(self, steps_ahead=200):
        """
        Projects fixed-point state and covariance forward.
        """
        x_proj = np.copy(self.x)
        P_proj = np.copy(self.P)
        
        cone_states = []
        cone_covariances = []
        
        for _ in range(steps_ahead):
            x_proj = self.matmul(self.F, x_proj.reshape(4, 1)).reshape(4)
            FP = self.matmul(self.F, P_proj)
            P_proj = self.matmul(FP, self.F.T) + self.Q_base
            
            # Convert back to float for the Python UI to process
            float_x = [self.to_float(int(v)) for v in x_proj]
            float_P = [[self.to_float(int(v)) for v in row] for row in P_proj]
            
            cone_states.append(float_x)
            cone_covariances.append(float_P)
            
        return cone_states, cone_covariances

    # ========================================================
    # Floating-point outputs
    # ========================================================

    def get_state_float(self):
        return np.array(
            [self.to_float(int(v)) for v in self.x],
            dtype=np.float64
        )

    def get_covariance_float(self):
        return np.array(
            [[self.to_float(int(v)) for v in row] for row in self.P],
            dtype=np.float64
        )