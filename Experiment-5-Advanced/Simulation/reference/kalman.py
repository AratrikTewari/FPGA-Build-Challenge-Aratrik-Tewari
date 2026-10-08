import numpy as np

F = np.array([
    [1.0, 0.0, 0.01, 0.0],
    [0.0, 1.0, 0.0, 0.01],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]
])

H = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0]
])

Q_base = np.array([
    [0.01, 0.0, 0.0, 0.0],
    [0.0, 0.01, 0.0, 0.0],
    [0.0, 0.0, 0.1, 0.0],
    [0.0, 0.0, 0.0, 0.1]
])

R = np.array([
    [1.0, 0.0],
    [0.0, 1.0]
])


class KalmanFilter:
    def __init__(self, initial_state):
        self.x = np.array(initial_state, dtype=float)
        self.P = np.array([
            [10.0, 0.0, 0.0, 0.0],
            [0.0, 10.0, 0.0, 0.0],
            [0.0, 0.0, 10.0, 0.0],
            [0.0, 0.0, 0.0, 10.0]
        ])
        # Gamma threshold for Chi-square distribution (2 Degrees of Freedom)
        # Represents ~99% confidence interval.
        self.gamma = 9.21 
        self.q_scale = 1.0

    def predict(self):
        self.x = F @ self.x
        # Adaptive process noise applied during prediction
        self.P = F @ self.P @ F.T + (Q_base * self.q_scale)

    def update(self, measurement):
        z = np.array(measurement, dtype=float)
        innovation = z - H @ self.x
        S = H @ self.P @ H.T + R

        # 1. Maneuver Detection (Normalized Innovation Squared)
        nis = innovation.T @ np.linalg.inv(S) @ innovation
        
        # If NIS exceeds threshold, target is maneuvering. Scale up Q.
        if nis > self.gamma:
            self.q_scale = min(nis / 2.0, 10.0) # Cap scaling to prevent instability
        else:
            self.q_scale = 1.0

        # 2. Standard Update
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        
        I = np.eye(4)
        self.P = (I - K @ H) @ self.P

    def step(self, measurement):
        self.predict()
        self.update(measurement)
        return self.x.copy()

    def project_risk_cone(self, steps_ahead=200):
        """
        Projects the current state and covariance forward to generate a risk cone.
        200 steps at 0.01s DT = 2.0 seconds horizon.
        """
        x_proj = self.x.copy()
        P_proj = self.P.copy()
        
        cone_states = []
        cone_covariances = []
        
        for _ in range(steps_ahead):
            x_proj = F @ x_proj
            P_proj = F @ P_proj @ F.T + Q_base
            cone_states.append(x_proj.copy())
            cone_covariances.append(P_proj.copy())
            
        return cone_states, cone_covariances