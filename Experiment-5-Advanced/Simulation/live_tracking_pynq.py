"""
Live Multi-Target Sensor + Kalman Tracking + PYNQ-Z2 Link
============================================================

This is your existing "LIVE MULTI-TARGET SENSOR + KALMAN TRACKING" demo,
restructured around the three jobs the write-up asked this Python model
to do:

  TASK 1 -- Act as a replacement virtual environment for physical sensors.
      -> `VirtualSensorEnvironment`. It owns the true target motion and
         hands out noisy measurement PACKETS with a stable schema
         (seq, timestamp, measurements, ground_truth). Nothing downstream
         cares whether those numbers came from this simulator or a real
         sensor rig -- swap this class out later and everything else
         keeps working.

  TASK 2 -- Compute the same data the FPGA will compute (so you can show
            software speed / correctness before hardware exists).
      -> `SoftwarePipeline` (plain float Kalman + collision/TTC -- the
         "CPU" side of the Stage 3 comparison in your write-up) and
         `HardwareLink` in "MODEL" mode (bit-accurate fixed-point Kalman
         + the same collision/TTC -- a software stand-in for what the
         Verilog core on the PL fabric will produce). Both run on every
         reading, both are timed, and both are shown side by side.

  TASK 3 -- Transfer the data to the PYNQ-Z2.
      -> `HardwareLink` in "PYNQ" mode opens a TCP socket to the board
         and sends each measurement packet as one JSON line, receiving
         back the tracked [x,y,vx,vy] states and the latency the board
         measured. A matching reference stub server
         (`run_pynq_stub_server`) is included so you can test the wire
         protocol end-to-end today, on a laptop or the actual board,
         even before the real AXI-Stream/DMA + Verilog core (Stage 4/5)
         exists -- point mode at "PYNQ" and it just needs that stub
         (or eventually the real overlay driver) listening on the other
         end.

Run the demo (default, no arguments needed):
    python live_tracking_pynq.py

Run the reference stub server instead (e.g. on the PYNQ-Z2 itself, or on
a second terminal on the same machine for testing):
    python live_tracking_pynq.py --server
    python live_tracking_pynq.py --server --port 6000
"""

import itertools
import json
import math
import random
import socket
import sys
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button, TextBox


# ============================================================
# SETTINGS
# ============================================================

MIN_TARGETS = 4
MAX_TARGETS = 8

ARENA_SIZE = 100.0

ANIMATION_DT = 0.03

MIN_SPEED = 2.0
MAX_SPEED = 6.0

# Motion is now "quite straight, with slight deviation": each target
# keeps a fixed base heading and speed, and wanders a little around that
# base heading using a mean-reverting (Ornstein-Uhlenbeck-style) random
# walk instead of a plain unbounded one -- a plain random walk in angle
# eventually accumulates into loops and reversals no matter how small
# the per-frame noise is, which doesn't look "straight" over a long demo.
# Mean-reversion keeps the wander bounded around the original direction
# indefinitely. HEADING_NOISE_STD is the per-frame innovation size (rad);
# HEADING_REVERSION_RATE is how strongly it's pulled back to the base
# heading each frame. Smaller noise / larger reversion = straighter.
HEADING_NOISE_STD = 0.05
HEADING_REVERSION_RATE = 0.03

DEFAULT_NOISE = 1.0
DEFAULT_FRACTIONAL_BITS = 12

COLLISION_DISTANCE = 2.0
WARNING_DISTANCE = 5.0
TTC_THRESHOLD = 2.0
PREDICTION_HORIZON = 5.0

# Predicted-trajectory dotted line: drawn from each target's current
# tracked position, in the direction of its last movement, extended to
# TRAJECTORY_LENGTH_MULTIPLIER times the length of that last movement
# segment. Two targets' predicted lines "interfering" (crossing, or
# passing within TRAJECTORY_ALERT_DISTANCE of each other) raises an alert.
TRAJECTORY_LENGTH_MULTIPLIER = 5
TRAJECTORY_ALERT_DISTANCE = 3.0

DEFAULT_PYNQ_HOST = "192.168.2.99"   # PYNQ-Z2's default static IP over USB
DEFAULT_PYNQ_PORT = 6000
SOCKET_TIMEOUT_S = 0.75


# ============================================================
# KALMAN FILTERS
#   - KalmanFilter          : float, golden reference ("software/CPU" side)
#   - FixedPointKalmanFilter: quantized, bit-accurate model of the Verilog
#                             core ("FPGA model" side, used until real
#                             hardware is attached)
# ============================================================

class KalmanFilter:
    """Floating-point Kalman filter -- unchanged from the original script."""

    def __init__(self, initial_state, noise):
        self.x = np.array(initial_state, dtype=float)
        self.P = np.eye(4) * 10.0
        self.H = np.array([[1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0]])
        self.R = np.array([[noise * noise, 0.0],
                            [0.0, noise * noise]])
        self.Q = np.array([
            [0.01, 0.00, 0.00, 0.00],
            [0.00, 0.01, 0.00, 0.00],
            [0.00, 0.00, 0.10, 0.00],
            [0.00, 0.00, 0.00, 0.10],
        ])

    def get_F(self, dt):
        return np.array([[1.0, 0.0, dt, 0.0],
                          [0.0, 1.0, 0.0, dt],
                          [0.0, 0.0, 1.0, 0.0],
                          [0.0, 0.0, 0.0, 1.0]])

    def step(self, measurement, dt):
        F = self.get_F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

        z = np.array(measurement, dtype=float)
        error = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ error
        self.P = (np.eye(4) - K @ self.H) @ self.P

        return self.x.copy()


class FixedPointKalmanFilter:
    """
    Quantized model of the FPGA's fixed-point Kalman core. Every quantity
    is rounded to a Q(int).(fractional_bits) grid after each step, the
    same way the hardware's finite word length would round it.

    Stage-3 precision validation on this project found fractional_bits=12
    (Q9.12, 21-bit signed words) tracks the float filter to well under a
    millimetre of RMSE -- see compare_fixed_float.py / verify_s_structure.py
    if you have them, or STAGE3_REPORT.md for the numbers. That's why 12
    is the default here.
    """

    def __init__(self, initial_state, noise, fractional_bits=DEFAULT_FRACTIONAL_BITS):
        self.fractional_bits = fractional_bits
        self.scale = 2 ** fractional_bits

        self._H_float = np.array([[1.0, 0.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.0, 0.0]])
        self._Q_base_float = np.array([
            [0.01, 0.00, 0.00, 0.00],
            [0.00, 0.01, 0.00, 0.00],
            [0.00, 0.00, 0.10, 0.00],
            [0.00, 0.00, 0.00, 0.10],
        ])
        R_float = np.array([[noise * noise, 0.0], [0.0, noise * noise]])

        self.H = self._quantize(self._H_float)
        self.R = self._quantize(R_float)
        self.x = self._quantize(np.array(initial_state, dtype=float))
        self.P = self._quantize(np.eye(4) * 10.0)

    def _quantize(self, arr):
        return np.round(np.asarray(arr, dtype=float) * self.scale).astype(np.int64)

    def _to_float(self, fixed_arr):
        return np.asarray(fixed_arr, dtype=float) / self.scale

    def step(self, measurement, dt):
        F_float = np.array([[1.0, 0.0, dt, 0.0],
                             [0.0, 1.0, 0.0, dt],
                             [0.0, 0.0, 1.0, 0.0],
                             [0.0, 0.0, 0.0, 1.0]])
        Q_float = self._Q_base_float

        F = self._quantize(F_float)
        x, P = self._to_float(self.x), self._to_float(self.P)
        F_f, Q_f = self._to_float(F), self._to_float(self._quantize(Q_float))

        x_pred = F_f @ x
        P_pred = F_f @ P @ F_f.T + Q_f
        self.x, self.P = self._quantize(x_pred), self._quantize(P_pred)

        H, R = self._to_float(self.H), self._to_float(self.R)
        x, P = self._to_float(self.x), self._to_float(self.P)
        z = np.array(measurement, dtype=float)
        innovation = z - H @ x
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x_new = x + K @ innovation
        P_new = (np.eye(4) - K @ H) @ P
        self.x, self.P = self._quantize(x_new), self._quantize(P_new)

        return self._to_float(self.x).tolist()


# ============================================================
# TASK 1: VIRTUAL SENSOR ENVIRONMENT
#   (drop-in replacement for a physical sensor rig)
# ============================================================

def _reflect_heading_x(heading):
    """Mirror a heading off a vertical wall (negates the x-velocity
    component, leaves y-velocity alone)."""
    return math.pi - heading


def _reflect_heading_y(heading):
    """Mirror a heading off a horizontal wall (negates the y-velocity
    component, leaves x-velocity alone)."""
    return -heading


class Target:
    """
    Moves in a near-straight line at constant speed: `base_heading` is
    fixed (until a wall bounce), and `heading_offset` wanders around 0
    with a mean-reverting random walk -- a small, bounded wobble around
    the base direction that never grows into loops or reversals, no
    matter how long the simulation runs.
    """

    def __init__(self, target_id, x, y, vx, vy):
        self.id = target_id
        self.x, self.y = x, y
        self.speed = math.hypot(vx, vy)
        self.base_heading = math.atan2(vy, vx)
        self.heading_offset = 0.0

    @property
    def heading(self):
        return self.base_heading + self.heading_offset

    @property
    def vx(self):
        return self.speed * math.cos(self.heading)

    @property
    def vy(self):
        return self.speed * math.sin(self.heading)

    def move(self, dt, speed_multiplier):
        # Slight deviation: a small mean-reverting nudge around the base
        # heading every frame -> gently wandering, but still overall
        # straight, paths.
        self.heading_offset += (
            random.gauss(0.0, HEADING_NOISE_STD)
            - HEADING_REVERSION_RATE * self.heading_offset
        )

        self.x += self.vx * dt * speed_multiplier
        self.y += self.vy * dt * speed_multiplier

        if self.x <= 0:
            self.x = 0.0
            self.base_heading = _reflect_heading_x(self.base_heading)
            self.heading_offset = 0.0
        elif self.x >= ARENA_SIZE:
            self.x = ARENA_SIZE
            self.base_heading = _reflect_heading_x(self.base_heading)
            self.heading_offset = 0.0
        if self.y <= 0:
            self.y = 0.0
            self.base_heading = _reflect_heading_y(self.base_heading)
            self.heading_offset = 0.0
        elif self.y >= ARENA_SIZE:
            self.y = ARENA_SIZE
            self.base_heading = _reflect_heading_y(self.base_heading)
            self.heading_offset = 0.0

    def get_state(self):
        return [self.x, self.y, self.vx, self.vy]


def create_targets(number):
    targets = []
    for i in range(number):
        x = random.uniform(10, 90)
        y = random.uniform(10, 90)
        angle = random.uniform(0, 2 * math.pi)
        speed = random.uniform(MIN_SPEED, MAX_SPEED)
        targets.append(Target(i, x, y, speed * math.cos(angle), speed * math.sin(angle)))
    return targets


class VirtualSensorEnvironment:
    """
    TASK 1. Stands in for a physical sensor rig: owns ground truth motion,
    and on request produces one measurement PACKET -- a fixed schema that
    a real sensor front-end (Stage 6, if you build one) would also need
    to produce. Everything downstream (SoftwarePipeline, HardwareLink)
    only ever sees packets, never touches Target objects directly, so
    swapping this class for a real sensor reader later is a one-file
    change.
    """

    def __init__(self, num_targets, noise_std):
        self.targets = create_targets(num_targets)
        self.noise_std = noise_std
        self.seq = 0

    def set_noise(self, noise_std):
        self.noise_std = noise_std

    def advance_motion(self, dt, speed_multiplier):
        for t in self.targets:
            t.move(dt, speed_multiplier)

    def read_packet(self):
        """Returns a dict: the sensor packet. This is the one function a
        real sensor front-end would need to reimplement."""
        self.seq += 1
        measurements = []
        ground_truth = []
        for t in self.targets:
            mx = t.x + random.gauss(0, self.noise_std)
            my = t.y + random.gauss(0, self.noise_std)
            measurements.append([mx, my])
            ground_truth.append(t.get_state())
        return {
            "seq": self.seq,
            "timestamp": time.time(),
            "measurements": measurements,
            "ground_truth": ground_truth,   # kept for the demo's TRUE dots;
                                             # a real sensor obviously
                                             # wouldn't be able to supply this
        }


# ============================================================
# COLLISION / TTC MODEL
#   (shared by both the software pipeline and the hardware-model pipeline
#   -- this is the same math either side needs to run downstream of
#   tracking, so both "compute the same data")
# ============================================================

def predict_position(state, dt_future):
    x, y, vx, vy = state
    return [x + vx * dt_future, y + vy * dt_future]


def time_of_closest_approach(state_a, state_b):
    dx = state_a[0] - state_b[0]
    dy = state_a[1] - state_b[1]
    dvx = state_a[2] - state_b[2]
    dvy = state_a[3] - state_b[3]
    rel_speed_sq = dvx * dvx + dvy * dvy
    if rel_speed_sq < 1e-9:
        return 0.0
    t = -(dx * dvx + dy * dvy) / rel_speed_sq
    return max(0.0, t)


def closest_approach(state_a, state_b, horizon=PREDICTION_HORIZON):
    t = min(time_of_closest_approach(state_a, state_b), horizon)
    pa = predict_position(state_a, t)
    pb = predict_position(state_b, t)
    return math.hypot(pa[0] - pb[0], pa[1] - pb[1]), t


def classify(min_distance, ttc):
    collision = (min_distance <= COLLISION_DISTANCE) and (ttc <= TTC_THRESHOLD)
    if collision:
        risk = "HIGH"
    elif min_distance <= WARNING_DISTANCE:
        risk = "MEDIUM"
    else:
        risk = "LOW"
    return collision, risk


def analyze_all_pairs(tracked_states):
    results = []
    n = len(tracked_states)
    for i, j in itertools.combinations(range(n), 2):
        min_dist, ttc = closest_approach(tracked_states[i], tracked_states[j])
        collision, risk = classify(min_dist, ttc)
        results.append({
            "pair": (i, j), "closest_distance": min_dist,
            "ttc": ttc, "risk": risk, "collision": collision,
        })
    risk_order = {"HIGH": 0, "MEDIUM": 1, "LOW": 2}
    results.sort(key=lambda r: (risk_order[r["risk"]], r["closest_distance"]))
    return results


# ============================================================
# PREDICTED-TRAJECTORY LINES + INTERFERENCE DETECTION
#
# Separate from the Kalman-velocity-based collision/TTC model above:
# this draws a simple dotted extrapolation of each target's last
# observed movement (previous tracked point -> current tracked point,
# extended TRAJECTORY_LENGTH_MULTIPLIER times) and flags any pair of
# those dotted lines that cross or pass close to each other.
# ============================================================

def trajectory_segment(curr_point, vx, vy, dt, multiplier=TRAJECTORY_LENGTH_MULTIPLIER):
    """
    Given the current tracked position and the Kalman filter's own
    velocity estimate (vx, vy), returns (curr_point, end_point) for the
    dotted trajectory line: a straight projection forward from
    curr_point, along the tracked velocity vector, `multiplier`
    sensor-intervals long. Returns None if the velocity is ~zero.

    This used to be computed from curr_point - prev_point (the
    difference between two consecutive tracked *positions*). That
    looked fine on paper but was jumpy in practice: each tracked
    position still carries some leftover measurement noise, and when a
    target only moves a little between two readings (slow target, high
    sensor rate, or a noisy sensor), that leftover noise is comparable
    in size to the real movement -- so the difference vector's
    direction is dominated by noise and can swing anywhere from one
    reading to the next. The velocity estimate, by contrast, is a
    Kalman state that the constant-velocity motion model constrains to
    change gradually, so it points in a continuous, physically
    plausible direction the way a real moving object would.
    """
    step_dx = vx * dt
    step_dy = vy * dt
    if step_dx == 0.0 and step_dy == 0.0:
        return None
    end_point = (curr_point[0] + step_dx * multiplier, curr_point[1] + step_dy * multiplier)
    return curr_point, end_point


def _orientation(p, q, r):
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if abs(val) < 1e-9:
        return 0
    return 1 if val > 0 else 2


def _on_segment(p, q, r):
    return (min(p[0], r[0]) - 1e-9 <= q[0] <= max(p[0], r[0]) + 1e-9 and
            min(p[1], r[1]) - 1e-9 <= q[1] <= max(p[1], r[1]) + 1e-9)


def segments_intersect(p1, p2, p3, p4):
    """Standard orientation-based segment intersection test."""
    o1, o2 = _orientation(p1, p2, p3), _orientation(p1, p2, p4)
    o3, o4 = _orientation(p3, p4, p1), _orientation(p3, p4, p2)

    if o1 != o2 and o3 != o4:
        return True
    if o1 == 0 and _on_segment(p1, p3, p2):
        return True
    if o2 == 0 and _on_segment(p1, p4, p2):
        return True
    if o3 == 0 and _on_segment(p3, p1, p4):
        return True
    if o4 == 0 and _on_segment(p3, p2, p4):
        return True
    return False


def _point_segment_distance(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def segment_distance(p1, p2, p3, p4):
    """Minimum distance between segment p1-p2 and segment p3-p4 (0.0 if
    they intersect)."""
    if segments_intersect(p1, p2, p3, p4):
        return 0.0
    return min(
        _point_segment_distance(p1, p3, p4),
        _point_segment_distance(p2, p3, p4),
        _point_segment_distance(p3, p1, p2),
        _point_segment_distance(p4, p1, p2),
    )


def find_trajectory_alerts(trajectories, alert_distance=TRAJECTORY_ALERT_DISTANCE):
    """
    trajectories: dict {target_index: (start_point, end_point)} for
    targets that currently have a predicted trajectory line.

    Returns a list of dicts, one per pair whose lines cross or come
    within alert_distance of each other, sorted by how close they get.
    """
    alerts = []
    indices = list(trajectories.keys())
    for a, b in itertools.combinations(indices, 2):
        p1, p2 = trajectories[a]
        p3, p4 = trajectories[b]
        dist = segment_distance(p1, p2, p3, p4)
        if dist <= alert_distance:
            alerts.append({"pair": (a, b), "distance": dist})
    alerts.sort(key=lambda r: r["distance"])
    return alerts


# ============================================================
# TASK 2: SOFTWARE PIPELINE
#   (the "CPU / Python" side of the Stage-3 comparison in your write-up)
# ============================================================

class SoftwarePipeline:
    """Plain float Kalman + collision/TTC, timed. This is the number that
    goes in the 'Software latency' row of the comparison panel."""

    def __init__(self, initial_measurements, noise_std):
        self.filters = [
            KalmanFilter([m[0], m[1], 0.0, 0.0], noise_std)
            for m in initial_measurements
        ]

    def process(self, measurements, dt):
        start = time.perf_counter()
        tracked = [f.step(m, dt) for f, m in zip(self.filters, measurements)]
        tracked = [list(map(float, t)) for t in tracked]
        pairs = analyze_all_pairs(tracked)
        latency_us = (time.perf_counter() - start) * 1e6
        return tracked, pairs, latency_us


# ============================================================
# TASK 3: HARDWARE LINK
#   MODEL mode -> bit-accurate fixed-point model, run on the host CPU,
#                 standing in for the FPGA until it's attached.
#   PYNQ mode  -> actually opens a socket and sends packets to a PYNQ-Z2
#                 (or anything speaking the same tiny JSON-line protocol).
#
# Wire protocol (one JSON object per line, newline-terminated):
#   client -> server : {"seq": int, "measurements": [[x, y], ...]}
#   server -> client : {"seq": int, "tracked": [[x, y, vx, vy], ...],
#                        "latency_us": float}
# ============================================================

class HardwareLink:
    def __init__(self, noise_std, fractional_bits=DEFAULT_FRACTIONAL_BITS):
        self.mode = "MODEL"          # "MODEL" or "PYNQ"
        self.noise_std = noise_std
        self.fractional_bits = fractional_bits
        self.filters = None          # used in MODEL mode
        self.sock = None             # used in PYNQ mode
        self.host = DEFAULT_PYNQ_HOST
        self.port = DEFAULT_PYNQ_PORT
        self.status = "MODEL (software-simulated FPGA, no board attached)"

    # ---- MODEL mode --------------------------------------------------

    def _init_model_filters(self, measurements):
        self.filters = [
            FixedPointKalmanFilter([m[0], m[1], 0.0, 0.0], self.noise_std,
                                    fractional_bits=self.fractional_bits)
            for m in measurements
        ]

    def set_fractional_bits(self, bits):
        if bits != self.fractional_bits:
            self.fractional_bits = bits
            self.filters = None  # force re-init with the new bit-width

    def set_noise(self, noise_std):
        self.noise_std = noise_std

    def reset(self):
        self.filters = None

    # ---- PYNQ mode -----------------------------------------------------

    def connect(self, host, port):
        """Try to open a real link to the PYNQ-Z2. Called from the
        'CONNECT' button. On failure, stays in / falls back to MODEL."""
        self.host, self.port = host, port
        try:
            s = socket.create_connection((host, port), timeout=SOCKET_TIMEOUT_S)
            s.settimeout(SOCKET_TIMEOUT_S)
            self.sock = s
            self.mode = "PYNQ"
            self.status = f"PYNQ CONNECTED ({host}:{port})"
            return True, self.status
        except OSError as exc:
            self.sock = None
            self.mode = "MODEL"
            self.status = f"PYNQ UNREACHABLE ({host}:{port}: {exc}) -- using MODEL"
            return False, self.status

    def disconnect(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.mode = "MODEL"
        self.status = "MODEL (disconnected by user)"

    def _send_to_pynq(self, seq, measurements):
        payload = json.dumps({"seq": seq, "measurements": measurements}) + "\n"
        self.sock.sendall(payload.encode("utf-8"))

        buf = b""
        start = time.perf_counter()
        while b"\n" not in buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("PYNQ closed the connection")
            buf += chunk
        round_trip_us = (time.perf_counter() - start) * 1e6

        line, _, _ = buf.partition(b"\n")
        reply = json.loads(line.decode("utf-8"))
        tracked = reply["tracked"]
        board_latency_us = reply.get("latency_us", round_trip_us)
        return tracked, board_latency_us, round_trip_us

    # ---- unified entry point -------------------------------------------

    def process(self, seq, measurements, dt):
        """
        Returns (tracked_states, pairs, latency_us, mode_label).
        In PYNQ mode, if anything goes wrong mid-demo (board rebooted,
        cable unplugged, etc.) this automatically falls back to MODEL for
        the rest of the run rather than crashing the GUI.
        """
        if self.mode == "PYNQ":
            try:
                tracked, board_latency_us, _ = self._send_to_pynq(seq, measurements)
                pairs = analyze_all_pairs(tracked)
                return tracked, pairs, board_latency_us, "PYNQ (hardware)"
            except (OSError, ConnectionError, ValueError, KeyError) as exc:
                self.status = f"PYNQ link lost ({exc}) -- falling back to MODEL"
                self.mode = "MODEL"
                self.sock = None
                # fall through to MODEL below

        if self.filters is None or len(self.filters) != len(measurements):
            self._init_model_filters(measurements)

        start = time.perf_counter()
        tracked = [f.step(m, dt) for f, m in zip(self.filters, measurements)]
        pairs = analyze_all_pairs(tracked)
        latency_us = (time.perf_counter() - start) * 1e6
        return tracked, pairs, latency_us, "MODEL (host CPU, fixed-point)"


# ============================================================
# REFERENCE STUB SERVER FOR THE PYNQ-Z2
#
# Run this on the board (or on a second process/terminal for testing the
# protocol locally) so `HardwareLink` in PYNQ mode has something to talk
# to. It currently just runs the same fixed-point model on whatever CPU
# it's started on -- swap `FixedPointKalmanFilter.step(...)` below for a
# real AXI-Stream/DMA call into the PL fabric once Stage 4/5 exist, and
# every client of this protocol (this whole GUI) needs no changes.
# ============================================================

def run_pynq_stub_server(host="0.0.0.0", port=DEFAULT_PYNQ_PORT,
                          fractional_bits=DEFAULT_FRACTIONAL_BITS,
                          noise_std=DEFAULT_NOISE):
    print(f"[pynq-stub] listening on {host}:{port} "
          f"(fractional_bits={fractional_bits})")
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)

    while True:
        conn, addr = server.accept()
        print(f"[pynq-stub] client connected from {addr}")
        filters = None
        buf = b""
        try:
            with conn:
                while True:
                    while b"\n" not in buf:
                        chunk = conn.recv(4096)
                        if not chunk:
                            raise ConnectionError("client disconnected")
                        buf += chunk
                    line, _, buf = buf.partition(b"\n")
                    packet = json.loads(line.decode("utf-8"))
                    measurements = packet["measurements"]

                    if filters is None or len(filters) != len(measurements):
                        filters = [
                            FixedPointKalmanFilter(
                                [m[0], m[1], 0.0, 0.0], noise_std,
                                fractional_bits=fractional_bits
                            )
                            for m in measurements
                        ]

                    start = time.perf_counter()
                    tracked = [f.step(m, dt=0.5) for f, m in zip(filters, measurements)]
                    latency_us = (time.perf_counter() - start) * 1e6

                    reply = json.dumps({
                        "seq": packet["seq"],
                        "tracked": tracked,
                        "latency_us": latency_us,
                    }) + "\n"
                    conn.sendall(reply.encode("utf-8"))
        except (ConnectionError, OSError, json.JSONDecodeError, KeyError) as exc:
            print(f"[pynq-stub] client disconnected ({exc})")


# ============================================================
# MAIN GUI
# ============================================================

def main():
    num_targets = 8
    env = VirtualSensorEnvironment(num_targets, DEFAULT_NOISE)
    software = SoftwarePipeline(
        [[t.x, t.y] for t in env.targets], DEFAULT_NOISE
    )
    hardware = HardwareLink(DEFAULT_NOISE, DEFAULT_FRACTIONAL_BITS)

    fig = plt.figure(figsize=(15, 8.5))
    fig.canvas.manager.set_window_title(
        "Live Multi-Target Tracking — Software vs FPGA-Model vs PYNQ-Z2"
    )

    ax = fig.add_axes([0.05, 0.33, 0.47, 0.60])
    info_ax = fig.add_axes([0.55, 0.33, 0.26, 0.60])
    info_ax.axis("off")

    ax.set_xlim(0, ARENA_SIZE)
    ax.set_ylim(0, ARENA_SIZE)
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_title("LIVE MULTI-TARGET SENSOR + KALMAN TRACKING")
    ax.grid(True, alpha=0.3)

    true_plot = ax.scatter([], [], s=100, label="TRUE")
    sensor_plot = ax.scatter([], [], s=35, alpha=0.6, label="SENSOR")
    kalman_plot = ax.scatter([], [], s=100, marker="x", label="KALMAN (SW)")

    # Predicted-trajectory dotted lines (one per target slot) + a marker
    # for any pair whose trajectories are about to interfere.
    traj_colors = plt.get_cmap("tab10")
    trajectory_lines = [
        ax.plot([], [], linestyle=":", linewidth=1.6,
                color=traj_colors(i % 10), alpha=0.85)[0]
        for i in range(MAX_TARGETS)
    ]
    alert_marker = ax.scatter([], [], s=220, marker="*", color="red",
                               zorder=6, label="TRAJECTORY ALERT")

    ax.legend(loc="upper left", fontsize=8)

    labels = [ax.text(0, 0, f"T{i}", fontsize=9) for i in range(MAX_TARGETS)]

    # ---- collision-risk LED panel --------------------------------------
    # One LED per possible body (T0..T7), each in its own fixed colour
    # (matching that target's trajectory-line colour). A LED glows
    # (full colour, bright ring, larger) whenever its body is part of a
    # pair the FPGA-model/PYNQ track currently predicts will collide;
    # otherwise it sits faded (dim, same colour, no ring). LEDs beyond
    # the currently active target count are shown as inert/off.
    led_ax = fig.add_axes([0.855, 0.33, 0.115, 0.60])
    led_ax.set_xlim(0, 1)
    led_ax.set_ylim(0, MAX_TARGETS + 1)
    led_ax.axis("off")
    led_ax.set_title("COLLISION\nLEDs", fontsize=9)

    LED_FADED_ALPHA = 0.22
    LED_OFF_ALPHA = 0.06
    led_colors = [traj_colors(i % 10) for i in range(MAX_TARGETS)]
    led_y_positions = [MAX_TARGETS - i for i in range(MAX_TARGETS)]

    led_plot = led_ax.scatter(
        [0.35] * MAX_TARGETS, led_y_positions, s=420,
        facecolors=[(*led_colors[i][:3], LED_FADED_ALPHA) for i in range(MAX_TARGETS)],
        edgecolors="none", linewidths=2.0, zorder=5,
    )
    led_labels = [
        led_ax.text(0.6, led_y_positions[i], f"T{i}", fontsize=9,
                    va="center", ha="left", color=led_colors[i])
        for i in range(MAX_TARGETS)
    ]

    info_text = info_ax.text(
        0, 1, "", transform=info_ax.transAxes,
        va="top", ha="left", fontsize=8.3, family="monospace"
    )

    # ---- sliders ------------------------------------------------------
    speed_ax = fig.add_axes([0.15, 0.255, 0.65, 0.025])
    speed_slider = Slider(speed_ax, "Speed", 0.1, 3.0, valinit=1.0)

    target_ax = fig.add_axes([0.15, 0.215, 0.65, 0.025])
    target_slider = Slider(target_ax, "Targets", MIN_TARGETS, MAX_TARGETS,
                            valinit=num_targets, valstep=1)

    rate_ax = fig.add_axes([0.15, 0.175, 0.65, 0.025])
    rate_slider = Slider(rate_ax, "Sensor Rate (Hz)", 0.5, 5.0,
                          valinit=2.0, valstep=0.1)

    noise_ax = fig.add_axes([0.15, 0.135, 0.65, 0.025])
    noise_slider = Slider(noise_ax, "Sensor Noise", 0.1, 5.0,
                           valinit=1.0, valstep=0.1)

    bits_ax = fig.add_axes([0.15, 0.095, 0.65, 0.025])
    bits_slider = Slider(bits_ax, "FPGA-model Bits", 2, 16,
                          valinit=DEFAULT_FRACTIONAL_BITS, valstep=1)

    reset_ax = fig.add_axes([0.83, 0.20, 0.11, 0.05])
    reset_button = Button(reset_ax, "RESET")

    trajectory_toggle_ax = fig.add_axes([0.83, 0.13, 0.11, 0.05])
    trajectory_toggle_button = Button(trajectory_toggle_ax, "Trajectory: ON")

    # PYNQ connection controls
    host_box_ax = fig.add_axes([0.15, 0.03, 0.22, 0.045])
    host_box = TextBox(host_box_ax, "PYNQ host  ", initial=DEFAULT_PYNQ_HOST)

    port_box_ax = fig.add_axes([0.40, 0.03, 0.08, 0.045])
    port_box = TextBox(port_box_ax, "port ", initial=str(DEFAULT_PYNQ_PORT))

    connect_ax = fig.add_axes([0.50, 0.03, 0.14, 0.045])
    connect_button = Button(connect_ax, "CONNECT PYNQ")

    disconnect_ax = fig.add_axes([0.66, 0.03, 0.16, 0.045])
    disconnect_button = Button(disconnect_ax, "USE MODEL ONLY")

    runtime = {
        "env": env,
        "software": software,
        "hardware": hardware,
        "last_sensor_time": time.perf_counter(),
        "reading_count": 0,
        "start_time": time.perf_counter(),
        "measurements": [None] * num_targets,
        "ground_truth": [None] * num_targets,
        "sw_tracked": [None] * num_targets,
        "hw_tracked": [None] * num_targets,
        "sw_latency_us": None,
        "hw_latency_us": None,
        "hw_mode_label": "MODEL (host CPU, fixed-point)",
        "pairs": [],
        "trajectories": {},
        "trajectory_alerts": [],
        "show_trajectory": True,
    }

    # ---- RESET ----------------------------------------------------------

    def reset_scene(event):
        number = int(target_slider.val)
        noise = noise_slider.val
        bits = int(bits_slider.val)

        new_env = VirtualSensorEnvironment(number, noise)
        new_software = SoftwarePipeline([[t.x, t.y] for t in new_env.targets], noise)

        runtime["env"] = new_env
        runtime["software"] = new_software
        runtime["hardware"].set_noise(noise)
        runtime["hardware"].set_fractional_bits(bits)
        runtime["hardware"].reset()

        runtime["last_sensor_time"] = time.perf_counter()
        runtime["reading_count"] = 0
        runtime["start_time"] = time.perf_counter()
        runtime["measurements"] = [None] * number
        runtime["ground_truth"] = [None] * number
        runtime["sw_tracked"] = [None] * number
        runtime["hw_tracked"] = [None] * number
        runtime["sw_latency_us"] = None
        runtime["hw_latency_us"] = None
        runtime["pairs"] = []
        runtime["trajectories"] = {}
        runtime["trajectory_alerts"] = []

        sensor_plot.set_offsets(np.empty((0, 2)))
        kalman_plot.set_offsets(np.empty((0, 2)))
        for line in trajectory_lines:
            line.set_data([], [])
        alert_marker.set_offsets(np.empty((0, 2)))

        print(f"\n=== NEW SCENE: targets={number} noise={noise} bits={bits} ===")

    reset_button.on_clicked(reset_scene)

    # ---- trajectory toggle ------------------------------------------------

    def on_toggle_trajectory(event):
        runtime["show_trajectory"] = not runtime["show_trajectory"]
        trajectory_toggle_button.label.set_text(
            "Trajectory: ON" if runtime["show_trajectory"] else "Trajectory: OFF"
        )

    trajectory_toggle_button.on_clicked(on_toggle_trajectory)

    # ---- PYNQ connect / disconnect ---------------------------------------

    def on_connect(event):
        host = host_box.text.strip()
        try:
            port = int(port_box.text.strip())
        except ValueError:
            print("[link] invalid port, ignoring")
            return
        ok, status = runtime["hardware"].connect(host, port)
        print(f"[link] {status}")

    def on_disconnect(event):
        runtime["hardware"].disconnect()
        print("[link] switched back to MODEL only")

    connect_button.on_clicked(on_connect)
    disconnect_button.on_clicked(on_disconnect)

    # ---- info panel text --------------------------------------------------

    def format_info():
        env = runtime["env"]
        targets = env.targets
        number = len(targets)
        pairs_count = number * (number - 1) // 2
        elapsed = time.perf_counter() - runtime["start_time"]
        hw = runtime["hardware"]

        lines = []
        lines.append("SYSTEM STATUS")
        lines.append("=" * 46)
        lines.append(f"Targets     : {number}   Pairs: {pairs_count}")
        lines.append(f"Sensor rate : {rate_slider.val:.1f} Hz    "
                      f"Noise: {noise_slider.val:.1f} m")
        lines.append(f"Readings    : {runtime['reading_count']}   "
                      f"Time: {elapsed:.1f} s")
        lines.append(f"Link        : {hw.status}")
        lines.append("")

        lines.append("LATENCY COMPARISON  (Stage-3 style)")
        lines.append("-" * 46)
        sw_us = runtime["sw_latency_us"]
        hw_us = runtime["hw_latency_us"]
        if sw_us is not None and hw_us is not None:
            lines.append(f"Software (CPU, float)   : {sw_us:8.1f} us")
            lines.append(f"{runtime['hw_mode_label']:24s}: {hw_us:8.1f} us")
            if hw.mode == "PYNQ" and hw_us > 0:
                lines.append(f"Speedup (SW / PYNQ)     : {sw_us / hw_us:6.2f} x")
            else:
                lines.append("Speedup                 : n/a (attach PYNQ-Z2")
                lines.append("                           for a real number)")
        else:
            lines.append("(waiting for first reading...)")
        lines.append("")

        lines.append("TOP COLLISION RISK  (from FPGA-model/PYNQ track)")
        lines.append("-" * 46)
        if runtime["pairs"]:
            top = runtime["pairs"][0]
            i, j = top["pair"]
            decision = "COLLISION PREDICTED" if top["collision"] else "clear"
            lines.append(f"Pair T{i} <-> T{j}   risk={top['risk']}")
            lines.append(f"closest dist = {top['closest_distance']:.2f} m   "
                         f"ttc = {top['ttc']:.2f} s")
            lines.append(f"decision: {decision}")
        else:
            lines.append("(waiting for first reading...)")
        lines.append("")

        lines.append("TRAJECTORY ALERTS  (predicted paths interfering)")
        lines.append("-" * 46)
        if not runtime["show_trajectory"]:
            lines.append("(trajectory lines hidden)")
        elif runtime["trajectory_alerts"]:
            for a in runtime["trajectory_alerts"]:
                i, j = a["pair"]
                lines.append(f"\u26a0 T{i} <-> T{j}   min gap = {a['distance']:.2f} m")
        else:
            lines.append("none")
        lines.append("")

        lines.append("TARGET DATA  (TRUE / MEAS / KAL-SW / KAL-HW)")
        lines.append("-" * 46)
        for i, t in enumerate(targets):
            m = runtime["measurements"][i]
            sw = runtime["sw_tracked"][i]
            hwv = runtime["hw_tracked"][i]
            m_s = f"({m[0]:5.1f},{m[1]:5.1f})" if m else "  waiting  "
            sw_s = f"({sw[0]:5.1f},{sw[1]:5.1f})" if sw else "  waiting  "
            hw_s = f"({hwv[0]:5.1f},{hwv[1]:5.1f})" if hwv else "  waiting  "
            lines.append(
                f"T{i} T=({t.x:5.1f},{t.y:5.1f}) M={m_s} SW={sw_s} HW={hw_s}"
            )

        return "\n".join(lines)

    # ---- animation loop ------------------------------------------------

    def update(frame):
        env = runtime["env"]
        targets = env.targets

        env.advance_motion(ANIMATION_DT, speed_slider.val)
        env.set_noise(noise_slider.val)

        true_positions = [[t.x, t.y] for t in targets]
        true_plot.set_offsets(np.array(true_positions))

        now = time.perf_counter()
        sensor_period = 1.0 / rate_slider.val

        if now - runtime["last_sensor_time"] >= sensor_period:
            dt_elapsed = now - runtime["last_sensor_time"]
            runtime["last_sensor_time"] = now
            runtime["reading_count"] += 1

            packet = env.read_packet()
            measurements = packet["measurements"]

            runtime["hardware"].set_fractional_bits(int(bits_slider.val))

            sw_tracked, sw_pairs, sw_latency_us = runtime["software"].process(
                measurements, dt_elapsed
            )
            hw_tracked, hw_pairs, hw_latency_us, hw_mode_label = runtime["hardware"].process(
                packet["seq"], measurements, dt_elapsed
            )

            runtime["measurements"] = measurements
            runtime["sw_tracked"] = sw_tracked
            runtime["hw_tracked"] = hw_tracked
            runtime["sw_latency_us"] = sw_latency_us
            runtime["hw_latency_us"] = hw_latency_us
            runtime["hw_mode_label"] = hw_mode_label
            runtime["pairs"] = hw_pairs

            sensor_plot.set_offsets(np.array(measurements))
            kalman_plot.set_offsets(np.array([[s[0], s[1]] for s in sw_tracked]))

            # ---- predicted-trajectory lines + interference detection ----
            trajectories = {}
            for i, est in enumerate(sw_tracked):
                curr_pos = (est[0], est[1])
                vx, vy = est[2], est[3]
                seg = trajectory_segment(curr_pos, vx, vy, dt_elapsed)
                if seg is not None:
                    trajectories[i] = seg
            runtime["trajectories"] = trajectories
            runtime["trajectory_alerts"] = find_trajectory_alerts(trajectories)

            print(f"\nREADING #{runtime['reading_count']}  dt={dt_elapsed:.3f}s  "
                  f"link={hw_mode_label}  SW={sw_latency_us:.1f}us  "
                  f"HW={hw_latency_us:.1f}us")
            if hw_pairs:
                top = hw_pairs[0]
                print(f"  top risk: T{top['pair'][0]}<->T{top['pair'][1]} "
                      f"dist={top['closest_distance']:.2f}m ttc={top['ttc']:.2f}s "
                      f"risk={top['risk']}")
            for a in runtime["trajectory_alerts"]:
                i, j = a["pair"]
                print(f"  \u26a0 TRAJECTORY ALERT: T{i} <-> T{j} "
                      f"min gap={a['distance']:.2f}m")

        # ---- draw / hide the predicted-trajectory lines ------------------
        alerted_targets = set()
        for a in runtime["trajectory_alerts"]:
            alerted_targets.update(a["pair"])

        for i in range(MAX_TARGETS):
            line = trajectory_lines[i]
            if runtime["show_trajectory"] and i in runtime["trajectories"]:
                start, end = runtime["trajectories"][i]
                line.set_data([start[0], end[0]], [start[1], end[1]])
                if i in alerted_targets:
                    line.set_color("red")
                    line.set_linewidth(2.4)
                    line.set_alpha(1.0)
                else:
                    line.set_color(traj_colors(i % 10))
                    line.set_linewidth(1.6)
                    line.set_alpha(0.85)
                line.set_visible(True)
            else:
                line.set_visible(False)

        if runtime["show_trajectory"] and runtime["trajectory_alerts"]:
            alert_points = []
            for a in runtime["trajectory_alerts"]:
                for idx in a["pair"]:
                    start, end = runtime["trajectories"][idx]
                    alert_points.append(end)
            alert_marker.set_offsets(np.array(alert_points))
            alert_marker.set_visible(True)
        else:
            alert_marker.set_offsets(np.empty((0, 2)))
            alert_marker.set_visible(False)

        for i in range(MAX_TARGETS):
            if i < len(targets):
                t = targets[i]
                labels[i].set_position((t.x + 1.5, t.y + 1.5))
                labels[i].set_visible(True)
            else:
                labels[i].set_visible(False)

        # ---- collision-risk LEDs ------------------------------------
        collision_targets = set()
        for p in runtime["pairs"]:
            if p["collision"]:
                collision_targets.update(p["pair"])

        led_facecolors = []
        led_edgecolors = []
        led_sizes = []
        for i in range(MAX_TARGETS):
            r, g, b = led_colors[i][:3]
            if i >= len(targets):
                led_facecolors.append((r, g, b, LED_OFF_ALPHA))
                led_edgecolors.append((0, 0, 0, 0))
                led_sizes.append(300)
                led_labels[i].set_alpha(0.25)
            elif i in collision_targets:
                led_facecolors.append((r, g, b, 1.0))
                led_edgecolors.append((1.0, 1.0, 1.0, 1.0))
                led_sizes.append(520)
                led_labels[i].set_alpha(1.0)
            else:
                led_facecolors.append((r, g, b, LED_FADED_ALPHA))
                led_edgecolors.append((0, 0, 0, 0))
                led_sizes.append(420)
                led_labels[i].set_alpha(0.6)
        led_plot.set_facecolors(led_facecolors)
        led_plot.set_edgecolors(led_edgecolors)
        led_plot.set_sizes(led_sizes)

        info_text.set_text(format_info())

        return ([true_plot, sensor_plot, kalman_plot, info_text, alert_marker, led_plot]
                + labels + trajectory_lines + led_labels)

    animation = FuncAnimation(fig, update, interval=30, blit=False)
    plt.show()


if __name__ == "__main__":
    if "--server" in sys.argv:
        port = DEFAULT_PYNQ_PORT
        if "--port" in sys.argv:
            port = int(sys.argv[sys.argv.index("--port") + 1])
        run_pynq_stub_server(port=port)
    else:
        main()
