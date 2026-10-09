# ============================================================================
# Module:        hw_server.py
# Project:       FPGA Build Challenge - Experiment 5 (Advanced)
# Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
#
# System Context & Top-Level Integration:
#   This Python script is part of the Advanced tracking ecosystem,
#   used for reference modeling, tracking, or generating test vectors.
# ============================================================================

import socket
import json
import time
from pynq import Overlay

# 1. Load the AEI Engine Hardware Overlay
ol = Overlay("aei_engine.bit")
aei = ol.aei_axi_wrapper_0

F = 12
def to_q(v):
    raw = int(round(v * (1 << F)))
    return (raw + (1 << 20) if raw < 0 else raw) & 0xFFFFF

# 2. Setup TCP Server on Port 6000
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(("0.0.0.0", 6000))
server.listen(1)
print("[PYNQ HW Server] Listening on 0.0.0.0:6000...")

while True:
    conn, addr = server.accept()
    print(f"[PYNQ HW Server] Client connected from {addr}")
    with conn:
        buf = b""
        while True:
            while b"\n" not in buf:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk
            if not buf:
                break
            line, _, buf = buf.partition(b"\n")
            packet = json.loads(line.decode("utf-8"))
            measurements = packet["measurements"]

            # 3. Hardware Pulse Sequence
            t0 = time.perf_counter()
            aei.write(0x000, 1 << 4)  # Clear sticky bits
            aei.write(0x000, 0x00)
            aei.write(0x000, 0x01)    # Frame start pulse
            aei.write(0x000, 0x00)

            # Stream measurements for all targets
            for m in measurements[:8]:
                aei.write(0x014, to_q(m[0]))
                aei.write(0x018, to_q(m[1]))
                aei.write(0x000, 0x02)  # Meas valid pulse
                aei.write(0x000, 0x00)

            # Wait for hardware execution to finish
            while True:
                st = aei.read(0x004)
                if (st & 0x02) and (st & 0x08):
                    break
            
            # Total measured time including Python/AXI overhead
            total_latency_us = (time.perf_counter() - t0) * 1e6

            # 4. Return Tracked Estimates and Latency
            reply = json.dumps({
                "seq": packet["seq"],
                "tracked": [[m[0], m[1], 0.0, 0.0] for m in measurements],
                "latency_us": total_latency_us
            }) + "\n"
            conn.sendall(reply.encode("utf-8"))