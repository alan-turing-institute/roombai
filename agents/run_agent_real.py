"""
Dead-reckoning real-robot agent for RoombaI.

Runs a trained PPO policy on the real Roomba without a camera by computing
observations analytically from a known start position + accumulated movement.

Observations that cannot be measured (lidar [0-7]) are zeroed.
Door state (obs[9]) is hardcoded to open (1.0) — assume the door is open.
Distance (obs[8]) and heading (obs[10-11]) are derived from dead reckoning.
Bumpers (obs[12]) are read from the pilot via the 'bumps' command.

Usage:
    cd agents
    uv run run_agent_real.py models/best/best_model.zip
    uv run run_agent_real.py models/best/best_model.zip --start-heading -162 --max-steps 500
"""

import argparse
import math
import re
import socket
import time

import numpy as np
from stable_baselines3 import PPO

# ---------------------------------------------------------------------------
# Map constants (derived from simulator/src/map_data.rs)
# pdf_pt(x, y) expands to (x*36, y*36) millimetres
# ---------------------------------------------------------------------------

START_X_MM = 54_720.0           # pdf_pt(1520, 775).x
START_Y_MM = 27_900.0           # pdf_pt(1520, 775).y

# Door 13 centre: midpoint of pdf_pt(983.95, 618.48) to pdf_pt(1031.84, 618.48)
DOOR13_X_MM = (983.95 + 1031.84) / 2 * 36   # ≈ 36 284 mm
DOOR13_Y_MM = 618.48 * 36                    # ≈ 22 265 mm

MAX_DIST_MM = 60_000.0

# ---------------------------------------------------------------------------
# Actions — must match roomba_env.py ACTIONS exactly
# ---------------------------------------------------------------------------

ACTIONS = [
    (0, "move 30",  "Fwd 30 cm"),
    (1, "move -15", "Back 15 cm"),
    (2, "turn 45",  "Left 45°"),
    (3, "turn -45", "Right 45°"),
    (4, "turn 90",  "Left 90°"),
]

# Dead-reckoning effects: ("move", mm) or ("turn", degrees)
ACTION_EFFECTS = [
    ("move",  300.0),    # 0: fwd 30 cm → +300 mm
    ("move", -150.0),    # 1: back 15 cm → -150 mm
    ("turn",   45.0),    # 2: left 45°
    ("turn",  -45.0),    # 3: right 45°
    ("turn",   90.0),    # 4: left 90°
]


# ---------------------------------------------------------------------------
# Dead reckoning
# ---------------------------------------------------------------------------

class DeadReckoningTracker:
    """Tracks (x, y, heading) from a known start using accumulated commands.

    Heading convention matches the simulator:
      heading=0 → facing right (+x), π/2 → facing up (+y)
      dx = dist * cos(heading),  dy = dist * sin(heading)
    """

    def __init__(self, start_heading_deg: float = 0.0):
        self.x = START_X_MM
        self.y = START_Y_MM
        self.heading = math.radians(start_heading_deg)

    def apply_action(self, action_idx: int) -> None:
        kind, amount = ACTION_EFFECTS[action_idx]
        if kind == "move":
            self.x += amount * math.cos(self.heading)
            self.y += amount * math.sin(self.heading)
        else:
            self.heading += math.radians(amount)

    @property
    def distance_to_door(self) -> float:
        return math.hypot(DOOR13_X_MM - self.x, DOOR13_Y_MM - self.y)

    def obs_fields(self):
        """Return (dist_norm, sin_heading, cos_heading)."""
        dist_norm = min(self.distance_to_door / MAX_DIST_MM, 1.0)
        return dist_norm, math.sin(self.heading), math.cos(self.heading)


# ---------------------------------------------------------------------------
# TCP helpers (mirrors record_demo.py)
# ---------------------------------------------------------------------------

def connect(host: str, port: int) -> socket.socket:
    for _ in range(20):
        try:
            s = socket.create_connection((host, port), timeout=3.0)
            s.settimeout(5.0)
            return s
        except OSError:
            time.sleep(0.5)
    raise ConnectionRefusedError(f"Cannot connect to {host}:{port}. Is the pilot running?")


def cmd(sock: socket.socket, command: str) -> str:
    sock.sendall((command + "\n").encode())
    data = b""
    while b"\n" not in data:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionResetError("Connection closed")
        data += chunk
    return data.split(b"\n")[0].decode().strip()


# ---------------------------------------------------------------------------
# Observation assembly
# ---------------------------------------------------------------------------

def get_obs(tracker: DeadReckoningTracker, sock: socket.socket) -> np.ndarray:
    obs = np.zeros(13, dtype=np.float32)

    # obs[0-7]: lidar — no hardware equivalent, leave as zero
    # obs[8-11]: dead reckoning
    dist_n, sin_h, cos_h = tracker.obs_fields()
    obs[8]  = dist_n
    obs[9]  = 1.0       # assume door is open
    obs[10] = sin_h
    obs[11] = cos_h

    # obs[12]: bumpers via pilot
    try:
        bumps_r = cmd(sock, "bumps")
        bvals = re.findall(r"bump[LR]=(\d)", bumps_r)
        obs[12] = float(any(v == "1" for v in bvals))
    except Exception:
        obs[12] = 0.0

    return obs


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run(
    model_path: str,
    host: str,
    port: int,
    start_heading_deg: float,
    max_steps: int,
    escape_dist_mm: float,
) -> None:
    print(f"Loading model from {model_path}")
    model = PPO.load(model_path, device="cpu")

    tracker = DeadReckoningTracker(start_heading_deg)
    print(f"Start: ({tracker.x:.0f}, {tracker.y:.0f}) mm  heading={start_heading_deg}°")
    print(f"Door 13: ({DOOR13_X_MM:.0f}, {DOOR13_Y_MM:.0f}) mm")
    print(f"Initial distance to door: {tracker.distance_to_door:.0f} mm")

    print(f"Connecting to {host}:{port}…")
    sock = connect(host, port)
    print("Connected.")

    resp = cmd(sock, "safe")
    print(f"safe → {resp}")

    for step in range(max_steps):
        obs = get_obs(tracker, sock)
        action, _ = model.predict(obs, deterministic=True)
        action_idx = int(action)
        _, pilot_cmd_str, description = ACTIONS[action_idx]

        try:
            resp = cmd(sock, pilot_cmd_str)
        except Exception as e:
            print(f"[{step:4d}] command failed: {e}")
            break

        tracker.apply_action(action_idx)
        dist = tracker.distance_to_door

        print(
            f"[{step:4d}] a={action_idx} {pilot_cmd_str:<12} {description:<14} "
            f"dist={dist:.0f}mm  pos=({tracker.x:.0f},{tracker.y:.0f})  {resp}"
        )

        if dist < escape_dist_mm:
            print(f"Reached Door 13! (dist={dist:.0f}mm < {escape_dist_mm:.0f}mm threshold)")
            break
    else:
        print(f"Reached max steps ({max_steps}) without escaping.")

    cmd(sock, "stop")
    sock.close()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a trained PPO agent on the real Roomba using dead reckoning."
    )
    parser.add_argument("model", help="Path to the trained model .zip")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9999)
    parser.add_argument(
        "--start-heading", type=float, default=0.0,
        help="Initial heading in degrees (0 = facing right/+x, same as simulator default). "
             "Adjust if the Roomba is placed facing a different direction.",
    )
    parser.add_argument("--max-steps", type=int, default=500)
    parser.add_argument(
        "--escape-dist", type=float, default=1500.0,
        help="Distance from Door 13 centre (mm) at which the run is declared a success.",
    )
    args = parser.parse_args()

    run(
        model_path=args.model,
        host=args.host,
        port=args.port,
        start_heading_deg=args.start_heading,
        max_steps=args.max_steps,
        escape_dist_mm=args.escape_dist,
    )


if __name__ == "__main__":
    main()
