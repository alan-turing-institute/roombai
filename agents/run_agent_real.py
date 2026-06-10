"""
Dead-reckoning real-robot agent for RoombaI.

Runs a trained PPO policy on the real Roomba without a camera by computing
observations analytically from a known start position + accumulated movement.

Lidar (obs[0-7]): approximate wall distances computed by ray-casting against
  the static wall map (map_walls.py). Dynamic obstacles and humans are not
  included — rays will read as max-range for those directions.
Distance (obs[8]) and heading (obs[10-11]): derived from dead reckoning.
Door state (obs[9]): hardcoded to open (1.0) — assume the door is open.
Bumpers (obs[12]): read from the pilot via the 'bumps' command.

Usage:
    cd agents
    uv run run_agent_real.py models/best/best_model.zip
    uv run run_agent_real.py models/best/best_model.zip --start-heading -162 --max-steps 500
"""

import argparse
from collections import deque
import math
import re
import socket
import time

import numpy as np
from stable_baselines3 import PPO

from map_walls import WALLS


def get_shielded_action(obs, predicted_action, action_history) -> tuple[int, str | None]:
    front_dist_mm = obs[0] * 50_000.0
    back_dist_mm = obs[4] * 50_000.0
    left_dist_mm = obs[2] * 50_000.0
    right_dist_mm = obs[6] * 50_000.0
    bumper_active = obs[12] > 0.5

    # 1. Stuck / Oscillation detection
    is_stuck = False
    stuck_reason = ""
    
    # Check for alternating turn loop (e.g., [2, 3, 2, 3] or [3, 2, 3, 2])
    if len(action_history) >= 4:
        last_four = list(action_history)[-4:]
        if last_four == [2, 3, 2, 3] or last_four == [3, 2, 3, 2]:
            is_stuck = True
            stuck_reason = "oscillation loop"
            
    # Check for spinning in place while front is close (e.g. 4 turns in a row)
    if not is_stuck and len(action_history) >= 4 and front_dist_mm < 400.0:
        last_four = list(action_history)[-4:]
        if all(a in [2, 3, 4] for a in last_four):
            is_stuck = True
            stuck_reason = "spinning in place"

    if is_stuck:
        if back_dist_mm > 200.0:
            action_history.clear()
            return 1, f"{stuck_reason.capitalize()} detected override: forcing backup (move -15) to escape"
        else:
            action_history.clear()
            return 4, f"{stuck_reason.capitalize()} detected override: forcing turn 90 to break cycle"

    # 2. Bumper Active Check
    if bumper_active:
        if predicted_action != 1:
            if back_dist_mm > 150.0:
                return 1, "Bumper active override: forcing backup (move -15)"
            else:
                if left_dist_mm >= right_dist_mm:
                    return 4, "Bumper active (rear blocked) override: forcing turn 90"
                else:
                    return 3, "Bumper active (rear blocked) override: forcing turn right (turn -45)"

    # 3. Proximity check
    if front_dist_mm < 400.0 and predicted_action == 0:
        if front_dist_mm < 250.0 and back_dist_mm > 200.0:
            return 1, f"Front obstacle extremely close ({front_dist_mm:.1f} mm) override: forcing backup (move -15)"
        else:
            if left_dist_mm >= right_dist_mm:
                return 2, f"Front obstacle too close ({front_dist_mm:.1f} mm) override: turning left (turn 45)"
            else:
                return 3, f"Front obstacle too close ({front_dist_mm:.1f} mm) override: turning right (turn -45)"

    return predicted_action, None

# ---------------------------------------------------------------------------
# Map constants (derived from simulator/src/map_data.rs)
# pdf_pt(x, y) expands to (x*36, y*36) millimetres
# ---------------------------------------------------------------------------

START_X_MM = 49_320.0           # pdf_pt(1370, 825).x
START_Y_MM = 29_700.0           # pdf_pt(1370, 825).y

# Door 13 centre: midpoint of pdf_pt(983.95, 618.48) to pdf_pt(1031.84, 618.48)
DOOR13_X_MM = (983.95 + 1031.84) / 2 * 36   # ≈ 36 284 mm
DOOR13_Y_MM = 618.48 * 36                    # ≈ 22 265 mm

MAX_DIST_MM  = 60_000.0
MAX_LIDAR_MM = 50_000.0   # 500 cm — matches MAX_LIDAR_CM in roomba_env.py

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
# Map-based lidar approximation (walls only — obstacles/humans excluded)
# ---------------------------------------------------------------------------

def _ray_segment_dist(ox, oy, dx, dy, x1, y1, x2, y2):
    """Return distance along ray (ox,oy)+t*(dx,dy) to segment, or None."""
    vx, vy = x2 - x1, y2 - y1
    denom = dx * vy - dy * vx
    if abs(denom) < 1e-6:
        return None
    t = ((x1 - ox) * vy - (y1 - oy) * vx) / denom
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


def _cast_ray(x, y, angle_rad):
    dx, dy = math.cos(angle_rad), math.sin(angle_rad)
    min_t = MAX_LIDAR_MM
    for (x1, y1), (x2, y2) in WALLS:
        t = _ray_segment_dist(x, y, dx, dy, x1, y1, x2, y2)
        if t is not None and t < min_t:
            min_t = t
    return min_t


def compute_lidar(tracker) -> np.ndarray:
    """Approximate 8-ray lidar from dead-reckoned pose against static wall map."""
    angles_deg = [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]
    result = np.empty(8, dtype=np.float32)
    for i, deg in enumerate(angles_deg):
        dist = _cast_ray(tracker.x, tracker.y, tracker.heading + math.radians(deg))
        result[i] = min(dist / MAX_LIDAR_MM, 1.0)
    return result


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

    # obs[0-7]: approximate wall distances via map-based ray cast
    obs[0:8] = compute_lidar(tracker)

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

    action_history = deque(maxlen=4)

    for step in range(max_steps):
        obs = get_obs(tracker, sock)
        action, _ = model.predict(obs, deterministic=True)
        action_idx = int(action)

        # Apply safety shield
        shielded_action, override_msg = get_shielded_action(obs, action_idx, action_history)
        if override_msg:
            print(f"[{step:4d}] {override_msg}")
            action_idx = shielded_action

        action_history.append(action_idx)

        kind, _ = ACTION_EFFECTS[action_idx]
        _, pilot_cmd_str, description = ACTIONS[action_idx]

        try:
            resp = cmd(sock, pilot_cmd_str)
        except Exception as e:
            print(f"[{step:4d}] command failed: {e}")
            break

        # For move commands, check whether the robot was physically blocked.
        # If bumped, back off 10 cm and correct the tracker instead of applying
        # the blocked forward move — prevents dead-reckoning from diverging.
        bumped = False
        if kind == "move":
            try:
                bumps_r = cmd(sock, "bumps")
                bvals = re.findall(r"bump[LR]=(\d)", bumps_r)
                bumped = any(v == "1" for v in bvals)
            except Exception:
                pass

        if bumped:
            try:
                cmd(sock, "move -10")   # back off 10 cm to relieve motors
            except Exception:
                pass
            # Don't apply the blocked forward move; apply the 10 cm backup instead
            tracker.x -= 100.0 * math.cos(tracker.heading)
            tracker.y -= 100.0 * math.sin(tracker.heading)
            print(f"[{step:4d}] a={action_idx} {pilot_cmd_str:<12} BUMPED — backed off 10 cm")
        else:
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
        "--start-heading", type=float, default=-90.0,
        help="Initial heading in degrees (0 = facing right/+x, -90 = facing south). "
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
