#!/usr/bin/env python3
"""
find_door.py — Navigate the Roomba to find and stop in a doorway.

Vision uses the Raspberry Pi AI Hat+ (Hailo) running YOLOv8 for object
detection, plus OpenCV geometric analysis for door/doorway recognition.
Consecutive frames are compared using wheel-odometry baselines to estimate
distance to the door and detect stuck/wheel-spin conditions.

Before driving forward, the script checks which thirds of the frame are
blocked by detected obstacles and steers toward the clearest path.

No external LLM calls.

Strategy:
  1. SCAN    — spin in 45° steps, capture + analyse at each stop.
  2. APPROACH — drive in chunks; check obstacle map before each burst,
               steer around obstacles, use actual driven distance as the
               optical-flow baseline.
  3. WAIT    — poll camera until door opens.
  4. ARRIVE  — stop when in_doorway or distance < ARRIVE_DIST_CM.
"""

import socket
import subprocess
import sys
import time
from pathlib import Path

import cv2

from vision import (
    OdometryTracker,
    analyze_motion,
    analyze_obstacles,
    detect_door_cv,
    init_hailo,
    run_yolo,
    yolo_labels,
)

# ── Config ────────────────────────────────────────────────────────────────────
PILOT_HOST = "127.0.0.1"
PILOT_PORT = 9999

PHOTO_PATH = Path("/tmp/roomba_view.jpg")
PHOTO_W    = 640
PHOTO_H    = 480

SCAN_STEPS      = 8
SCAN_STEP_DEG   = 360.0 / SCAN_STEPS
TURN_RATE_DEG_S = 60.0

DRIVE_SPEED_CM_S = 20.0
DRIVE_CHUNK_CM   = 30.0
STEER_DEG        = 20.0
BUMP_BACKUP_CM   = 20.0

WAIT_POLL_S     = 2.0
MAX_SCAN_ROUNDS = 6
MAX_WAIT_POLLS  = 30

SLOW_DIST_CM     = 150.0   # slow to half speed when door closer than this
ARRIVE_DIST_CM   = 40.0    # stop and declare arrival when this close
STUCK_RECOVERIES = 2

# Proactive avoidance: steer if a blocking obstacle is closer than this
AVOID_STEER_DIST_CM = 120.0

# ── Module-level state (single-threaded — no locks needed) ───────────────────
odom      = OdometryTracker()
_prev_img = None   # last captured BGR image for flow comparison


def capture_and_analyse(baseline_cm: float = 0.0) -> dict:
    """
    Capture a frame, run door detection + YOLO obstacles, and (if baseline_cm > 0)
    optical-flow motion analysis against the previous frame.

    baseline_cm — actual distance driven since last capture (from odometry).
                  Pass 0 when stationary (scan steps, wait polling).

    Returns merged dict:
      door_visible, door_open, door_position, in_doorway,
      door_pixel_width, door_distance_cm,
      person_visible,
      obstacles      — list of detection dicts from analyze_obstacles()
      blocked        — {"left": bool, "center": bool, "right": bool}
      clear_path     — "left" | "center" | "right" | None
      nearest_cm     — distance to nearest blocking obstacle (cm) or None
      scene_depth_cm, stuck, flow_px,
      notes
    """
    global _prev_img

    subprocess.run(
        ["rpicam-still", "--nopreview",
         "--width", str(PHOTO_W), "--height", str(PHOTO_H),
         "--rotation", "180",
         "-o", str(PHOTO_PATH), "-t", "500"],
        check=True, capture_output=True,
    )

    curr_img = cv2.imread(str(PHOTO_PATH))
    null: dict = {
        "door_visible": False, "door_open": False, "door_position": None,
        "in_doorway": False, "door_pixel_width": 0, "door_distance_cm": None,
        "person_visible": False,
        "obstacles": [], "blocked": {"left": False, "center": False, "right": False},
        "clear_path": None, "nearest_cm": None,
        "scene_depth_cm": None, "stuck": False, "flow_px": 0.0,
        "notes": "image read failed",
    }
    if curr_img is None:
        return null

    frame_h, frame_w = curr_img.shape[:2]

    # ── Door detection ────────────────────────────────────────────────────
    door = detect_door_cv(curr_img)

    # ── YOLO + obstacle map ───────────────────────────────────────────────
    detections = run_yolo(curr_img)
    labels     = yolo_labels(detections)

    # Motion analysis uses scene_depth as fallback for unknown-height objects
    motion: dict = {"scene_depth_cm": None, "stuck": False, "flow_px": 0.0}
    if _prev_img is not None and baseline_cm > 0.0:
        motion = analyze_motion(_prev_img, curr_img, baseline_cm)

    obs_map = analyze_obstacles(detections, frame_w, frame_h, motion.get("scene_depth_cm"))

    _prev_img = curr_img

    result = {
        **door,
        "person_visible": "person" in labels,
        "obstacles":      obs_map["obstacles"],
        "blocked":        obs_map["blocked"],
        "clear_path":     obs_map["clear_path"],
        "nearest_cm":     obs_map["nearest_cm"],
        "scene_depth_cm": motion.get("scene_depth_cm"),
        "stuck":          motion.get("stuck", False),
        "flow_px":        motion.get("flow_px", 0.0),
    }

    # ── Print summary ─────────────────────────────────────────────────────
    blocking = [d for d in obs_map["obstacles"] if d["blocking"]]
    block_str = ", ".join(
        f"{d['class_name']}@{d['position']}~{d['distance_cm']}cm"
        for d in blocking
    ) or "none"
    print(
        f"  [vision] door={result['door_visible']} open={result['door_open']} "
        f"pos={result['door_position']} in_doorway={result['in_doorway']} "
        f"door_dist≈{result.get('door_distance_cm')}cm | "
        f"obstacles=[{block_str}] blocked={obs_map['blocked']} clear={obs_map['clear_path']} "
        f"depth≈{result['scene_depth_cm']}cm flow={result['flow_px']:.1f}px "
        f"stuck={result['stuck']} person={result['person_visible']}"
    )
    return result


def _choose_avoid_turn(blocked: dict, clear_path: str | None) -> float:
    """
    Choose a steering angle to move toward the clearest third.
    Returns degrees (+CCW / −CW).
    """
    if clear_path == "right":
        return -STEER_DEG
    if clear_path == "left":
        return +STEER_DEG
    return -90.0   # all blocked — large CW turn


# ── Pilot communication ───────────────────────────────────────────────────────
def send_cmd(cmd: str) -> str:
    try:
        with socket.create_connection((PILOT_HOST, PILOT_PORT), timeout=5) as s:
            s.sendall((cmd + "\n").encode())
            s.settimeout(5)
            reply = s.makefile().readline().strip()
            print(f"  pilot> {cmd!r}  →  {reply}")
            return reply
    except Exception as e:
        print(f"  pilot> {cmd!r}  →  ERROR: {e}")
        return f"ERR {e}"


def bumped() -> bool:
    r = send_cmd("bumps")
    return "bumpL=1" in r or "bumpR=1" in r


def turn(deg: float):
    send_cmd(f"turn {deg:.0f}")
    odom.turn(deg)
    time.sleep(abs(deg) / TURN_RATE_DEG_S + 0.4)


def drive_forward(cm: float, speed: float = DRIVE_SPEED_CM_S) -> float:
    secs = cm / speed
    send_cmd(f"forward {speed:.0f} {secs:.1f}")
    odom.forward(speed, secs)
    time.sleep(secs + 0.3)
    return cm


def backup(cm: float = BUMP_BACKUP_CM):
    secs = cm / DRIVE_SPEED_CM_S
    send_cmd(f"back {DRIVE_SPEED_CM_S:.0f} {secs:.1f}")
    odom.forward(-DRIVE_SPEED_CM_S, secs)
    time.sleep(secs + 0.4)


# ── Navigation phases ─────────────────────────────────────────────────────────
def scan_for_door() -> float | None:
    """
    Rotate 360° in SCAN_STEPS. Baseline=0 (stationary between shots).
    At each step, also log all blocking obstacles so the operator can see
    the full scene map at each heading.
    Returns heading offset (°) when door found, 0.0 if in doorway, None if not found.
    """
    print("\n── SCAN ─────────────────────────────────────────")
    for i in range(SCAN_STEPS):
        heading_deg = i * SCAN_STEP_DEG
        print(f"  Step {i+1}/{SCAN_STEPS}  (~{heading_deg:.0f}°)")
        v = capture_and_analyse(baseline_cm=0.0)

        if v.get("in_doorway"):
            print("  Already in the doorway!")
            return 0.0

        if v.get("door_visible"):
            pos   = v.get("door_position") or "center"
            state = "open" if v.get("door_open") else "CLOSED"
            dist  = v.get("door_distance_cm")
            conf  = v.get("confidence", 0)
            offset = {"left": -SCAN_STEP_DEG / 3, "center": 0.0, "right": +SCAN_STEP_DEG / 3}[pos]
            print(
                f"  Door ({state}, {pos}, dist≈{dist}cm, conf={conf:.2f})  "
                f"offset={offset:+.0f}°"
            )
            if v.get("person_visible"):
                print("  (Person visible — door may open soon)")
            return offset

        # No door here — note any obstacles at this heading for situational awareness
        blocking = [d for d in v["obstacles"] if d["blocking"]]
        if blocking:
            obs_str = ", ".join(
                f"{d['class_name']}@{d['position']}~{d['distance_cm']}cm"
                for d in blocking
            )
            print(f"  Obstacles at {heading_deg:.0f}°: {obs_str}")

        turn(SCAN_STEP_DEG)

    print("  No door found in 360° scan.")
    return None


def wait_for_door_to_open() -> bool:
    """Poll camera (stationary, baseline=0) until door opens."""
    print("\n── WAITING for door to open ─────────────────────")
    for poll in range(1, MAX_WAIT_POLLS + 1):
        print(f"  Poll {poll}/{MAX_WAIT_POLLS}")
        v = capture_and_analyse(baseline_cm=0.0)
        if v.get("in_doorway") or v.get("door_open"):
            print("  Door is open!")
            return True
        if v.get("person_visible"):
            print("  Person visible — door may open soon")
        time.sleep(WAIT_POLL_S)
    print("  Timed out waiting.")
    return False


def approach_door() -> str:
    """
    Drive toward the door in chunks.

    Before each forward burst:
      1. Check the obstacle map for blocking objects in the center third.
      2. If center is blocked and obstacle is within AVOID_STEER_DIST_CM,
         steer toward the clear path before driving.

    Uses the actual driven distance as the optical-flow baseline to get
    door distance and stuck feedback.

    Returns: "arrived" | "lost" | "closed"
    """
    print("\n── APPROACH ─────────────────────────────────────")
    lost_count     = 0
    stuck_count    = 0
    prev_door_dist: float | None = None

    while True:
        if bumped():
            print("  Bump! Backing up and steering around.")
            backup()
            turn(45.0)
            lost_count = 0
            continue

        # ── Proactive obstacle check ──────────────────────────────────────
        # Use current vision state (from the last capture_and_analyse call)
        # before deciding speed/direction for this burst.
        blocked    = {}   # will be populated after first capture
        clear_path = None

        speed = DRIVE_SPEED_CM_S
        chunk = DRIVE_CHUNK_CM

        if prev_door_dist is not None:
            if prev_door_dist < SLOW_DIST_CM:
                speed = max(10.0, DRIVE_SPEED_CM_S * prev_door_dist / SLOW_DIST_CM)
                chunk = min(DRIVE_CHUNK_CM, prev_door_dist * 0.35)
                print(f"  Slow: speed={speed:.0f}cm/s chunk={chunk:.0f}cm (door≈{prev_door_dist:.0f}cm)")

        # Capture BEFORE driving to get the current obstacle map
        v_pre      = capture_and_analyse(baseline_cm=0.0)
        blocked    = v_pre.get("blocked", {})
        clear_path = v_pre.get("clear_path")
        nearest_cm = v_pre.get("nearest_cm")

        if blocked.get("center") and nearest_cm is not None and nearest_cm < AVOID_STEER_DIST_CM:
            deg = _choose_avoid_turn(blocked, clear_path)
            print(
                f"  Obstacle in center at ≈{nearest_cm:.0f}cm — "
                f"steering {deg:+.0f}° toward clear={clear_path}"
            )
            turn(deg)
            # Re-capture after turning so door detection is current
            v_pre      = capture_and_analyse(baseline_cm=0.0)
            blocked    = v_pre.get("blocked", {})
            clear_path = v_pre.get("clear_path")

        # Update door dist from pre-drive capture
        if v_pre.get("door_distance_cm") is not None:
            prev_door_dist = v_pre["door_distance_cm"]

        # ── Drive ─────────────────────────────────────────────────────────
        driven = drive_forward(chunk, speed)

        # Analyse post-drive with baseline = actual driven distance
        v = capture_and_analyse(baseline_cm=driven)
        prev_door_dist = v.get("door_distance_cm") or prev_door_dist

        # Stuck check
        if v.get("stuck"):
            stuck_count += 1
            print(f"  Stuck ({stuck_count}/{STUCK_RECOVERIES})")
            backup()
            turn(45.0)
            if stuck_count >= STUCK_RECOVERIES:
                send_cmd("stop")
                return "lost"
            continue
        stuck_count = 0

        # Arrival check
        if v.get("in_doorway"):
            print("  In doorway — stopping!")
            send_cmd("stop")
            return "arrived"
        if prev_door_dist is not None and prev_door_dist < ARRIVE_DIST_CM:
            print(f"  Door distance {prev_door_dist:.0f}cm < {ARRIVE_DIST_CM:.0f}cm — arrived!")
            send_cmd("stop")
            return "arrived"

        # Closed door
        if v.get("door_visible") and not v.get("door_open", True):
            print(f"  Door closed at ≈{prev_door_dist}cm — waiting.")
            send_cmd("stop")
            return "closed"

        # Lost door
        if not v.get("door_visible"):
            lost_count += 1
            print(f"  Door not visible ({lost_count}/3)")
            if lost_count >= 3:
                send_cmd("stop")
                return "lost"
            drive_forward(15.0)
            continue

        lost_count = 0
        pos = v.get("door_position") or "center"
        if pos == "left":
            print(f"  Steering left (door≈{prev_door_dist}cm)")
            turn(+STEER_DEG)
        elif pos == "right":
            print(f"  Steering right (door≈{prev_door_dist}cm)")
            turn(-STEER_DEG)


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    print("═══ Roomba door finder (YOLO obstacles + OpenCV + flow depth) ═══")

    status = send_cmd("full")
    if status.startswith("ERR"):
        print("Cannot reach pilot daemon — is it running?")
        sys.exit(1)
    print(f"Robot: {status}\n")

    if init_hailo():
        print("[hailo] YOLOv8 ready")
    else:
        print("[hailo] not available — OpenCV only")

    for round_num in range(1, MAX_SCAN_ROUNDS + 1):
        print(f"\n═══ Round {round_num}/{MAX_SCAN_ROUNDS} ═══")

        offset = scan_for_door()

        if offset is None:
            print("No door found. Moving to a new position.")
            drive_forward(80.0)
            continue

        if offset == 0.0:
            print("\n═══ Mission complete: already in doorway ═══")
            return

        if abs(offset) > 5:
            print(f"\nFine-tuning heading {offset:+.0f}°")
            turn(offset)

        outcome = approach_door()

        if outcome == "arrived":
            print("\n═══ Mission complete: stopped in doorway ═══")
            return

        if outcome == "closed":
            opened = wait_for_door_to_open()
            if opened:
                outcome2 = approach_door()
                if outcome2 == "arrived":
                    print("\n═══ Mission complete: stopped in doorway ═══")
                    return

        print("\nRescanning from current position.")

    print("\nCould not reach the door after all attempts. Stopping.")
    send_cmd("stop")


if __name__ == "__main__":
    main()
