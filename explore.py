#!/usr/bin/env python3
"""
explore.py — Concurrent Roomba room exploration for door finding.

Vision runs on the Raspberry Pi AI Hat+ (Hailo) using YOLOv8 for object
detection, plus OpenCV geometric analysis for door/doorway recognition and
optical-flow depth estimation between consecutive frames.

Before every forward drive the mover thread reads the current obstacle map
from shared state and steers toward the clearest visible path, falling back
to reactive bump-handling when proactive avoidance is insufficient.

Threads:
  mover  — movement state machine, updates OdometryTracker after each command
  camera — frame capture + YOLO obstacles + door detection + flow depth (every 2 s)
  logger — flushes state to JSON every second

STRATEGY FILE  /tmp/roomba_strategy.json
  {
    "mode":         "EXPLORE" | "APPROACH" | "WAIT" | "STOP",
    "door_bearing": <degrees from start, float> | null,
    "notes":        "free text"
  }
"""

import json
import math
import queue
import shutil
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from vision import (
    OdometryTracker,
    analyze_motion,
    analyze_obstacles,
    detect_door_cv,
    init_hailo,
    run_yolo,
    yolo_labels,
)

# ── Config ─────────────────────────────────────────────────────────────────
PILOT_HOST = "127.0.0.1"
PILOT_PORT = 9999

FRAME_DIR     = Path("/tmp/roomba_frames")
LOG_FILE      = Path("/tmp/roomba_log.txt")
STATE_FILE    = Path("/tmp/roomba_state.json")
STRATEGY_FILE = Path("/tmp/roomba_strategy.json")

FRAME_INTERVAL  = 2.0   # seconds between camera captures
MOVE_SPEED      = 35    # cm/s forward speed
MOVE_BURST      = 3.0   # seconds per forward burst
TURN_AFTER_BUMP = 90    # degrees to turn after bumping (always CW)

DOOR_CONFIRM_FRAMES = 2
APPROACH_SLOW_DIST  = 150  # cm — half-speed below this
APPROACH_STOP_DIST  = 40   # cm — stop and declare arrival

# Proactive avoidance: steer if an obstacle is detected closer than this
AVOID_STEER_DIST_CM = 120  # cm

# ── Shared state ────────────────────────────────────────────────────────────
_lock = threading.Lock()
_state: dict = {
    "mode":            "EXPLORE",
    "heading":         0.0,
    "pos_x":           0.0,
    "pos_y":           0.0,
    "bumps":           0,
    "frames_captured": 0,
    "last_detection":  [],     # list of detection dicts from run_yolo()
    "last_door":       None,
    "door_bearing":    None,
    "scene_depth_cm":  None,
    "stuck":           False,
    # Obstacle map — updated every frame by camera thread
    "blocked":         {"left": False, "center": False, "right": False},
    "clear_path":      None,   # "left" | "center" | "right" | None
    "nearest_cm":      None,   # distance to nearest blocking obstacle
    "log_lines":       0,
}

odom = OdometryTracker()


def state_get(key):
    with _lock:
        return _state[key]


def state_set(**kw):
    with _lock:
        _state.update(kw)


# ── Logging ─────────────────────────────────────────────────────────────────
LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
LOG_FILE.touch()
_log_q: queue.Queue = queue.Queue()


def log(msg: str):
    ts   = time.strftime("%H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line, flush=True)
    _log_q.put(line)


def _log_writer():
    with LOG_FILE.open("a") as f:
        while True:
            try:
                line = _log_q.get(timeout=1)
                f.write(line + "\n")
                f.flush()
                with _lock:
                    _state["log_lines"] += 1
            except queue.Empty:
                pass


# ── TTS ──────────────────────────────────────────────────────────────────────
_SPEAK_FILE = Path("/tmp/speak_queue.txt")


def speak(text: str):
    try:
        with _SPEAK_FILE.open("a") as f:
            f.write(text + "\n")
    except Exception:
        pass


# ── Pilot TCP ────────────────────────────────────────────────────────────────
def send_cmd(cmd: str, timeout: float = 5.0) -> str:
    try:
        with socket.create_connection((PILOT_HOST, PILOT_PORT), timeout=timeout) as s:
            s.sendall((cmd + "\n").encode())
            s.settimeout(timeout)
            return s.makefile().readline().strip()
    except Exception as e:
        return f"ERR {e}"


# ── Camera ───────────────────────────────────────────────────────────────────
FRAME_DIR.mkdir(parents=True, exist_ok=True)
_CURRENT_FRAME = Path("/tmp/roomba_current.jpg")


def capture_frame(path: Path) -> bool:
    try:
        r = subprocess.run(
            ["rpicam-still", "--nopreview",
             "--width", "640", "--height", "480",
             "--rotation", "180",
             "-o", str(path), "-t", "500"],
            capture_output=True, timeout=5,
        )
        return r.returncode == 0
    except Exception as e:
        log(f"capture error: {e}")
        return False


# ── Strategy file ─────────────────────────────────────────────────────────────
def _read_strategy():
    try:
        if STRATEGY_FILE.exists():
            data = json.loads(STRATEGY_FILE.read_text())
            mode = data.get("mode")
            if mode and mode != state_get("mode"):
                log(f"[STRATEGY] mode override → {mode}")
                speak(f"Strategy update. Switching to {mode}.")
                notes = data.get("notes", "")
                if notes:
                    log(f"[STRATEGY] {notes}")
                state_set(mode=mode)
                if data.get("door_bearing") is not None:
                    state_set(door_bearing=float(data["door_bearing"]))
    except Exception as e:
        log(f"strategy read error: {e}")


# ── Camera + vision thread ───────────────────────────────────────────────────
def camera_thread():
    frame_num          = 0
    door_confirm_count = 0
    prev_img: np.ndarray | None = None
    prev_odom: tuple[float, float, float] | None = None

    while state_get("mode") != "STOP":
        time.sleep(FRAME_INTERVAL)
        frame_num += 1
        path = FRAME_DIR / f"frame_{frame_num:05d}.jpg"

        if not capture_frame(path):
            continue

        try:
            shutil.copy(str(path), str(_CURRENT_FRAME))
        except Exception:
            pass

        for old in sorted(FRAME_DIR.glob("frame_*.jpg"))[:-20]:
            old.unlink(missing_ok=True)

        state_set(frames_captured=frame_num)

        curr_img = cv2.imread(str(path))
        if curr_img is None:
            continue

        curr_odom = odom.snapshot()
        frame_h, frame_w = curr_img.shape[:2]

        # ── Optical-flow depth / stuck detection ─────────────────────────
        scene_depth: float | None = None
        if prev_img is not None and prev_odom is not None:
            dx = curr_odom[0] - prev_odom[0]
            dy = curr_odom[1] - prev_odom[1]
            baseline_cm = math.hypot(dx, dy)
            motion      = analyze_motion(prev_img, curr_img, baseline_cm)
            scene_depth = motion.get("scene_depth_cm")
            stuck       = motion.get("stuck", False)
            flow        = motion.get("flow_px", 0.0)
            state_set(scene_depth_cm=scene_depth, stuck=stuck)
            log(
                f"[FLOW] frame {frame_num}: baseline={baseline_cm:.1f}cm "
                f"flow={flow:.1f}px depth≈{scene_depth}cm stuck={stuck}"
            )
            if stuck and state_get("mode") in ("EXPLORE", "APPROACH"):
                speak("Robot appears stuck. Changing direction.")

        prev_img  = curr_img
        prev_odom = curr_odom

        # ── YOLO obstacle detection ───────────────────────────────────────
        detections = run_yolo(curr_img)
        obs_map    = analyze_obstacles(detections, frame_w, frame_h, scene_depth)

        state_set(
            last_detection=detections,
            blocked=obs_map["blocked"],
            clear_path=obs_map["clear_path"],
            nearest_cm=obs_map["nearest_cm"],
        )

        if detections:
            labels    = yolo_labels(detections)
            blocking  = [d for d in obs_map["obstacles"] if d["blocking"]]
            block_str = ", ".join(
                f"{d['class_name']}@{d['position']}~{d['distance_cm']}cm"
                for d in blocking
            ) or "none"
            log(
                f"[YOLO] frame {frame_num}: detected=[{', '.join(labels)}] "
                f"blocking=[{block_str}] "
                f"blocked={obs_map['blocked']} clear={obs_map['clear_path']}"
            )
            if "person" in labels:
                log("[YOLO] person visible — door may open soon")
                speak("Person detected. Watching for door.")

        # ── Door detection ────────────────────────────────────────────────
        door = detect_door_cv(curr_img)
        state_set(last_door=door)

        if door["door_visible"]:
            dist = door.get("door_distance_cm")
            log(
                f"[DOOR] frame {frame_num}: {door['notes']} "
                f"dist≈{dist}cm conf={door['confidence']:.2f}"
            )
            door_confirm_count += 1
        else:
            door_confirm_count = 0

        if door_confirm_count >= DOOR_CONFIRM_FRAMES:
            mode = state_get("mode")
            dist = door.get("door_distance_cm") or 9999

            if door["in_doorway"] or dist < APPROACH_STOP_DIST:
                if mode != "STOP":
                    log("[VISION] In doorway — stopping!")
                    speak("I am in the doorway. Mission complete.")
                    state_set(mode="STOP")
                    door_confirm_count = 0

            elif door["door_open"] and mode not in ("APPROACH", "STOP", "WAIT"):
                pos     = door.get("door_position", "center")
                heading = odom.heading
                offset  = {"left": 30, "center": 0, "right": -30}.get(pos, 0)
                bearing = (heading + offset) % 360
                log(
                    f"[VISION] Open door on {pos} at ≈{dist:.0f}cm "
                    f"→ APPROACH bearing={bearing:.0f}°"
                )
                speak(f"Door on the {pos}, about {int(dist)} centimetres. Approaching.")
                state_set(mode="APPROACH", door_bearing=bearing)
                door_confirm_count = 0

            elif not door["door_open"] and mode == "APPROACH":
                log(f"[VISION] Door closed at ≈{dist:.0f}cm — WAIT")
                speak("Door is closed. Waiting nearby.")
                state_set(mode="WAIT")
                door_confirm_count = 0


# ── Movement thread ──────────────────────────────────────────────────────────
def mover_thread():
    moves_since_full   = 0
    _approach_bumps    = 0
    _consecutive_stuck = 0

    def do_forward(speed: int, secs: float) -> str:
        r = send_cmd(f"forward {speed} {secs:.1f}")
        odom.forward(speed, secs)
        state_set(pos_x=odom.x, pos_y=odom.y)
        time.sleep(secs + 0.1)
        return r

    def do_back(speed: int = MOVE_SPEED, secs: float = 0.8) -> str:
        r = send_cmd(f"back {speed} {secs:.1f}")
        odom.forward(-speed, secs)
        state_set(pos_x=odom.x, pos_y=odom.y)
        time.sleep(secs + 0.1)
        return r

    def do_turn(deg: float):
        send_cmd(f"turn {deg:.0f}")
        odom.turn(deg)
        state_set(heading=odom.heading)
        time.sleep(abs(deg) / 60.0 + 0.3)

    def bumped() -> tuple[bool, bool]:
        r = send_cmd("bumps")
        return "bumpL=1" in r, "bumpR=1" in r

    def choose_avoid_turn() -> float:
        """
        Decide which direction to steer based on which thirds are clear.
        Returns degrees to turn (+CCW, −CW).
        """
        blocked    = state_get("blocked") or {}
        clear_path = state_get("clear_path")
        if clear_path == "right":
            return -40.0   # CW toward right
        if clear_path == "left":
            return 40.0    # CCW toward left
        # All thirds blocked — turn 90° CW (consistent with wall-following)
        return -90.0

    speak("Beginning room exploration.")
    log("[MOVER] starting pos=(0,0) heading=0°")

    while True:
        _read_strategy()
        mode = state_get("mode")

        moves_since_full += 1
        if moves_since_full >= 20:
            send_cmd("full")
            moves_since_full = 0

        if mode == "STOP":
            send_cmd("stop")
            log("[MOVER] stopped")
            speak("Stopping.")
            break

        if mode != "APPROACH":
            _approach_bumps = 0

        if mode == "WAIT":
            time.sleep(1.0)
            continue

        # ── Stuck recovery ────────────────────────────────────────────────
        if state_get("stuck"):
            _consecutive_stuck += 1
            log(f"[MOVER] stuck ({_consecutive_stuck}) — backing + large turn")
            do_back(MOVE_SPEED, 1.2)
            do_turn(-120 if _consecutive_stuck % 2 == 0 else 120)
            state_set(stuck=False)
            if _consecutive_stuck >= 3:
                _consecutive_stuck = 0
                state_set(mode="EXPLORE")
            continue
        else:
            _consecutive_stuck = 0

        # ── APPROACH mode ─────────────────────────────────────────────────
        if mode == "APPROACH":
            door_bearing = state_get("door_bearing")
            if door_bearing is None:
                state_set(mode="EXPLORE")
                continue

            heading = odom.heading
            delta   = (door_bearing - heading + 180) % 360 - 180
            if abs(delta) > 10:
                log(f"[MOVER] APPROACH: turning {delta:+.0f}° toward bearing {door_bearing:.0f}°")
                do_turn(delta)

            last_door = state_get("last_door") or {}
            door_dist = last_door.get("door_distance_cm") or 9999
            if door_dist < APPROACH_SLOW_DIST:
                speed = max(15, int(MOVE_SPEED * door_dist / APPROACH_SLOW_DIST))
                burst = 1.5
                log(f"[MOVER] APPROACH: slow ({speed}cm/s), door≈{door_dist:.0f}cm")
            else:
                speed = MOVE_SPEED
                burst = MOVE_BURST

            # Proactive avoidance: if center is blocked on approach, steer around
            blocked    = state_get("blocked") or {}
            nearest_cm = state_get("nearest_cm")
            if blocked.get("center") and nearest_cm and nearest_cm < AVOID_STEER_DIST_CM:
                deg = choose_avoid_turn()
                log(
                    f"[MOVER] APPROACH: obstacle in center at ≈{nearest_cm:.0f}cm "
                    f"→ steering {deg:+.0f}°"
                )
                speak(f"Obstacle ahead. Steering around.")
                do_turn(deg)

            do_forward(speed, burst)
            bl, br = bumped()
            if bl or br:
                _approach_bumps += 1
                log(f"[MOVER] APPROACH: bump ({_approach_bumps}/5)")
                do_back(MOVE_SPEED, 0.8)
                do_turn(45 if br else -45)
                if _approach_bumps >= 5:
                    log("[MOVER] APPROACH: too many bumps — EXPLORE")
                    speak("Too many bumps. Reassessing.")
                    state_set(mode="EXPLORE")
                    _approach_bumps = 0

        # ── EXPLORE mode ──────────────────────────────────────────────────
        else:
            blocked    = state_get("blocked") or {}
            nearest_cm = state_get("nearest_cm")
            clear_path = state_get("clear_path")

            x, y = odom.x, odom.y
            log(
                f"[MOVER] EXPLORE heading={odom.heading:.0f}° pos=({x:.0f},{y:.0f})cm "
                f"blocked={blocked} nearest≈{nearest_cm}cm clear={clear_path}"
            )

            # Proactive avoidance: steer before driving if center is blocked
            if blocked.get("center") and nearest_cm and nearest_cm < AVOID_STEER_DIST_CM:
                deg = choose_avoid_turn()
                obstacle_str = f"≈{nearest_cm:.0f}cm"
                log(
                    f"[MOVER] proactive avoid: obstacle in center at {obstacle_str} "
                    f"→ clear_path={clear_path}, turning {deg:+.0f}°"
                )
                speak(f"Obstacle ahead at {int(nearest_cm)} centimetres. Steering {('right' if deg < 0 else 'left')}.")
                do_turn(deg)
                # Don't skip the forward drive; after turning, center should be clear
            elif blocked.get("center") and (nearest_cm is None or nearest_cm >= AVOID_STEER_DIST_CM):
                # Obstacle visible but far enough — just log it
                log(f"[MOVER] obstacle in center but distant (≈{nearest_cm}cm) — continuing")

            do_forward(MOVE_SPEED, MOVE_BURST)

            bl, br = bumped()
            if bl or br:
                side = "left" if bl else "right"
                log(f"[MOVER] bump {side} at heading={odom.heading:.0f}°")
                speak("Bump. Turning right.")
                state_set(bumps=state_get("bumps") + 1)
                do_back(MOVE_SPEED, 0.5)
                do_turn(-TURN_AFTER_BUMP)   # always CW for consistent wall-following


# ── State writer thread ───────────────────────────────────────────────────────
def state_writer_thread():
    while state_get("mode") != "STOP":
        try:
            with _lock:
                snap = dict(_state)
            STATE_FILE.write_text(json.dumps(snap, indent=2))
        except Exception:
            pass
        time.sleep(1.0)


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    log("=" * 60)
    log("explore.py — YOLO obstacle avoidance + OpenCV + flow depth")
    log(f"Strategy file: {STRATEGY_FILE}")
    log(f"Frames:        {FRAME_DIR}")
    log(f"Log:           {LOG_FILE}")
    log("=" * 60)
    speak("Explore script starting. Hailo YOLO obstacle avoidance active.")

    r = send_cmd("full")
    log(f"pilot full: {r}")
    r = send_cmd("sense")
    log(f"pilot sense: {r}")
    if r.startswith("ERR"):
        log("Cannot reach pilot daemon — is it running?")
        sys.exit(1)

    if not STRATEGY_FILE.exists():
        STRATEGY_FILE.write_text(json.dumps({
            "mode": "EXPLORE", "door_bearing": None, "notes": "initial"
        }, indent=2))

    if init_hailo():
        log("Hailo ready")
    else:
        log("Hailo not available — running OpenCV only")

    threads = [
        threading.Thread(target=_log_writer,         daemon=True, name="log"),
        threading.Thread(target=state_writer_thread,  daemon=True, name="state"),
        threading.Thread(target=camera_thread,        daemon=True, name="camera"),
        threading.Thread(target=mover_thread,                      name="mover"),
    ]
    for t in threads:
        t.start()

    threads[-1].join()
    log("explore.py done.")
    speak("Exploration complete.")


if __name__ == "__main__":
    main()
