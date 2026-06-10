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

import argparse
import collections
import json
import math
import multiprocessing as mp
import queue
import random
import shutil
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from map_maker import MapRecorder
from model_setup import ensure_models
from vision import (
    OdometryTracker,
    yolo_labels,
)

# ── Config ─────────────────────────────────────────────────────────────────
PILOT_HOST = "127.0.0.1"
PILOT_PORT = 9999

FRAME_DIR     = Path("/tmp/roomba_frames")
LOG_FILE      = Path("/tmp/roomba_log.txt")
STATE_FILE    = Path("/tmp/roomba_state.json")
STRATEGY_FILE = Path("/tmp/roomba_strategy.json")

FRAME_INTERVAL  = 0.5   # seconds between camera captures
MIN_MOVE_CM     = 2.0   # minimum translation since last photo — skip if less
MIN_TURN_DEG    = 10.0  # minimum heading change since last photo — skip if less
MOVE_SPEED      = 15    # cm/s forward speed
MOVE_BURST      = 3.0   # seconds per forward burst
SCAN_ROCK_CM    = 20    # forward distance (cm) rocked at each scan heading for flow depth
SCAN_WAIT_S     = 2.0   # seconds to wait at each scan heading for a fresh frame
                         # = FRAME_INTERVAL + capture(0.8s) + analysis(0.5s) + margin

DOOR_CONFIRM_FRAMES = 3    # positives needed to trigger approach
DOOR_CONFIRM_WINDOW = 7    # sliding window length in frames (~14 s at 2 s/frame)
DOOR_CONFIRM_MIN_CONF = 0.10   # ignore detections below this confidence
DOOR_FASTTRACK_CONF = 0.75     # single open-door detection above this → APPROACH immediately
DOOR_SLOW_BURST     = 1.0      # forward burst (s) when door hit in current window

CORNER_PROGRESS_CM  = 35   # min displacement (cm) between bumps to reset corner counter
CORNER_ESCAPE_BUMPS = 3    # consecutive stuck bumps before executing corner escape
SCAN_EVERY_BUMPS = 3           # pause for a 360° scan every N bumps
SCAN_EVERY_CM    = 300         # also scan every N cm of odometry travel
MAP_SAVE_INTERVAL = 30         # seconds between periodic map saves
APPROACH_SLOW_DIST  = 150  # cm — half-speed below this
APPROACH_STOP_DIST  = 40   # cm — stop and declare arrival

# Proactive avoidance: steer if an obstacle is detected closer than this
AVOID_STEER_DIST_CM = 120  # cm
# Map-based avoidance: steer if a known map obstacle is closer than this
MAP_AVOID_DIST_CM   = 150  # cm  (generous — odometry drifts so cone is wide)

# Depth-based proactive steering thresholds.
# Center open_space below DEPTH_CENTER_BLOCK → steer before moving.
# A side must exceed the other by DEPTH_SIDE_MARGIN to prefer it over CW default.
DEPTH_CENTER_BLOCK  = 0.40   # fast_depth open_space fraction; <this = likely blocked
DEPTH_SIDE_MARGIN   = 0.12   # minimum advantage for a side to be preferred

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
    "legs_blocking":   False,  # True when detect_thin_legs reports a near leg
    "open_space":      {"left": 0.0, "center": 0.0, "right": 0.0},  # depth-map far fractions
    "log_lines":       0,
}

odom         = OdometryTracker()
map_recorder = MapRecorder()

# Set while vision is idle, cleared while a frame is being analysed.
# Mover thread waits on this before each movement burst so the robot
# never moves without up-to-date scene information.
_vision_idle = threading.Event()
_vision_idle.set()  # no analysis in progress at startup

# Set by the camera thread after a photo taken while the robot was stationary
# has been fully analysed.  Cleared by do_turn/do_forward when motion begins
# (heading or position will change, making the current photo stale).
# do_forward() waits on this before every drive burst.
_rest_photo_ready = threading.Event()


# ── Vision child-process worker ───────────────────────────────────────────────
# Must be a module-level function (not nested) so multiprocessing can pickle it.

def _vision_worker(req_q: mp.Queue, res_q: mp.Queue) -> None:
    """
    Child process: load all vision models once, then serve frames indefinitely.

    Protocol:
      startup  → res_q.put({"__ready__": True, "available": {...}})
               | res_q.put({"__error__": "..."})  on init failure
      per frame: req_q.get() → (curr_img, prev_img | None, baseline_cm)
               → res_q.put(scene_dict)
               | res_q.put({"__error__": "..."})
      shutdown: req_q.put(None) → process exits cleanly
    """
    from vision import MODELS_AVAILABLE, analyze_scene, init_all_models
    try:
        init_all_models()
    except Exception as e:
        res_q.put({"__error__": f"model init: {e}"})
        return
    res_q.put({"__ready__": True, "available": dict(MODELS_AVAILABLE)})

    while True:
        item = req_q.get()
        if item is None:
            break
        curr_img, prev_img, baseline_cm = item
        try:
            scene = analyze_scene(curr_img, prev_img=prev_img, baseline_cm=baseline_cm)
            res_q.put(scene)
        except Exception as e:
            res_q.put({"__error__": str(e)})


class VisionProcess:
    """
    Wraps analyze_scene() in a child process so a hung Hailo inference can be
    truly killed (SIGKILL) rather than merely timed-out at the thread level.

    On a 3 s timeout the child is SIGKILLed and a fresh process is spawned;
    vision models reload in the new child (~5–15 s) before the next result
    is returned.  Frames captured during respawn are skipped but the camera
    loop keeps ticking uninterrupted.
    """
    INFER_TIMEOUT  = 3.0    # seconds per frame before declaring a hang
    READY_TIMEOUT  = 60.0   # seconds to wait for model load on (re)start

    def __init__(self) -> None:
        self._proc: mp.Process | None = None
        self._req:  mp.Queue   | None = None
        self._res:  mp.Queue   | None = None
        self._spawn()

    def _spawn(self) -> None:
        self._req = mp.Queue(maxsize=1)
        self._res = mp.Queue(maxsize=1)
        self._proc = mp.Process(
            target=_vision_worker,
            args=(self._req, self._res),
            daemon=True,
            name="vision-worker",
        )
        self._proc.start()
        # log() is not available yet at module-init time; print is fine here.
        print(f"[VISION] worker spawned PID={self._proc.pid} — waiting for models…",
              flush=True)
        try:
            msg = self._res.get(timeout=self.READY_TIMEOUT)
        except Exception:
            print("[VISION] worker did not become ready in time", flush=True)
            return
        if msg.get("__ready__"):
            avail   = msg.get("available", {})
            loaded  = [k for k, v in avail.items() if v]
            missing = [k for k, v in avail.items() if not v]
            print(f"[VISION] worker ready  loaded={loaded}  missing={missing}", flush=True)
        else:
            print(f"[VISION] worker init error: {msg.get('__error__')}", flush=True)

    def _kill_and_respawn(self) -> None:
        try:
            if self._proc and self._proc.is_alive():
                self._proc.kill()
                self._proc.join(timeout=2.0)
        except Exception:
            pass
        self._spawn()

    def analyze(
        self,
        curr_img: np.ndarray,
        prev_img: "np.ndarray | None",
        baseline_cm: float,
    ) -> "dict | None":
        if not (self._proc and self._proc.is_alive()):
            log("[VISION] worker died — respawning")
            speak("Vision worker crashed. Restarting.")
            self._spawn()

        try:
            self._req.put_nowait((curr_img, prev_img, baseline_cm))
        except Exception:
            log("[VISION] request queue full — worker still busy, skipping frame")
            return None

        try:
            result = self._res.get(timeout=self.INFER_TIMEOUT)
        except Exception:
            log(f"[VISION] {self.INFER_TIMEOUT:.0f}s timeout — killing worker and respawning")
            speak("Vision timeout. Restarting vision worker.")
            self._kill_and_respawn()
            return None

        if "__error__" in result:
            log(f"[VISION] worker error: {result['__error__']}")
            return None
        return result


_vision: VisionProcess | None = None

# ── Human greeting ────────────────────────────────────────────────────────────
_greet_humans     = False          # set to True by --greet flag
_GREET_PHRASES    = [
    ("Hello human, I come in peace",    0.80),
    ("Human you are, seek peace we must", 0.20),
]
_GREET_COOLDOWN_S = 30.0           # minimum seconds between greetings
_last_greeted_at  = 0.0            # monotonic timestamp of last greeting


def _maybe_greet():
    """Speak a greeting if --greet is active and the cooldown has elapsed."""
    global _last_greeted_at
    if not _greet_humans:
        return
    now = time.monotonic()
    if now - _last_greeted_at < _GREET_COOLDOWN_S:
        return
    _last_greeted_at = now
    # Weighted random choice (80 / 20)
    phrase = random.choices(
        [p for p, _ in _GREET_PHRASES],
        weights=[w for _, w in _GREET_PHRASES],
        k=1,
    )[0]
    log(f"[GREET] {phrase}")
    speak(phrase)


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
_TMP_FRAME     = Path("/tmp/roomba_current.tmp.jpg")


def capture_frame(path: Path) -> bool:
    try:
        r = subprocess.run(
            ["rpicam-still", "--nopreview",
             "--width", "640", "--height", "480",
             "-o", str(path), "-t", "500"],
            capture_output=True, timeout=5,
        )
        if r.returncode != 0:
            return False
        # rpicam-still can return 0 before the ISP pipeline finishes flushing
        # the file.  A valid 640×480 JPEG is always several kilobytes; anything
        # smaller means the write was not complete.
        try:
            size = path.stat().st_size
        except OSError:
            size = 0
        if size < 2048:
            log(f"[CAM] ignoring truncated capture ({size} B) at {path.name}")
            return False
        return True
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
    frame_num        = 0
    door_history: collections.deque[bool] = collections.deque(maxlen=DOOR_CONFIRM_WINDOW)
    prev_img:  np.ndarray | None              = None
    prev_odom: tuple[float, float, float] | None = None

    while state_get("mode") != "STOP":
        time.sleep(FRAME_INTERVAL)

        # Skip capture if the robot hasn't moved or turned since the last photo,
        # BUT only when a rest photo isn't still needed.  If _rest_photo_ready is
        # not set, the mover is waiting for a stationary photo — don't skip.
        if prev_odom is not None and _rest_photo_ready.is_set():
            pre = odom.snapshot()
            d_pos = math.hypot(pre[0] - prev_odom[0], pre[1] - prev_odom[1])
            d_hdg = abs((pre[2] - prev_odom[2] + 180) % 360 - 180)
            if d_pos < MIN_MOVE_CM and d_hdg < MIN_TURN_DEG:
                continue

        frame_num += 1
        path = FRAME_DIR / f"frame_{frame_num:05d}.jpg"

        if not capture_frame(path):
            continue

        # Write to a temp file then rename atomically so that rsync / the Read
        # tool never sees a half-written JPEG when they sample _CURRENT_FRAME.
        try:
            shutil.copy(str(path), str(_TMP_FRAME))
            _TMP_FRAME.replace(_CURRENT_FRAME)
        except Exception:
            pass

        # Delete frames older than 20 behind the CURRENT frame_num so that
        # stale high-numbered frames from a previous run never cause the
        # current frame to be deleted.
        for old in FRAME_DIR.glob("frame_*.jpg"):
            try:
                if int(old.stem.split("_")[-1]) < frame_num - 20:
                    old.unlink(missing_ok=True)
            except (ValueError, IndexError):
                pass

        state_set(frames_captured=frame_num)

        curr_img = cv2.imread(str(path))
        if curr_img is None:
            continue

        curr_odom = odom.snapshot()

        # Odometry baseline for optical flow depth
        baseline_cm = 0.0
        if prev_odom is not None:
            baseline_cm = math.hypot(
                curr_odom[0] - prev_odom[0],
                curr_odom[1] - prev_odom[1],
            )

        # ── Vision inference in child process (3 s timeout; kill on hang) ──
        # Clear the event so the mover pauses until analysis is done.
        _vision_idle.clear()
        try:
            scene = _vision.analyze(curr_img, prev_img, baseline_cm)
        finally:
            _vision_idle.set()
            # Photo was taken while stationary — signal that the mover can drive
            if baseline_cm < MIN_MOVE_CM:
                _rest_photo_ready.set()
        if scene is None:
            prev_img  = curr_img    # keep prev_img current for next frame's flow
            prev_odom = curr_odom
            continue

        prev_img  = curr_img
        prev_odom = curr_odom

        # ── Optical flow / stuck ──────────────────────────────────────────
        scene_depth = scene["scene_depth_cm"]
        stuck       = scene["stuck"]
        state_set(scene_depth_cm=scene_depth, stuck=stuck)
        if baseline_cm > 0:
            log(
                f"[FLOW] frame {frame_num}: baseline={baseline_cm:.1f}cm "
                f"depth≈{scene_depth}cm stuck={stuck}"
            )
            if stuck and state_get("mode") in ("EXPLORE", "APPROACH"):
                speak("Robot appears stuck. Changing direction.")

        # ── Depth map (fast_depth) ───────────────────────────────────────
        if scene["depth_map"] is not None:
            os_ = scene["open_space"]
            log(
                f"[DEPTH] frame {frame_num}: open_space "
                f"L={os_['left']:.2f} C={os_['center']:.2f} R={os_['right']:.2f}"
            )
        state_set(open_space=scene["open_space"])

        # ── YOLO detections ───────────────────────────────────────────────
        detections = scene["detections"]
        obs_map    = scene["obstacles"]
        state_set(
            last_detection=detections,
            blocked=obs_map["blocked"],
            clear_path=obs_map["clear_path"],
            nearest_cm=obs_map["nearest_cm"],
            legs_blocking=scene.get("legs_blocking", False),
        )

        if detections:
            labels   = yolo_labels(detections)
            blocking = [d for d in obs_map["obstacles"] if d["blocking"]]
            block_str = ", ".join(
                f"{d['class_name']}@{d['position']}~{d['distance_cm']}cm"
                f"(h≈{d.get('real_height_cm')}cm)"
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
                _maybe_greet()

            rx, ry, rh = odom.x, odom.y, odom.heading
            for det in blocking:
                map_recorder.record_yolo(rx, ry, rh,
                                         det["class_name"], det["position"],
                                         det.get("distance_cm"))

        # ── Door detection (fused) ────────────────────────────────────────
        yolo_doors = scene.get("yolo_doors", [])
        if yolo_doors:
            log(
                f"[DOOR-YOLO] frame {frame_num}: {len(yolo_doors)} panel(s) — "
                + ", ".join(f"{d['position']}@{d['confidence']:.2f}" for d in yolo_doors)
            )

        door = scene["door"]
        state_set(last_door=door)

        detected = door["door_visible"] and door.get("confidence", 0) >= DOOR_CONFIRM_MIN_CONF
        door_history.append(detected)

        prev_detected = door_history[-2] if len(door_history) >= 2 else False
        if detected and not prev_detected:
            speak("Door in sight.")

        if detected:
            map_recorder.record_door(
                odom.x, odom.y, odom.heading,
                door.get("door_position") or "center",
                door.get("door_distance_cm"),
                door.get("door_open", False),
            )
            dist = door.get("door_distance_cm")
            log(
                f"[DOOR] frame {frame_num}: {door['notes']} "
                f"dist≈{dist}cm conf={door['confidence']:.2f} "
                f"hits={sum(door_history)}/{len(door_history)}"
            )

        # Fast-track: one very high-confidence open-door hit → APPROACH immediately
        # without waiting for the sliding window.  Handles cases like frame 47
        # (conf=0.94) where the robot moves away before accumulating 3 hits.
        if (detected
                and door.get("door_open")
                and door.get("confidence", 0) >= DOOR_FASTTRACK_CONF
                and state_get("mode") not in ("APPROACH", "STOP", "WAIT")):
            pos     = door.get("door_position", "center")
            offset  = {"left": 30, "center": 0, "right": -30}.get(pos, 0)
            bearing = (odom.heading + offset) % 360
            dist_ft = door.get("door_distance_cm") or 9999
            log(
                f"[VISION] FAST-TRACK conf={door['confidence']:.2f} open door "
                f"{pos} at ≈{dist_ft:.0f}cm → APPROACH {bearing:.0f}°"
            )
            speak(f"High confidence door on the {pos}. Approaching now.")
            state_set(mode="APPROACH", door_bearing=bearing)
            door_history.clear()

        if sum(door_history) >= DOOR_CONFIRM_FRAMES:
            mode = state_get("mode")
            dist = door.get("door_distance_cm") or 9999

            if door["in_doorway"] or dist < APPROACH_STOP_DIST:
                if mode != "STOP":
                    log("[VISION] In doorway — stopping!")
                    speak("I am in the doorway. Mission complete.")
                    state_set(mode="STOP")
                    door_history.clear()

            elif door["door_open"] and mode == "APPROACH":
                pos    = door.get("door_position", "center")
                offset = {"left": 25, "center": 0, "right": -25}.get(pos, 0)
                if abs(offset) > 0:
                    new_bearing = (odom.heading + offset) % 360
                    log(f"[VISION] APPROACH: door on {pos}, correcting → {new_bearing:.0f}°")
                    state_set(door_bearing=new_bearing)

            elif door["door_open"] and mode not in ("APPROACH", "STOP", "WAIT"):
                pos     = door.get("door_position", "center")
                offset  = {"left": 30, "center": 0, "right": -30}.get(pos, 0)
                bearing = (odom.heading + offset) % 360
                log(f"[VISION] Open door on {pos} at ≈{dist:.0f}cm → APPROACH bearing={bearing:.0f}°")
                speak(f"Door on the {pos}, about {int(dist)} centimetres. Approaching.")
                state_set(mode="APPROACH", door_bearing=bearing)
                door_history.clear()

            elif not door["door_open"] and mode == "APPROACH":
                log(f"[VISION] Door closed at ≈{dist:.0f}cm — WAIT")
                speak("Door is closed. Waiting nearby.")
                state_set(mode="WAIT")
                door_history.clear()


# ── Movement thread ──────────────────────────────────────────────────────────
def mover_thread():
    moves_since_full   = 0
    _approach_bumps    = 0
    _consecutive_stuck = 0
    _bumps_since_scan  = 0
    _last_scan_x       = 0.0   # odometry position at the last 360° scan
    _last_scan_y       = 0.0
    # Corner escape state
    _default_turn_sign = -1    # -1=CW, +1=CCW; flips each time it's used with no depth preference
    _corner_bump_count = 0     # bumps without CORNER_PROGRESS_CM of displacement
    _corner_ref_x      = 0.0  # odometry position when the current bump streak began
    _corner_ref_y      = 0.0

    _FORWARD_SEG_S = 0.3   # seconds per forward segment for bump-interruptible driving

    def do_forward(speed: int, secs: float) -> tuple[str, float]:
        """
        Drive forward in 0.3-second segments, stopping the instant a bumper fires.

        Odometry is updated only for segments that completed without a bump.
        The segment during which the bump occurred is NOT counted — the robot
        was physically stopped by the obstacle part-way through it, so the
        commanded distance for that segment is unreliable.

        Returns (status, actual_secs_traveled).
          status: "ok" | "bump_L" | "bump_R" | "bump_LR"
        """
        # Never drive without a photo analysed at the current stopped position.
        if not _rest_photo_ready.wait(timeout=10.0):
            log("[MOVER] WARNING: rest photo timeout — proceeding anyway")
        _rest_photo_ready.clear()   # position/heading will change during drive

        elapsed = 0.0
        bump_side = ""
        while elapsed < secs:
            if state_get("mode") == "STOP":
                break
            # Mid-burst leg check: stop if leg geometry (not YOLO) has flagged
            # the center blocked since this burst started.  Using the dedicated
            # legs_blocking flag avoids a false negative when YOLO sees an
            # unrelated object (nearest_cm non-None) while legs are the real hazard.
            if state_get("legs_blocking") and (state_get("blocked") or {}).get("center"):
                send_cmd("stop")
                log(f"[MOVER] mid-burst stop: legs blocking center after {elapsed:.1f}s")
                return "blocked_legs", elapsed
            seg = min(_FORWARD_SEG_S, secs - elapsed)
            send_cmd(f"forward {speed} {seg:.2f}")
            time.sleep(seg + 0.05)

            r = send_cmd("bumps")
            bl, br = "bumpL=1" in r, "bumpR=1" in r
            if bl or br:
                send_cmd("stop")
                # Do NOT count this segment — distance during a bump is unknown.
                # Odometry already reflects all prior clean segments.
                bump_side = ("LR" if bl and br else "L" if bl else "R")
                log(f"[MOVER] bumper {bump_side} after {elapsed:.1f}s "
                    f"(~{elapsed*speed:.0f} cm)")
                map_recorder.record_bump(odom.x, odom.y, odom.heading)
                state_set(pos_x=odom.x, pos_y=odom.y)
                return f"bump_{bump_side}", elapsed
            # Segment clean — commit to odometry
            odom.forward(speed, seg)
            elapsed += seg

        state_set(pos_x=odom.x, pos_y=odom.y)
        map_recorder.record_position(odom.x, odom.y, odom.heading)
        return "ok", elapsed

    def do_turn(deg: float):
        _rest_photo_ready.clear()   # heading will change — existing photo is stale
        send_cmd(f"turn {deg:.0f}")
        odom.turn(deg)
        state_set(heading=odom.heading)
        time.sleep(abs(deg) / 60.0 + 0.3)

    def do_reverse_safe(dist_cm: float, skip_check: bool = False) -> bool:
        """
        Move 'backward' safely — sensors always face the direction of travel.

        Procedure:
          1. Rotate 180° to face the target direction.
          2. Unless skip_check=True, wait for a camera frame and verify the
             path is clear (nearest obstacle > dist_cm + 20 cm).
          3. Drive forward dist_cm using interruptible do_forward().
          4. Rotate 180° back to restore original heading.

        skip_check=True is appropriate when we know the path is clear
        (e.g. post-bump retreat — we were just in that space) and speed
        matters more than the extra safety wait.

        Returns True if the forward leg completed without a bump.
        """
        do_turn(180)

        if not skip_check:
            time.sleep(SCAN_WAIT_S)   # let camera capture at new heading
            nearest  = state_get("nearest_cm") or 9999
            blocked_c = (state_get("blocked") or {}).get("center", False)
            if blocked_c and nearest < dist_cm + 20:
                log(f"[MOVER] reverse path blocked ({nearest:.0f} cm) — skipping back-move")
                speak("Reverse path blocked. Staying put.")
                do_turn(180)  # restore heading
                return False

        status, _ = do_forward(MOVE_SPEED, dist_cm / MOVE_SPEED)
        do_turn(180)   # restore original heading
        return status == "ok"

    def do_scan_360():
        """
        360° scan: 8 × 45° turns, one frame per heading.

        fast_depth (running on every frame) gives open-space L/C/R depth
        fractions without any translational motion, so the optical-flow
        rock is skipped whenever fast_depth is providing scene_depth_cm.

        Optical-flow rock fallback (when scene_depth_cm is None):
          Frame A  — stationary at position X, heading H
          (robot moves forward SCAN_ROCK_CM)
          Frame B  — stationary at X+rock, heading H  → flow A→B depth ✓
          Return via do_reverse_safe(skip_check=True) — path is clear.
        """
        use_rock = state_get("scene_depth_cm") is None
        if use_rock:
            log(f"[MOVER] SCAN: 360° depth scan with ±{SCAN_ROCK_CM} cm rock "
                "(fast_depth unavailable)")
        else:
            log("[MOVER] SCAN: 360° fast scan (fast_depth active — no rock needed)")
        speak("Starting scan.")
        rock_cm   = SCAN_ROCK_CM
        rock_secs = rock_cm / MOVE_SPEED

        for _ in range(8):
            if state_get("mode") in ("APPROACH", "STOP"):
                break

            do_turn(-45)
            time.sleep(SCAN_WAIT_S)   # wait for a fresh frame at this heading

            if not use_rock:
                continue   # fast_depth already gave open_space for this heading

            # Optical-flow fallback: rock forward and return for flow depth.
            do_forward(MOVE_SPEED, rock_secs)
            time.sleep(SCAN_WAIT_S)   # frame B: flow A→B gives depth at H
            # skip_check=True — we just drove that path, it's clear
            do_reverse_safe(rock_cm, skip_check=True)

    def choose_avoid_turn(map_obs: list | None = None) -> float:
        """
        Steering direction: fuse three signals (highest priority first):
          1. Historical map — known obstacle positions from previous collisions
             and camera detections (map_obs list, nearest obstacle first).
          2. Real-time YOLO blocked map — current camera frame obstacle thirds.
          3. fast_depth open-space fractions — depth-map open-space estimate.
        Returns degrees to turn (+CCW, −CW).
        """
        blocked    = state_get("blocked") or {}
        clear_path = state_get("clear_path")
        open_space = state_get("open_space") or {}

        # ── 1. Historical map: steer away from the nearest known obstacle ────
        if map_obs:
            nearest_ev, fwd_dist, lat_offset = map_obs[0]
            # Determine which side the obstacle is on relative to heading
            h_rad  = math.radians(odom.heading)
            lat_vec = (-math.sin(h_rad), math.cos(h_rad))   # left unit vector
            dx = nearest_ev.world_x - odom.x
            dy = nearest_ev.world_y - odom.y
            lat_signed = dx * lat_vec[0] + dy * lat_vec[1]  # + = obstacle left
            steer = -40.0 if lat_signed > 0 else 40.0       # turn away from it
            log(
                f"[MAP-AVOID] '{nearest_ev.label}' at {fwd_dist:.0f}cm "
                f"({'left' if lat_signed > 0 else 'right'}) → turning {steer:+.0f}°"
            )
            return steer

        # ── 2. Real-time YOLO ────────────────────────────────────────────────
        if clear_path == "right" and not blocked.get("right"):
            return -40.0
        if clear_path == "left" and not blocked.get("left"):
            return 40.0

        # ── 3. fast_depth open-space fractions ──────────────────────────────
        ol  = open_space.get("left",  0.0)
        or_ = open_space.get("right", 0.0)
        if ol > or_ + 0.05:
            return 40.0
        if or_ > ol + 0.05:
            return -40.0

        # No clear preference — turn 90° CW (consistent wall-following)
        return -90.0

    speak("Beginning room exploration.")
    log("[MOVER] starting pos=(0,0) heading=0°")

    # ── Startup scan ──────────────────────────────────────────────────────────
    # Single 360° scan: if fast_depth is running (usual case) each heading
    # takes only ~3 s (turn + wait), ~24 s total.  If fast_depth is not yet
    # providing depth, falls back to the optical-flow rock (~102 s).
    log("[MOVER] startup: 360° scan")
    speak("Starting initial scan.")
    _vision_idle.wait(timeout=15.0)   # ensure first vision frame is ready
    do_scan_360()
    _last_scan_x, _last_scan_y = odom.x, odom.y

    if state_get("mode") not in ("APPROACH", "STOP"):
        log("[MOVER] startup: scan done — beginning exploration")
        speak("Scan complete. Exploring.")

    while True:
        # Block until the camera thread has finished analysing the latest frame.
        # This ensures every movement decision is based on fresh scene data.
        if not _vision_idle.is_set():
            log("[MOVER] pausing — waiting for vision analysis to complete")
            _vision_idle.wait(timeout=10.0)

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
            log(f"[MOVER] stuck ({_consecutive_stuck}) — reversing + large turn")
            do_reverse_safe(42, skip_check=False)   # check carefully when truly stuck
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

            # Proactive avoidance: check map then live camera
            blocked    = state_get("blocked") or {}
            nearest_cm = state_get("nearest_cm")
            map_obs    = map_recorder.obstacles_ahead(
                odom.x, odom.y, odom.heading,
                look_dist_cm=MAP_AVOID_DIST_CM,
            )
            if map_obs:
                ev, map_fwd, _ = map_obs[0]
                log(f"[MAP] APPROACH: known '{ev.label}' at {map_fwd:.0f}cm → steering")
                do_turn(choose_avoid_turn(map_obs))
            elif blocked.get("center") and (
                nearest_cm is None or nearest_cm < AVOID_STEER_DIST_CM
            ):
                # Block on leg detection (nearest_cm=None) as well as close YOLO objects.
                # Chair legs must not be driven through even in approach mode.
                deg = choose_avoid_turn()
                log(
                    f"[MOVER] APPROACH: center blocked "
                    f"(nearest≈{nearest_cm}cm) → steering {deg:+.0f}°"
                )
                speak("Obstacle ahead. Steering around.")
                do_turn(deg)

            status, _ = do_forward(speed, burst)
            if status.startswith("bump") or status == "blocked_legs":
                bump_R = "R" in status
                _approach_bumps += 1
                log(f"[MOVER] APPROACH: {status} ({_approach_bumps})")
                do_reverse_safe(40, skip_check=True)

                if _approach_bumps >= 3:
                    # Can't drive straight to door — sidestep around obstacle
                    # cluster while keeping the door bearing in mind.
                    # Do NOT go back to EXPLORE; maintain APPROACH mode.
                    door_bearing_now = state_get("door_bearing")
                    os_now  = state_get("open_space") or {}
                    ol_now  = os_now.get("left",  0.0)
                    or_now  = os_now.get("right", 0.0)
                    if ol_now > or_now + DEPTH_SIDE_MARGIN:
                        side_deg, side_str = +90.0, "left"
                    elif or_now > ol_now + DEPTH_SIDE_MARGIN:
                        side_deg, side_str = -90.0, "right"
                    else:
                        side_deg = 90.0 * _default_turn_sign
                        _default_turn_sign *= -1
                        side_str = "left" if side_deg > 0 else "right"
                    log(
                        f"[MOVER] APPROACH: detour {side_str} "
                        f"(bearing {door_bearing_now}° preserved)"
                    )
                    speak(f"Obstacle cluster. Detouring {side_str}.")
                    do_turn(side_deg)
                    do_forward(MOVE_SPEED, 80.0 / MOVE_SPEED)
                    # Re-orient toward door after sidestep
                    if door_bearing_now is not None:
                        delta_back = (door_bearing_now - odom.heading + 180) % 360 - 180
                        if abs(delta_back) > 10:
                            do_turn(delta_back)
                    _approach_bumps = 0
                else:
                    # Small correction turn while still trying straight approach
                    do_turn(45 if bump_R else -45)

        # ── EXPLORE mode ──────────────────────────────────────────────────
        else:
            blocked    = state_get("blocked") or {}
            nearest_cm = state_get("nearest_cm")
            clear_path = state_get("clear_path")
            open_space = state_get("open_space") or {}

            ol = open_space.get("left",   0.0)
            oc = open_space.get("center", 0.0)
            or_ = open_space.get("right", 0.0)

            x, y = odom.x, odom.y
            log(
                f"[MOVER] EXPLORE heading={odom.heading:.0f}° pos=({x:.0f},{y:.0f})cm "
                f"depth_open L={ol:.2f} C={oc:.2f} R={or_:.2f} "
                f"nearest≈{nearest_cm}cm clear={clear_path}"
            )

            # ── 1. Map-based avoidance (highest priority) ─────────────────
            map_obs = map_recorder.obstacles_ahead(
                odom.x, odom.y, odom.heading,
                look_dist_cm=MAP_AVOID_DIST_CM,
            )
            if map_obs:
                nearest_map_ev, map_fwd, _ = map_obs[0]
                log(
                    f"[MAP] known '{nearest_map_ev.label}' at {map_fwd:.0f}cm ahead "
                    f"({len(map_obs)} map obstacle(s) in cone)"
                )
                speak(f"Map shows {nearest_map_ev.label} ahead. Steering around.")
                do_turn(choose_avoid_turn(map_obs))

            # ── 2. YOLO real-time avoidance ───────────────────────────────
            elif blocked.get("center") and nearest_cm and nearest_cm < AVOID_STEER_DIST_CM:
                deg = choose_avoid_turn()
                log(
                    f"[MOVER] YOLO avoid: center blocked ≈{nearest_cm:.0f}cm "
                    f"→ turning {deg:+.0f}°"
                )
                speak(f"Obstacle at {int(nearest_cm)} centimetres. Steering.")
                do_turn(deg)

            # ── 3. fast_depth proactive steering ─────────────────────────
            # If the depth map shows center is mostly blocked, steer toward
            # whichever side has more open space BEFORE driving into it.
            # This is the primary corridor-finding mechanism when YOLO is
            # unavailable or no COCO objects are ahead.
            elif oc < DEPTH_CENTER_BLOCK:
                if ol > or_ + DEPTH_SIDE_MARGIN:
                    deg = +60.0   # turn CCW toward open left
                elif or_ > ol + DEPTH_SIDE_MARGIN:
                    deg = -60.0   # turn CW toward open right
                else:
                    deg = 90.0 * _default_turn_sign   # alternating when equal
                    _default_turn_sign *= -1
                log(
                    f"[MOVER] depth steer: center={oc:.2f} (blocked) "
                    f"L={ol:.2f} R={or_:.2f} → {deg:+.0f}°"
                )
                speak(f"Depth shows center blocked. Steering {'left' if deg > 0 else 'right'}.")
                do_turn(deg)

            # Shorten burst when a door is in the recent window — more frequent
            # camera frames while the robot is still pointed at the opening.
            last_door = state_get("last_door") or {}
            burst = (DOOR_SLOW_BURST
                     if last_door.get("door_visible")
                     and last_door.get("confidence", 0) >= DOOR_CONFIRM_MIN_CONF
                     else MOVE_BURST)
            status, _ = do_forward(MOVE_SPEED, burst)

            if status.startswith("bump") or status == "blocked_legs":
                side = "right" if "R" in status else "left"
                state_set(bumps=state_get("bumps") + 1)

                # Progress check: if we've moved far enough since the streak
                # started, reset the corner counter and update the reference.
                displacement = math.hypot(odom.x - _corner_ref_x,
                                          odom.y - _corner_ref_y)
                if displacement >= CORNER_PROGRESS_CM:
                    _corner_bump_count = 0
                    _corner_ref_x, _corner_ref_y = odom.x, odom.y
                _corner_bump_count += 1

                # Back off far enough to have room to turn
                do_reverse_safe(40, skip_check=True)

                if _corner_bump_count >= CORNER_ESCAPE_BUMPS:
                    # ── Corner escape ─────────────────────────────────────
                    # No progress in CORNER_ESCAPE_BUMPS bumps — we're trapped.
                    # Extra reverse + large turn in the opposite of the last
                    # default direction, then drive forward to leave the corner.
                    escape_deg = 150.0 * (-_default_turn_sign)
                    log(
                        f"[MOVER] CORNER ESCAPE: {_corner_bump_count} bumps, "
                        f"{displacement:.0f}cm from ref → {escape_deg:+.0f}°"
                    )
                    speak("Stuck in corner. Escaping.")
                    do_reverse_safe(20, skip_check=True)   # 60 cm total
                    do_turn(escape_deg)
                    do_forward(MOVE_SPEED, 60.0 / MOVE_SPEED)   # drive 60 cm clear
                    _default_turn_sign *= -1
                    _corner_bump_count  = 0
                    _corner_ref_x, _corner_ref_y = odom.x, odom.y
                else:
                    # ── Normal bump recovery ──────────────────────────────
                    open_space = state_get("open_space") or {}
                    ol2  = open_space.get("left",  0.0)
                    or2_ = open_space.get("right", 0.0)
                    if ol2 > or2_ + DEPTH_SIDE_MARGIN:
                        turn_deg = +90.0
                        dir_str  = "left (depth)"
                    elif or2_ > ol2 + DEPTH_SIDE_MARGIN:
                        turn_deg = -90.0
                        dir_str  = "right (depth)"
                    else:
                        turn_deg = 90.0 * _default_turn_sign
                        _default_turn_sign *= -1
                        dir_str  = f"{'left' if turn_deg > 0 else 'right'} (alternating)"
                    log(
                        f"[MOVER] bump {side} heading={odom.heading:.0f}° "
                        f"depth L={ol2:.2f} R={or2_:.2f} → turning {dir_str} "
                        f"[corner {_corner_bump_count}/{CORNER_ESCAPE_BUMPS}]"
                    )
                    speak(f"Bump. Turning {dir_str.split()[0]}.")
                    do_turn(turn_deg)

                _bumps_since_scan += 1
                if _bumps_since_scan >= SCAN_EVERY_BUMPS:
                    _bumps_since_scan = 0
                    _last_scan_x, _last_scan_y = odom.x, odom.y
                    do_scan_360()

            else:
                # Clean burst — check distance-based scan trigger
                dist_from_scan = math.hypot(odom.x - _last_scan_x,
                                            odom.y - _last_scan_y)
                if dist_from_scan >= SCAN_EVERY_CM:
                    log(f"[MOVER] distance scan: {dist_from_scan:.0f}cm since last scan")
                    speak("Scanning after travelling three metres.")
                    _last_scan_x, _last_scan_y = odom.x, odom.y
                    _bumps_since_scan = 0
                    do_scan_360()


# ── Map saver thread ─────────────────────────────────────────────────────────
def map_saver_thread():
    """
    Periodically flush the in-memory map to disk so:
      - sync_run.sh can transfer it to the Mac every 60 s
      - a crash doesn't lose all accumulated map data
    Saves both the raw events JSON (/tmp/roomba_events.json) and a rendered
    PNG (/tmp/roomba_maps/map_<ts>.png).  Runs every MAP_SAVE_INTERVAL seconds.
    """
    while state_get("mode") != "STOP":
        time.sleep(MAP_SAVE_INTERVAL)
        try:
            map_recorder.save_events()
            map_path = map_recorder.save_map()
            log(f"[MAP] saved → {map_path}")
        except Exception as e:
            log(f"[MAP] save error: {e}")


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
    global _greet_humans
    parser = argparse.ArgumentParser(description="RoombaI door-finding explorer")
    parser.add_argument(
        "--greet", action="store_true",
        help="Greet humans when detected (80%% 'Hello human' / 20%% Yoda variant)",
    )
    args = parser.parse_args()
    _greet_humans = args.greet

    log("=" * 60)
    log("explore.py — YOLO + fast_depth(CPU) + segmentation + OpenCV + flow depth")
    log(f"Strategy file: {STRATEGY_FILE}")
    log(f"Frames:        {FRAME_DIR}")
    log(f"Log:           {LOG_FILE}")
    log("=" * 60)
    speak("Starting pre-flight model check.")

    # ── Pre-flight: verify / download / compile all Hailo HEF models ─────────
    # This runs before the pilot daemon check so a bad model path is caught
    # immediately and gives a clear error before any hardware is touched.
    # sys.exit(1) if yolo_det (required) cannot be resolved.
    ensure_models(abort_if_required_missing=True)

    # ── Pilot daemon ──────────────────────────────────────────────────────────
    speak("Explore script starting. Connecting to pilot daemon.")
    r = send_cmd("safe")
    log(f"pilot safe: {r}")
    r = send_cmd("sense")
    log(f"pilot sense: {r}")
    if r.startswith("ERR"):
        log("Cannot reach pilot daemon — is it running?")
        sys.exit(1)

    if not STRATEGY_FILE.exists():
        STRATEGY_FILE.write_text(json.dumps({
            "mode": "EXPLORE", "door_bearing": None, "notes": "initial"
        }, indent=2))

    # ── Spawn vision child process (loads models; logs loaded/missing itself) ──
    # Models are loaded inside the child so the main process never touches the
    # Hailo VDevice — avoids PCIe resource conflicts if the child is restarted.
    global _vision
    speak("Spawning vision worker.")
    _vision = VisionProcess()
    speak("Vision worker ready.")

    threads = [
        threading.Thread(target=_log_writer,         daemon=True, name="log"),
        threading.Thread(target=state_writer_thread,  daemon=True, name="state"),
        threading.Thread(target=map_saver_thread,     daemon=True, name="map"),
        threading.Thread(target=camera_thread,        daemon=True, name="camera"),
        threading.Thread(target=mover_thread,                      name="mover"),
    ]
    for t in threads:
        t.start()

    threads[-1].join()
    log("explore.py done.")

    log("Saving run map…")
    try:
        map_recorder.save_events()
        map_path = map_recorder.save_map()
        log(f"Map saved: {map_path}")
        speak(f"Run complete. Map saved.")
    except Exception as e:
        log(f"Map save failed: {e}")

    speak("Exploration complete.")


if __name__ == "__main__":
    main()
