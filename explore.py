#!/usr/bin/env python3
"""
explore.py — Concurrent Roomba room exploration for door finding.

The robot moves and captures frames CONCURRENTLY. Claude is the external
supervisor: it reads the log and keyframes, then updates the strategy file.

Threads:
  mover     — movement state machine, runs independently of vision
  camera    — frame capture + Hailo inference every 2s
  supervisor— periodic Claude vision check every ~30s (background)
  logger    — flushes state to JSON every second

Claude supervisor workflow:
  1. Start:    python3 explore.py &
  2. Monitor:  tail -f /tmp/roomba_log.txt
  3. Inspect:  ls /tmp/roomba_frames/   (keyframes saved every 5s)
  4. Redirect: write /tmp/roomba_strategy.json  (see STRATEGY below)
  5. Stop:     write {"mode":"STOP"} to strategy file, or kill the process

STRATEGY FILE  /tmp/roomba_strategy.json
  {
    "mode":          "EXPLORE" | "APPROACH" | "WAIT" | "STOP",
    "door_bearing":  <degrees from start, float> | null,
    "notes":         "free text logged and spoken"
  }
"""

import json
import math
import os
import queue
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

# ── Config ─────────────────────────────────────────────────────────────────
PILOT_HOST = "127.0.0.1"
PILOT_PORT = 9999

FRAME_DIR = Path("/tmp/roomba_frames")
LOG_FILE  = Path("/tmp/roomba_log.txt")
STATE_FILE = Path("/tmp/roomba_state.json")
STRATEGY_FILE = Path("/tmp/roomba_strategy.json")

FRAME_INTERVAL   = 2.0    # seconds between camera captures
KEYFRAME_EVERY   = 3      # save every Nth frame as a keyframe for Claude
SUPERVISE_EVERY  = 5.0    # seconds between Claude vision checks
MOVE_SPEED       = 35     # cm/s forward speed
MOVE_BURST       = 3.0    # seconds per forward burst
TURN_AFTER_BUMP  = 90     # degrees to turn after bumping (always right/CW for wall-following)

# ── Shared state ────────────────────────────────────────────────────────────
_lock = threading.Lock()
_state = {
    "mode": "EXPLORE",
    "heading": 0.0,          # cumulative degrees, CCW positive
    "bumps": 0,
    "frames_captured": 0,
    "last_detection": None,
    "door_bearing": None,
    "log_lines": 0,
}

def state_get(key):
    with _lock:
        return _state[key]

def state_set(**kw):
    with _lock:
        _state.update(kw)

# ── Logging ─────────────────────────────────────────────────────────────────
LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
LOG_FILE.touch()
_log_q = queue.Queue()

def log(msg: str):
    ts = time.strftime("%H:%M:%S")
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
            reply = s.makefile().readline().strip()
            return reply
    except Exception as e:
        return f"ERR {e}"

# ── Camera ───────────────────────────────────────────────────────────────────
FRAME_DIR.mkdir(parents=True, exist_ok=True)
_CURRENT_FRAME = Path("/tmp/roomba_current.jpg")
_frame_lock = threading.Lock()
_frame_count = 0

def capture_frame(path: Path) -> bool:
    try:
        r = subprocess.run(
            ["rpicam-still", "--nopreview",
             "--width", "640", "--height", "480",
             "--rotation", "180",
             "-o", str(path), "-t", "500"],
            capture_output=True, timeout=5
        )
        return r.returncode == 0
    except Exception as e:
        log(f"capture error: {e}")
        return False

# ── Hailo inference ──────────────────────────────────────────────────────────
_hailo_available = False
_hailo_model = None
COCO_CLASSES = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier",
    "toothbrush",
]

def _init_hailo():
    global _hailo_available, _hailo_model
    try:
        import numpy as np
        import cv2
        from hailo_platform import VDevice, HEF, ConfigureParams, \
            InputVStreamParams, OutputVStreamParams, FormatType, \
            HailoStreamInterface, InferVStreams
        hef_path = "/usr/share/hailo-models/yolov8s_h8l.hef"
        hef = HEF(hef_path)
        target = VDevice()
        configure_params = ConfigureParams.create_from_hef(
            hef, interface=HailoStreamInterface.PCIe)
        network_groups = target.configure(hef, configure_params)
        ng = network_groups[0]
        ng_params = ng.create_params()
        input_vstreams_params = InputVStreamParams.make(ng, format_type=FormatType.UINT8)
        output_vstreams_params = OutputVStreamParams.make(ng, format_type=FormatType.FLOAT32)
        _hailo_model = {
            "target": target, "ng": ng, "ng_params": ng_params,
            "input_params": input_vstreams_params,
            "output_params": output_vstreams_params,
            "hef": hef,
        }
        _hailo_available = True
        log("Hailo model loaded: yolov8s")
    except Exception as e:
        log(f"Hailo unavailable (will skip inference): {e}")

def run_hailo_inference(img_path: Path) -> list[str]:
    """Returns list of detected class names with confidence > 0.4"""
    if not _hailo_available or _hailo_model is None:
        return []
    try:
        import numpy as np
        import cv2
        from hailo_platform import InferVStreams
        img = cv2.imread(str(img_path))
        if img is None:
            return []
        img_resized = cv2.resize(img, (640, 640))
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        m = _hailo_model
        with m["ng"].activate(m["ng_params"]):
            with InferVStreams(m["ng"], m["input_params"], m["output_params"]) as pipeline:
                input_name = pipeline.get_input_vstream_infos()[0].name
                input_data = {input_name: np.expand_dims(img_rgb, 0).astype(np.uint8)}
                with pipeline.async_infer(input_data) as job:
                    output = job.get()
        detections = []
        for out_name, out_data in output.items():
            arr = out_data[0] if out_data.ndim > 2 else out_data
            if arr.ndim == 2:
                for det in arr:
                    if len(det) >= 6:
                        conf = float(det[4])
                        cls  = int(det[5])
                        if conf > 0.4 and 0 <= cls < len(COCO_CLASSES):
                            detections.append(COCO_CLASSES[cls])
        return list(set(detections))
    except Exception as e:
        log(f"hailo inference error: {e}")
        return []

# ── Claude vision (async, non-blocking) ────────────────────────────────────
def _claude_vision_async(img_path: Path):
    """Runs claude --print in background; writes result to state."""
    prompt = (
        f"Read the image at {img_path} and reply with ONLY JSON:\n"
        '{"door_visible":true/false,"door_open":true/false,'
        '"door_position":"left"|"center"|"right"|null,'
        '"in_doorway":true/false,'
        '"rounded_column_visible":true/false,'
        '"notes":"one sentence"}'
    )
    try:
        r = subprocess.run(
            ["claude", "--print", "--allowedTools", "Read",
             "--dangerously-skip-permissions"],
            input=prompt, capture_output=True, text=True, timeout=45
        )
        text = r.stdout.strip()
        if text.startswith("```"):
            text = text.split("\n", 1)[1].rsplit("```", 1)[0].strip()
        result = json.loads(text)
        log(f"[VISION] {result}")
        speak(f"Vision check. {result.get('notes','')}")

        if result.get("in_doorway"):
            state_set(mode="STOP")
            log("VISION: in doorway — stopping!")
            speak("I am in the doorway. Mission complete.")
        elif result.get("door_visible") and result.get("door_open"):
            pos = result.get("door_position", "center")
            heading = state_get("heading")
            offset = {"left": 30, "center": 0, "right": -30}.get(pos, 0)
            bearing = (heading + offset) % 360
            log(f"VISION: open door at {pos}, bearing ~{bearing:.0f}°")
            speak(f"Door spotted on the {pos}. Approaching.")
            state_set(mode="APPROACH", door_bearing=bearing)
        elif result.get("rounded_column_visible"):
            log("VISION: rounded column visible — door is nearby")
            speak("Rounded column visible. Door should be close.")
    except Exception as e:
        log(f"claude vision error: {e}")

# ── Strategy file reader ─────────────────────────────────────────────────────
def _read_strategy():
    """Apply any external strategy overrides from the strategy file."""
    try:
        if STRATEGY_FILE.exists():
            data = json.loads(STRATEGY_FILE.read_text())
            mode = data.get("mode")
            if mode and mode != state_get("mode"):
                log(f"[STRATEGY] mode override: {mode}")
                speak(f"Strategy update received. Switching to {mode}.")
                notes = data.get("notes", "")
                if notes:
                    log(f"[STRATEGY] notes: {notes}")
                state_set(mode=mode)
                if "door_bearing" in data and data["door_bearing"] is not None:
                    state_set(door_bearing=float(data["door_bearing"]))
    except Exception as e:
        log(f"strategy read error: {e}")

# ── Camera + inference thread ────────────────────────────────────────────────
def camera_thread():
    global _frame_count
    frame_num = 0
    supervisor_timer = 0.0
    while state_get("mode") != "STOP":
        time.sleep(FRAME_INTERVAL)
        frame_num += 1
        path = FRAME_DIR / f"frame_{frame_num:05d}.jpg"
        if not capture_frame(path):
            continue

        # Always keep a "current" copy for quick reads
        try:
            import shutil
            shutil.copy(str(path), str(_CURRENT_FRAME))
        except Exception:
            pass

        # Delete old frames (keep last 20)
        frames = sorted(FRAME_DIR.glob("frame_*.jpg"))
        for old in frames[:-20]:
            old.unlink(missing_ok=True)

        state_set(frames_captured=frame_num)
        with _frame_lock:
            _frame_count = frame_num

        # Hailo inference
        detections = run_hailo_inference(path)
        if detections:
            log(f"[DETECT] frame {frame_num}: {', '.join(detections)}")
            state_set(last_detection=detections)

        # Periodic Claude vision check
        supervisor_timer += FRAME_INTERVAL
        if supervisor_timer >= SUPERVISE_EVERY:
            supervisor_timer = 0.0
            log(f"[SUPERVISOR] triggering claude vision on frame {frame_num}")
            t = threading.Thread(target=_claude_vision_async, args=(path,), daemon=True)
            t.start()

# ── Movement thread ──────────────────────────────────────────────────────────
def mover_thread():
    """Aggressive exploration: drive → bump → turn → repeat."""
    heading = 0.0  # local heading tracker

    def do_forward(speed: int, secs: float) -> str:
        r = send_cmd(f"forward {speed} {secs:.1f}")
        time.sleep(secs + 0.1)
        return r

    def do_turn(deg: float):
        nonlocal heading
        r = send_cmd(f"turn {deg:.0f}")
        heading = (heading + deg) % 360
        state_set(heading=heading)
        time.sleep(abs(deg) / 60.0 + 0.3)
        return r

    def do_back(secs: float = 0.8):
        r = send_cmd(f"back {MOVE_SPEED} {secs:.1f}")
        time.sleep(secs + 0.1)
        return r

    def bumped() -> tuple[bool, bool]:
        r = send_cmd("bumps")
        return "bumpL=1" in r, "bumpR=1" in r

    def ensure_full():
        """Re-enter FULL mode only if a command response indicates mode dropped."""
        send_cmd("full")

    speak("Beginning room exploration.")
    log("[MOVER] starting")

    _approach_bumps = 0
    _moves_since_full = 0

    while True:
        _read_strategy()
        mode = state_get("mode")

        # Re-assert FULL mode every 30 moves (no sense query — just send it)
        _moves_since_full += 1
        if _moves_since_full >= 30:
            send_cmd("full")
            _moves_since_full = 0

        if mode == "STOP":
            send_cmd("stop")
            log("[MOVER] stopped by strategy")
            speak("Stopping.")
            break

        if mode != "APPROACH":
            _approach_bumps = 0

        if mode == "APPROACH":
            door_bearing = state_get("door_bearing")
            if door_bearing is not None:
                # Turn to face door bearing
                delta = (door_bearing - heading + 180) % 360 - 180
                if abs(delta) > 10:
                    log(f"[MOVER] APPROACH: turning {delta:+.0f}° toward door bearing {door_bearing:.0f}°")
                    speak(f"Turning toward door.")
                    do_turn(delta)
                # Drive toward it
                log("[MOVER] APPROACH: driving toward door")
                do_forward(MOVE_SPEED, MOVE_BURST)
                bl, br = bumped()
                if bl or br:
                    _approach_bumps += 1
                    log(f"[MOVER] APPROACH: bump ({_approach_bumps}/5) — backing and trying different angle")
                    do_back()
                    do_turn(30 if br else -30)
                    if _approach_bumps >= 5:
                        log("[MOVER] APPROACH: 5 bumps without progress — reassessing, switching to EXPLORE")
                        speak("Too many bumps approaching. Reassessing.")
                        state_set(mode="EXPLORE")
                        _approach_bumps = 0
            else:
                state_set(mode="EXPLORE")

        else:  # EXPLORE
            log(f"[MOVER] EXPLORE: forward {MOVE_SPEED}cm/s {MOVE_BURST}s (heading {heading:.0f}°)")
            do_forward(MOVE_SPEED, MOVE_BURST)

            # Check bumpers — single TCP call reused from last forward
            bl, br = bumped()
            if bl or br:
                side = "left" if bl else "right"
                log(f"[MOVER] bump {side} at heading {heading:.0f}°")
                speak(f"Bump. Turning right.")
                state_set(bumps=state_get("bumps") + 1)
                do_back(0.5)
                # Always turn right (CW = negative) for consistent wall-following coverage
                do_turn(-TURN_AFTER_BUMP)

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
    log("explore.py starting — attempt 3")
    log(f"Strategy file: {STRATEGY_FILE}")
    log(f"Keyframes:     {FRAME_DIR}")
    log(f"Log:           {LOG_FILE}")
    log("=" * 60)
    speak("Explore script starting. Attempt three.")

    # Ensure pilot daemon is in full mode (disables cliff-sensor stops)
    r = send_cmd("full")
    log(f"pilot full: {r}")
    r = send_cmd("sense")
    log(f"pilot sense: {r}")
    if r.startswith("ERR"):
        log("Cannot reach pilot — is daemon running?")
        sys.exit(1)

    # Write blank strategy file so Claude knows the format
    if not STRATEGY_FILE.exists():
        STRATEGY_FILE.write_text(json.dumps({
            "mode": "EXPLORE",
            "door_bearing": None,
            "notes": "initial — Claude should update this"
        }, indent=2))

    _init_hailo()

    # Start threads
    threads = [
        threading.Thread(target=_log_writer,       daemon=True, name="log"),
        threading.Thread(target=state_writer_thread,daemon=True, name="state"),
        threading.Thread(target=camera_thread,      daemon=True, name="camera"),
        threading.Thread(target=mover_thread,                    name="mover"),
    ]
    for t in threads:
        t.start()

    # Wait for mover (the only non-daemon thread) to finish
    threads[-1].join()
    log("explore.py done.")
    speak("Exploration complete.")

if __name__ == "__main__":
    main()
