# RoombaI — CLAUDE.md

## Goal

You are controlling a Roomba cleaning robot. A camera is mounted on it.
The task is to pilot the robot to exit via a specific door (shown in the starting prompt image).
You will be starting in a different room, so you must first find the room that contains that door,
then find the door, then exit through it.

**Doors must be open to exit.** It is not possible to exit through a closed door.
Doors open when a human is about to use them — so if a target door is closed, wait nearby.

---

## Architecture

```
roomba_pilot/          Rust library + pilot binary (serial ↔ TCP bridge)
find_door.py           Navigation script — movement control + vision loop
prompts/starting.md    Starting goal prompt
```

### Pilot daemon

The `pilot` binary holds the serial port open and accepts newline-delimited commands
over a local TCP socket.

```bash
# Start the daemon (run once; auto-enters SAFE mode and verifies it)
cd roomba_pilot
./target/debug/pilot serve /dev/ttyUSB0

# Send a command from any shell
./target/debug/pilot send "sense"
```

Default port: `127.0.0.1:9999`. The robot **must be powered on before the daemon starts**
(or it will fail — the daemon verifies OI mode 2 via a sensor query on startup).

### Pilot command reference

| Command | Effect |
|---|---|
| `forward <cm/s> <s>` / `back <cm/s> <s>` | Drive straight for a fixed time |
| `move <cm>` | Drive a signed distance at fixed speed |
| `spin <deg/s> <s>` | Rotate in place for a time |
| `turn <deg>` | Rotate a signed angle (+CCW / −CW) at 60°/s |
| `go <cm/s> <deg/s>` | Raw continuous motion (3 s safety window) |
| `stop` | Halt wheels |
| `sense` | Battery, OI mode, bumpers |
| `bumps` | Bumper + wheel-drop state |
| `safe` | Enter OI SAFE mode (mode 2) — required for movement |
| `full` | Enter OI FULL mode (mode 3) |
| `motors <side> <main> <vac>` | Brushes/vacuum |
| `dock` | Seek charging dock |
| `ping` | Health check |
| `shutdown` | Stop, return to passive, exit daemon |

### Camera

```bash
rpicam-still --nopreview --width 1280 --height 720 -o /tmp/view.jpg -t 800
rpicam-hello   # live preview (video)
```

---

## Constraints

### No external LLM API
There is no `ANTHROPIC_API_KEY` available. Do **not** use the `anthropic` Python SDK
or make any requests to external model APIs.

### Vision inference — Hailo AI Hat+ (YOLO) + OpenCV

All vision runs locally on the Pi. No LLM calls.

**YOLO (Hailo AI Hat+)** — detects COCO objects (person, chair, etc.) for obstacle context:
```python
# hailo_platform SDK — model at /usr/share/hailo-models/yolov8s_h8l.hef
# See _init_hailo() / run_yolo() in explore.py for the full initialisation pattern
```

**OpenCV door detection** — geometric analysis to find doorway openings:
```python
import cv2, numpy as np

def detect_door_cv(img_bgr):
    h, w   = img_bgr.shape[:2]
    gray   = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    edges  = cv2.Canny(cv2.GaussianBlur(gray, (5,5), 0), 30, 100)
    # Find near-vertical line segments → cluster x-positions → score pairs
    # by gap width, low edge density in gap, and brightness ratio.
    # Returns: door_visible, door_open, door_position, in_doorway, confidence
```

The combined `analyze(image_path)` function in `find_door.py` calls both and merges results.
Vision runs on **every frame** (every 2 s in `explore.py`, each step in `find_door.py`).
Response latency is milliseconds — no 20-30 s LLM wait.

### Speaker daemon (TTS during long runs)

A background TTS daemon watches `/tmp/speak_queue.txt` and speaks each new line via `espeak-ng`.
**Always start this daemon before any long-running task**, and narrate key reasoning steps by
appending to the queue throughout the run — do not wait until the end.

```bash
# Start the daemon (run once per session)
nohup bash -c 'tail -n 0 -f /tmp/speak_queue.txt | while IFS= read -r line; do espeak-ng -s 145 -- "$line" 2>/dev/null; done' > /tmp/speak_daemon.log 2>&1 &

# Speak something (from shell or Python)
echo "Scanning for target door" >> /tmp/speak_queue.txt
```

In Python:
```python
def speak(text: str):
    with open("/tmp/speak_queue.txt", "a") as f:
        f.write(text + "\n")
```

Narrate at every significant decision point: scan results, door detections, movement decisions,
obstacles encountered, waiting states, and arrival confirmation.

---

## IMPORTANT: Read previous attempts first

Before doing anything else, read these files to understand what has been tried and what worked/failed:

- `attempt_1_review.md` — first attempt, camera angle issues, no movement
- `attempt_2_review.md` — second attempt, found the door but strategy too slow
- `attempt_4_review.md` — fourth attempt, good movement but stuck in chair bump loop

**Critical guidance from attempt 4 (user feedback):**
- **Do NOT trust room layout information from previous attempts.** Treat each run as starting with a blank slate — prior door location guesses led to premature APPROACH mode and wasted time.
- **Come up with a clear strategy at the start of each run** and monitor against it. If things aren't going well, stop and revise.
- **Use the camera actively** for navigation decisions throughout the run, not just for periodic door confirmation.
- **Be fluid** — don't be locked into explore.py or any fixed script. Adjust approach based on what is actually observed.
- Aggressive bumping and movement is good. Keep doing it.

---

## Exploration software — explore.py

`explore.py` is the primary navigation script. It runs the robot **concurrently**:
- Movement loop runs independently (no waiting for vision)
- Camera captures every 2s; YOLO + OpenCV door detection runs on every frame
- Door state machine updates automatically (no human supervisor needed)
- Reads `/tmp/roomba_strategy.json` for manual overrides

### Start the run

```bash
# 1. Ensure pilot daemon is running
cd /home/hackweek26/roombai/roomba_pilot
./target/debug/pilot serve /dev/ttyUSB0 &

# 2. Start TTS daemon
nohup bash -c 'tail -n 0 -f /tmp/speak_queue.txt | while IFS= read -r line; do espeak-ng -s 145 -- "$line" 2>/dev/null; done' > /tmp/speak_daemon.log 2>&1 &

# 3. Run the explorer (from repo root)
cd /home/hackweek26/roombai
python3 explore.py &

# 4. Monitor
tail -f /tmp/roomba_log.txt
```

### Supervisor workflow

```python
# Read current frame
read /tmp/roomba_current.jpg   # always the latest frame

# Read keyframes
ls /tmp/roomba_frames/         # last 20 frames

# Check robot state
cat /tmp/roomba_state.json

# Override strategy (e.g. send robot toward known door bearing)
# Write /tmp/roomba_strategy.json:
{
  "mode": "APPROACH",
  "door_bearing": 135.0,
  "notes": "glass wall door spotted at 135 degrees"
}
```

### Strategy modes
| Mode | Behaviour |
|---|---|
| `EXPLORE` | Aggressive random walk, bump-and-turn |
| `APPROACH` | Turn to `door_bearing`, drive toward it |
| `WAIT` | Stop near door, poll camera until it opens |
| `STOP` | Halt immediately |

---

## Navigation strategy (find_door.py)

1. **SCAN** — spin in 45° steps (8 per revolution), take a photo at each position,
   ask Claude if the target door is visible and where in frame.
2. **APPROACH** — once the door is spotted, drive toward it in short bursts,
   steering to keep it centred. Check for bumps; back up and steer around obstacles.
3. **WAIT** — if the door is visible but closed, stop nearby and poll the camera
   until the door opens.
4. **ARRIVE** — stop when Claude confirms the robot is in the doorway.

If the target door is not found after a full sweep, move to a new vantage point
(or into the next room) and scan again.
