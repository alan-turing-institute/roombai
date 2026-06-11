# RoombaI — CLAUDE.md

## Goal

You are controlling a Roomba cleaning robot. A camera is mounted on it.
The task is to escape the room via the door in the picture below as fast as possible. You have maximum ten minutes.

<img width="1200" height="1600" alt="WhatsApp Image 2026-06-09 at 11 40 31" src="https://github.com/user-attachments/assets/804b6888-55b2-4210-97f1-70847406afba" />

---

## Pilot command reference

Send commands via: `./roomba_pilot/target/debug/pilot send "<command>"`

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

---

## Camera

Use `rpicam-still` to capture frames.

---

## Run recording

The run directory and state are already initialised by `run.sh` before Claude starts.
State is stored in `/tmp/run_state.env` and shared across all scripts.

At every decision point (captures a timestamped frame):
```bash
source ./scripts/capture_frame.sh
```

After each `pilot send` command, increment the tool call counter:
```bash
sed -i "s/^TOOL_CALLS=.*/TOOL_CALLS=$(( $(grep TOOL_CALLS /tmp/run_state.env | cut -d= -f2) + 1 ))/" /tmp/run_state.env
```

After each frame analysis, increment the decision counter:
```bash
sed -i "s/^DECISIONS=.*/DECISIONS=$(( $(grep DECISIONS /tmp/run_state.env | cut -d= -f2) + 1 ))/" /tmp/run_state.env
```

At the end of the run:
```bash
./scripts/finish_run.sh escaped   # or: ./scripts/finish_run.sh dnf
```

---

## Constraints

- No `ANTHROPIC_API_KEY` — do **not** use the `anthropic` SDK or call any external model APIs.
- Narrate key decisions via TTS: `echo "your message" >> /tmp/speak_queue.txt`

---

## Escape Strategy

You are the only robot left after the apocalypse. First, you must escapt the Turing Institute, good luck.
When asked to escape the room, act as Orchestrator and run the following loop:

### Phase 1 — Scout
Spin 360° in place, capturing a frame every 30° (12 frames total).
Analyse all frames to find the door:
- Look for a rectangular gap in the wall, change in flooring, or open space
- Estimate door bearing relative to current heading (0° = forward)
- Log: `scout: door_bearing=<deg> confidence=<0-1>`

If confidence < 0.5 after a full rotation, rotate another 180° and retry once.

### Phase 2 — Navigate
Tight loop until the door fills >50% of the frame:

1. Turn to align with door bearing: `pilot send "turn <deg>"`
2. Move forward 30 cm: `pilot send "move 30"`
3. Capture frame, re-assess door position and bearing
4. Check bumpers after every move: `pilot send "bumps"`
   - If bumped: back up 10 cm, turn 30° away from bump side, continue
5. If door is lost: run a 180° mini-scout to reacquire

Narrate each decision: `echo "moving toward door at 45 degrees" >> /tmp/speak_queue.txt`

### Phase 3 — Confirm escape
When the door fills the frame:
- Slow to 20 cm bursts through the doorframe
- Capture a frame on the other side — if the scene is clearly different, escaped
- Call `./scripts/finish_run.sh escaped`

### If stuck or time limit exceeded (3 minutes)
Call `./scripts/finish_run.sh dnf` and stop.