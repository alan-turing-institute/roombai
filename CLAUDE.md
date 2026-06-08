# RoombaI — CLAUDE.md

## Goal

You are controlling a Roomba cleaning robot. A camera is mounted on it.
The task is the escape the room via an open door.

---

## Architecture

```
roomba_pilot/          Rust library + pilot binary (serial ↔ TCP bridge)
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

`rpicam-still` and `rpicam-hello` can be used to capture still or video, respectively.

---

## Constraints

### No external LLM API
There is no `ANTHROPIC_API_KEY` available. Do **not** use the `anthropic` SDK
or make any requests to external model APIs.

### Speaker daemon (TTS during long runs)

A background TTS daemon watches `/tmp/speak_queue.txt` and speaks each new line via `espeak-ng`.
**Always start this daemon before any long-running task**, and narrate key reasoning steps by
appending to the queue throughout the run — do not wait until the end.

```bash
# Start the daemon (run once per session)
nohup bash -c 'tail -n 0 -f /tmp/speak_queue.txt | while IFS= read -r line; do espeak-ng -s 145 -- "$line" 2>/dev/null; done' > /tmp/speak_daemon.log 2>&1 &

# Speak something
echo "Scanning for target door" >> /tmp/speak_queue.txt
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
