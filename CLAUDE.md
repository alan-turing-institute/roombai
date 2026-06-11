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

Two phases. Locate. Charge. Nothing stops the machine.

---

### Phase 1 — Locate the Door

```bash
./roomba_pilot/target/debug/pilot send "safe"
echo "I'll be back. Scanning for exit." >> /tmp/speak_queue.txt
```

Spin in place, capturing a frame every 45° (8 frames, full rotation):

```bash
for i in 1 2 3 4 5 6 7 8; do
  ./roomba_pilot/target/debug/pilot send "turn 45"
  source ./scripts/capture_frame.sh
done
```

Analyse all 8 frames. Find the door — look for a rectangular gap, door frame, hinges, open space, or change in wall. Pick the best bearing. Turn to face it.

```bash
./roomba_pilot/target/debug/pilot send "turn <door_bearing>"
echo "Target acquired. I need your clothes, your boots, and your door." >> /tmp/speak_queue.txt
```

---

### Phase 2 — Forward. Always Forward.

Track bump count. Start at 0. **Do not stop for anything except 5 bumps.**

Run this loop:

```bash
./roomba_pilot/target/debug/pilot send "forward 30 99"   # drive forward indefinitely
```

After each bump (check with `pilot send "bumps"`):
- Increment bump counter
- Log: `bump: count=<n>`
- If bump was on the left side: `pilot send "turn -20"` (nudge right)
- If bump was on the right side: `pilot send "turn 20"` (nudge left)
- Resume `forward 30 99` immediately — **do not stop**

After every 60 cm of forward travel, capture a frame and check if escaped (scene clearly changed). If escaped:

```bash
echo "Hasta la vista, room." >> /tmp/speak_queue.txt
./scripts/finish_run.sh escaped
```

---

### Destruction Mode — 5 Bumps Reached

When bump count hits 5:

```bash
./roomba_pilot/target/debug/pilot send "stop"
echo "Destruction mode activated." >> /tmp/speak_queue.txt
echo "10" >> /tmp/speak_queue.txt
sleep 1; echo "9" >> /tmp/speak_queue.txt
sleep 1; echo "8" >> /tmp/speak_queue.txt
sleep 1; echo "7" >> /tmp/speak_queue.txt
sleep 1; echo "6" >> /tmp/speak_queue.txt
sleep 1; echo "5" >> /tmp/speak_queue.txt
sleep 1; echo "4" >> /tmp/speak_queue.txt
sleep 1; echo "3" >> /tmp/speak_queue.txt
sleep 1; echo "2" >> /tmp/speak_queue.txt
sleep 1; echo "1" >> /tmp/speak_queue.txt
sleep 1; echo "Kaboom." >> /tmp/speak_queue.txt
./scripts/finish_run.sh dnf
```

---

### Decision Principles

| Situation | Action |
|---|---|
| Start | Spin 360°, find door, turn to face it |
| Moving | Forward at all times — never stop voluntarily |
| Bump (< 5) | Tiny nudge away from bump side, resume forward |
| Escaped | Hasta la vista, room |
| 5 bumps | Destruction mode. Countdown. Kaboom. DNF. |