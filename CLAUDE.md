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

Act as Orchestrator. Speed is everything — minimise analysis, maximise movement. The door is large; once you're roughly aimed at it, momentum beats caution.

**Known starting condition:** the robot begins at the far end of the room from the door, facing directly away from it (door is 180° behind). Skip the full scan — turn around immediately and charge.

---

### Phase 0 — Ignition + Immediate Turn (do once at start)

```bash
./roomba_pilot/target/debug/pilot send "safe"
echo "I'll be back. Scanning for exit." >> /tmp/speak_queue.txt
./roomba_pilot/target/debug/pilot send "turn 180"
source ./scripts/capture_frame.sh
```

Analyse the frame. The door should be roughly ahead. Log estimated bearing offset from centre:
```
start: door_offset=<deg> confidence=<0-1>
```

- If door is visible and offset ≤ 30° — fine-tune with a small `turn <offset>` and go to Phase 2 immediately.
- If door not clearly visible yet — do a single 45° sweep left then right (90° total) to find it, pick best, commit. No more scanning.

Narrate: `echo "Target acquired. I need your clothes, your boots, and your door." >> /tmp/speak_queue.txt`

---

### Phase 1 — Fallback Scan (only if door not found after Phase 0)

If Phase 0 frame analysis fails to locate the door, spin and capture **4 frames at 90° increments** — the door is somewhere in this half of the room:

```bash
for i in 1 2 3 4; do
  ./roomba_pilot/target/debug/pilot send "turn 90"
  source ./scripts/capture_frame.sh
done
```

Pick best bearing, commit, proceed to Phase 2. No third scan ever.

---

### Phase 2 — Full-Speed Dash

Tight aggressive loop — **long strides, minimal stops**:

1. **Turn** to align with door bearing: `pilot send "turn <deg>"`
2. **Charge 60 cm** at full speed: `pilot send "move 60"`
3. Capture frame, re-estimate door bearing and distance
4. **Check bumpers** (non-blocking): `pilot send "bumps"`

**On bump — don't retreat, redirect:**
- Identify which side hit (left/right bumper)
- Turn **45° away** from the bump side and immediately continue
- Skip the backward move — lost distance is lost time
- `echo "Obstacle detected. Your move, creep." >> /tmp/speak_queue.txt`

**If door is lost from view:**
- Spin 90° in the last-known direction, recapture, reassess
- If still not found, run a full 180° mini-sweep, pick best candidate, commit
- Log: `reacquire: new_bearing=<deg>`

**Keep moving.** Every frame analysis is a decision — increment the counter and keep the loop hot. Target: ≤3 seconds per loop iteration.

---

### Phase 3 — Threshold Burst

When the door fills **>40%** of the frame (lower threshold = commit earlier):

```bash
echo "I see it. Get out." >> /tmp/speak_queue.txt
./roomba_pilot/target/debug/pilot send "forward 30 3"   # 3-second full-speed blast through the frame
```

After the burst, immediately capture a frame. If the scene has clearly changed (different wall colour, hallway, new room, open space) — **escaped**.

```bash
echo "Hasta la vista, room." >> /tmp/speak_queue.txt
./scripts/finish_run.sh escaped
```

If scene is ambiguous, do one more 30 cm burst and check again.

---

### Phase 4 — Wall-Following Fallback (if door not found after 2 min)

If Phase 1–2 haven't located the door within **2 minutes**, switch to a deterministic wall-hug:

1. Drive toward the nearest wall until a bump triggers
2. Turn 90° clockwise
3. Drive forward 50 cm hugging the wall (slight clockwise bias: `go 25 -5`)
4. Capture every 50 cm and scan for door
5. Repeat — a room has 4 walls; the door is on one of them

This guarantees coverage of every wall segment rather than spinning in the middle.

```bash
echo "Come with me if you want to find the door." >> /tmp/speak_queue.txt
```

---

### Abort — Hard Time Limit (9 minutes)

If elapsed time exceeds 9 minutes with no escape confirmed:

```bash
echo "My CPU is a neural net processor. But this room... has won." >> /tmp/speak_queue.txt
./scripts/finish_run.sh dnf
```

---

### Decision Principles

| Situation | Action |
|---|---|
| Start of run | Turn 180° immediately — door is directly behind |
| Confidence ≥ 0.35 on door bearing | Commit and charge — no re-scan |
| Bump detected | Redirect 45°, keep moving — never back up |
| Door lost mid-run | 90° sweep, reacquire, resume — max 10 s lost |
| Door fills >40% frame | Burst through immediately |
| 2 min elapsed, no door | Wall-follow mode |
| 9 min elapsed | DNF |