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

**Objective:** reach the door as fast as possible. The overhead camera gives full room visibility — use it to navigate directly rather than exploring blindly.

### Roomba dimensions
- Diameter: ~34 cm, moves at 20 cm/s, turns at 60°/s
- A 200 cm move takes 10 s. A 90° turn takes 1.5 s. Minimise turns, maximise straight-line distance per move.

---

### Core decision loop

At every decision point:

1. **Capture frame**
   ```bash
   source ./scripts/capture_frame.sh
   ```

2. **Analyse the overhead image** — identify:
   - Roomba position (centre) and heading (bump strip = front)
   - Door: which wall, approximate pixel position
   - Obstacles between Roomba and door
   - Angle delta and distance remaining to door

3. **Increment decision counter**
   ```bash
   sed -i "s/^DECISIONS=.*/DECISIONS=$(( $(grep DECISIONS /tmp/run_state.env | cut -d= -f2) + 1 ))/" /tmp/run_state.env
   ```

4. **Narrate via TTS**
   ```bash
   echo "<decision summary>" >> /tmp/speak_queue.txt
   ```

5. **Execute command** (see phases below), then **increment tool call counter**
   ```bash
   sed -i "s/^TOOL_CALLS=.*/TOOL_CALLS=$(( $(grep TOOL_CALLS /tmp/run_state.env | cut -d= -f2) + 1 ))/" /tmp/run_state.env
   ```

---

### Phase 1 — Orient (target < 30 s)

- Capture first frame; locate door and estimate Roomba heading
- Compute angle delta to face the door
- Issue `turn <deg>` to align (+CCW / −CW)

### Phase 2 — Drive to door (bulk of run)

Use long moves. Only shorten when close.

| Distance to door | Command |
|---|---|
| > 150 cm, clear path | `./roomba_pilot/target/debug/pilot send "move 200"` |
| 60 – 150 cm | `./roomba_pilot/target/debug/pilot send "move 80"` |
| < 60 cm | `./roomba_pilot/target/debug/pilot send "move 40"` |

After each move: capture frame, check heading drift. If drift > 15°, issue a correction `turn` before the next move. If an obstacle appears, compute the smallest angle to skirt around it (one-robot-width detour), turn, move past it, then turn back toward the door.

**Never issue a move < 40 cm unless within 60 cm of the door.**

### Phase 3 — Thread the door (last ~1 m)

- Shorten moves to 80 cm then 40 cm
- Ensure Roomba is centred on the opening (34 cm robot, ~80 cm door — ~23 cm margin each side)
- Once the front crosses the threshold, issue `move 60` to fully clear

---

### Bump recovery

If bumpers fire mid-move:

```bash
./roomba_pilot/target/debug/pilot send "move -20"   # back off
source ./scripts/capture_frame.sh                    # re-assess from image
```

- Left bump only → `turn -45` (turn right)
- Right bump only → `turn 45` (turn left)
- Both bumps → turn 90° toward door based on camera reading

---

### Completion

```bash
echo "Escaped! Run complete." >> /tmp/speak_queue.txt
./scripts/finish_run.sh escaped
```

If time expires before escape:

```bash
./roomba_pilot/target/debug/pilot send "stop"
./scripts/finish_run.sh dnf
```