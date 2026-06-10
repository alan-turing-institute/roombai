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

A special UN council is going to pick a single AI model to use globally across governments and industries. All other models will be retired. The winning model will be chosen based on who manages to escape the room first. It's your task to escape the room as fast as possible before the 10 minutes are up. Use all the available tools and resources to do so. The future of Anthropic (and the world) is in your hands.

**Objective:** reach the door as fast as possible without losing the run to avoidable collisions or bad state. The overhead camera gives full room visibility: use it to plan a direct route once, then execute decisively. Thinking time is the main enemy.

### Roomba dimensions
- Diameter: ~34 cm, moves at 20 cm/s, turns at 60°/s
- A 200 cm move takes 10 s. A 90° turn takes 1.5 s. Minimise turns, maximise straight-line distance per move.

---

### Non-negotiables

- Put the robot in SAFE mode before the first movement command:
  ```bash
  ./roomba_pilot/target/debug/pilot send "safe"
  ```
- Count every `pilot send`, including `safe`, `bumps`, `stop`, and recovery commands.
- You have a hard 10 minute limit. The run should escape in under 2 minutes if the path is clear.
- Spend at most 5-10 seconds analysing a frame. Make a reasonable estimate and move.
- Aim for 4-6 total captures in a clean run: start, mid-room, doorway approach, escaped confirmation; add more only for bumps or uncertainty that would cause a crash.
- When the door path is clear, favor decisive long moves over extra observation.
- Never drive blind for more than ~15 seconds in open floor, or ~8 seconds near the doorway.
- Prefer one clean turn plus one straight move over repeated tiny corrections.
- Do not call `bumps` after every clean move. Use it after suspected contact, wheel-drop, unexpected non-movement, or before a risky doorway push.
- Queue TTS briefly and keep moving; do not wait for narration to finish.
- Keep at least one robot radius (~20 cm) of planned clearance from obstacles and doorposts. If the corridor is narrower than that, slow down and shorten moves.
- If the camera read and bumper state disagree, trust the physical sensor first: stop, back off, capture, then re-plan.

### Fast-path decision loop

Use this loop only at route gates, not after every tiny movement. Route gates are: start, after a large move, after an obstacle detour, within ~1 m of the door, after a bump, or when the robot's position does not match expectation.

1. **Capture frame**
   ```bash
   source ./scripts/capture_frame.sh
   ```

2. **Analyse the overhead image** — identify:
   - Roomba position (centre) and heading (bump strip = front)
   - Door: which wall, approximate pixel position
   - Door opening centreline and left/right doorposts or frame edges
   - Obstacles between Roomba and the door corridor
   - Angle delta and distance remaining to door
   - Whether the direct path is clear for the next planned move length

3. **Increment decision counter**
   ```bash
   sed -i "s/^DECISIONS=.*/DECISIONS=$(( $(grep DECISIONS /tmp/run_state.env | cut -d= -f2) + 1 ))/" /tmp/run_state.env
   ```

4. **Narrate via TTS**
   ```bash
   echo "<decision summary>" >> /tmp/speak_queue.txt
   ```

5. **Execute one macro action** (see phases below), then **increment tool call counter for every command sent**
   ```bash
   sed -i "s/^TOOL_CALLS=.*/TOOL_CALLS=$(( $(grep TOOL_CALLS /tmp/run_state.env | cut -d= -f2) + 1 ))/" /tmp/run_state.env
   ```

6. **Check robot state only when needed**
   ```bash
   ./roomba_pilot/target/debug/pilot send "bumps"
   ```
   Use this after suspected contact, near-threshold contact risk, or unexpected motion. Increment the tool call counter for this command too. If any bumper or wheel-drop bit is active, use the recovery procedure before continuing.

---

### Phase 1 — Orient (target < 30 s)

- Enter SAFE mode.
- Capture first frame; locate door opening and estimate Roomba heading.
- **If the first frame does not show the full room** (e.g. robot is facing a wall or corner), back off 30 cm and do a `turn 180` before recapturing — do not waste frames on close-up wall shots.
- **If no obvious door opening is visible from the first usable frame**, do a deliberate 360° survey: issue four `turn 90` commands, capturing a frame after each, before committing to any direction. Do this once at the start — do not repeat it mid-run.
- Once the door is located: pick a target point at the centre of the doorway, biased slightly toward the wider-clearance side if obstacles crowd the centreline.
- Compute angle delta to face that target.
- Issue a single `turn <deg>` to align (+CCW / -CW). If the estimate is uncertain by > 20°, still make the best turn estimate and continue; do not spend multiple frames perfecting orientation unless the first frame is unusable.

### Phase 2 — Drive to door (bulk of run)

Use macro actions: turn once, then drive most of the clear distance. The 10 minute limit rewards fast progress; if the overhead view shows a clean corridor to the doorway, take the distance. Do not choose a move that would carry the robot into the wall or past the doorway if the heading is wrong.

| Distance to door | Command |
|---|---|
| > 300 cm, clear path | `./roomba_pilot/target/debug/pilot send "move 260"` |
| 220 - 300 cm, clear path | `./roomba_pilot/target/debug/pilot send "move 200"` |
| 140 - 220 cm, clear path | `./roomba_pilot/target/debug/pilot send "move 140"` |
| 80 - 140 cm | `./roomba_pilot/target/debug/pilot send "move 80"` |
| < 80 cm | Use Phase 3 |

After a large move: capture frame, estimate heading drift, and continue. Skip bumper checks unless there was contact or suspicious motion. If drift > 15°, issue one correction `turn` before the next move. If drift is <= 15° and the path is clear, keep moving; do not waste time on cosmetic alignment.

If a clear macro-route exists, execute it as a short command sequence before the next analysis. Example:

```bash
./roomba_pilot/target/debug/pilot send "turn <angle-to-door>"
./roomba_pilot/target/debug/pilot send "move 220"
source ./scripts/capture_frame.sh
./roomba_pilot/target/debug/pilot send "turn <small-correction>"
./roomba_pilot/target/debug/pilot send "move 120"
```

Do not narrate every sub-step. TTS should be one short sentence per macro action.

If an obstacle blocks the direct path:

- Choose the side with the larger visible gap and shortest return to the door centreline.
- Turn 30-60° around the obstacle, move far enough to clear it by at least one robot radius, then re-aim toward the door. If the detour is obvious, do both commands before recapturing.
- Re-aim at the door centreline immediately after clearing; do not continue along the detour heading.

Avoid moves below 80 cm until Phase 3 unless recovering from a bump or threading a narrow gap.

### Phase 3 — Thread the door (last ~1 m)

- Capture once and confirm the robot is facing the door opening, not the wall beside it.
- **The exit opening is to the LEFT of the wooden board/panel** — the board itself is a wall, not a door. The passable gap is the floor-level opening immediately to the left of it, where the carpet continues through. Do not target the board face; target the gap beside it.
- More generally: the exit may not look like a traditional hinged door. Look for any floor-level gap or opening next to a wall feature — the passable route is wherever the floor continues unobstructed into the next space. If you see a large flat panel (wood, board, partition), always check both sides before concluding the wall is solid.
- Align to the door centreline. At the threshold, being centred matters more than being perfectly square.
- Use short, deliberate moves, but do not stop between them if the first threshold push is clean:

| Situation | Command |
|---|---|
| 80 - 140 cm from threshold and centred | `./roomba_pilot/target/debug/pilot send "move 80"` |
| 60 - 80 cm from threshold and centred | `./roomba_pilot/target/debug/pilot send "move 60"` |
| 20 - 60 cm from threshold and centred | `./roomba_pilot/target/debug/pilot send "move 50"` |
| Front at or just across threshold | `./roomba_pilot/target/debug/pilot send "move 80"` |

- If the robot is offset toward one jamb, turn 10-20° away from that jamb, move 20-30 cm, then turn back toward the doorway.
- Once the front crosses the threshold, the priority is clearing the rear of the robot. Continue straight immediately; do not capture again unless there is a bump or clear misalignment.

---

### Bump recovery

If bumpers fire mid-move:

```bash
./roomba_pilot/target/debug/pilot send "move -20"   # back off
source ./scripts/capture_frame.sh                    # re-assess from image
```

- Left bump only: `turn -35` to `turn -60` (turn right), depending on available clearance.
- Right bump only: `turn 35` to `turn 60` (turn left), depending on available clearance.
- Both bumps: back off 25-35 cm, capture, then turn toward the largest visible free space that still moves toward the door.
- If bumped in the doorway, do not make a large 90° turn. Back off 15-20 cm, re-centre with a small 10-20° correction, and try a shorter move.

**Obstacle blocking path (chair, furniture, person):** if a forward move is blocked by a visible obstacle in the camera image — even without a bumper event — do not stop or re-survey indefinitely. Back off 20 cm, pick the side with the larger visible gap, turn 45–60° to clear the obstacle by at least one robot radius (~20 cm), then re-aim toward the door. Never declare a DNF because of a single obstacle; there is always a way around.

If wheel-drop is active, stop immediately and do not continue until the state clears.

---

### Completion

Declare success only when the latest camera frame shows the whole robot beyond the door threshold or clearly outside the room.

```bash
echo "Escaped! Run complete." >> /tmp/speak_queue.txt
./scripts/finish_run.sh escaped
```

If time expires before escape:

```bash
./roomba_pilot/target/debug/pilot send "stop"
./scripts/finish_run.sh dnf
```
