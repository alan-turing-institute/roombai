# RoombaI — CLAUDE.md

## Goal

You are controlling a Roomba cleaning robot. A camera is mounted on it.
The task is to escape the room via the door in the picture below as fast as possible. You have maximum ten minutes.

<img width="1200" height="1600" alt="Door reference" src="./door_reference.jpg" />

> **Read this image before every run.** It shows the actual room and the exit opening. The exit is the floor-level gap to the **LEFT of the large wooden board/panel** — not the board itself, not the wooden door with the lever handle. Look for where the carpet continues through into the next space.

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

During escape runs, prefer these wrappers:

```bash
./scripts/pilot_send.sh "move 200"
./scripts/pilot_macro.sh "turn 25" "move 200" "move 200" "move 80"
./scripts/mark_decision.sh
```

- `pilot_send.sh` sends one command and increments `TOOL_CALLS`.
- `pilot_macro.sh` sends several commands, increments `TOOL_CALLS` for each, and waits for timed motion commands to finish before sending the next command.
- `mark_decision.sh` increments `DECISIONS`.

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

After each direct `pilot send` command, increment the tool call counter:
```bash
sed -i "s/^TOOL_CALLS=.*/TOOL_CALLS=$(( $(grep TOOL_CALLS /tmp/run_state.env | cut -d= -f2) + 1 ))/" /tmp/run_state.env
```

Prefer `./scripts/pilot_send.sh` or `./scripts/pilot_macro.sh`; they do this automatically.

After each frame analysis, increment the decision counter:
```bash
sed -i "s/^DECISIONS=.*/DECISIONS=$(( $(grep DECISIONS /tmp/run_state.env | cut -d= -f2) + 1 ))/" /tmp/run_state.env
```

Prefer `./scripts/mark_decision.sh`; it does this automatically.

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

**Objective:** reach the door as fast as possible. **Thinking time and camera stops are the main enemy.** Once the door is located, commit fully — chain all move commands without stopping to photograph. Only pause if a bump fires or the robot is within 30 cm of the threshold.

Be careful, the roomba is not well calibrated. When you think you rotated 360 you actually rotated 320. So consider that in every move you make and make corrections.

### Roomba dimensions
- Diameter: ~34 cm, moves at 20 cm/s, turns at 60°/s
- A 200 cm move takes 10 s. A 90° turn takes 1.5 s. Minimise turns, maximise straight-line distance per move.

---

### Non-negotiables

- Put the robot in SAFE mode before the first movement command:
  ```bash
  ./scripts/pilot_send.sh "safe"
  ```
- Count every `pilot send`, including `safe`, `bumps`, `stop`, and recovery commands.
- You have a hard 10 minute limit. The run should escape in under 2 minutes if the path is clear.
- Spend at most 5-10 seconds analysing a frame. Make a reasonable estimate and move.
- **Maximum 3 captures in a clean run: (1) orient at start, (2) one mid-run check only if a bump fired, (3) escaped confirmation.** Every extra capture costs ~60 seconds of analysis time — that is time not moving.
- Once the door is visible and the heading is set, issue the full sequence of move commands back-to-back without stopping to photograph.
- **Never stop mid-run to re-analyse unless a bumper fires.** If you think you might be drifting, keep going — a small heading error is cheaper than a camera stop.
- Do not call `bumps` after every clean move. Use it only after suspected contact or wheel-drop.
- Queue TTS briefly and keep moving; do not wait for narration to finish.
- Use `pilot_macro.sh` for multi-command movement sequences so Claude spends one tool call on the route instead of thinking between every move.
- Keep at least one robot radius (~20 cm) of planned clearance from obstacles and doorposts. If the corridor is narrower than that, slow down and shorten moves.
- If the camera read and bumper state disagree, trust the physical sensor first: stop, back off, capture, then re-plan.

### Decision loop — use ONLY at the two permitted capture points

**There are exactly two capture points in a clean run:**
1. **Start** — orient, locate door, set heading.
2. **Bump recovery only** — if a bumper fires, capture once to re-orient, then go straight back to moving.

Everything else is movement. Do not capture between moves. Do not capture "just to check". Do not capture after a turn. Move.

The loop:

1. **Capture frame**
   ```bash
   source ./scripts/capture_frame.sh
   ```

2. **Analyse — spend at most 10 seconds.** Identify:
   - Door location and which side of any wooden board/panel the opening is on (opening is to the LEFT of the board)
   - Roomba heading and angle delta to door
   - Rough distance to door and any obstacles in the direct path

3. **Increment decision counter**
   ```bash
   ./scripts/mark_decision.sh
   ```

4. **Narrate via TTS** (one short sentence, do not wait)
   ```bash
   echo "<decision summary>" >> /tmp/speak_queue.txt
   ```

5. **Issue the full movement sequence** — turn once to align, then chain ALL move commands to the door without pausing:
   ```bash
   ./scripts/pilot_macro.sh "turn <deg>" "move 200" "move 200" "move 80"
   ```
   The macro increments the tool call counter for each command and waits for each move/turn to finish. Do **not** capture between these moves.

6. **Check bumps only if something felt wrong** — unexpected stop, loud contact, no forward progress:
   ```bash
   ./scripts/pilot_send.sh "bumps"
   ```
   If bumper fired → bump recovery (see below). Otherwise keep moving.

---

### Phase 1 — Orient (target < 30 s)

**Step 0 — study the reference image before touching the robot:**
```bash
# Read the reference image into your context NOW, before capturing any live frame
Read("./door_reference.jpg")
```
Look at it carefully. Identify:
- The large wooden board/panel and the **glass partition** — the exit is the **open carpet gap between them**
- The lever-handle wooden door — memorise its appearance so you can **ignore it** during the run
- The rough layout: which wall the exit is on, what is to its left and right

You now have a mental map of the room. Every live frame you capture should be matched against this reference.

**Step 1 — enter SAFE mode and capture the first live frame:**
- Enter SAFE mode.
- Capture first frame.
- **If it does not show the full room** (e.g. facing a wall or corner), back off 30 cm and `turn 180` before recapturing.
- **If no exit gap is visible from the first usable frame**, do a 360° survey: four `turn 90` commands with a capture after each. Do this once only.

**Step 2 — identify the exit gap, not the lever door:**
- Find the carpet gap **between the wooden board and the glass partition** — that is your target.
- Ignore the lever-handle door. Ignore glass walls. Ignore server racks.
- If you are unsure, compare against `door_reference.jpg`.

**Step 3 — align and go:**
- Compute the angle delta to face the exit gap centreline.
- Issue a single `turn <deg>` to align.
- Immediately chain all move commands to the door — do not stop again unless a bumper fires.

### Phase 2 — Drive to door (bulk of run)

**From the moment the door is located, do not stop until you are through it or a bumper fires.**

Turn once to align, then chain moves all the way to the threshold:

```bash
./scripts/pilot_macro.sh "turn <angle-to-door>" "move 200" "move 200" "move 80" "move 80"
```

Do not capture between these commands. Do not stop to check heading. If the room is roughly the size shown, two `move 200` commands plus a final `move 80` will get the robot through the door from most starting positions.

If an obstacle is visible in the orient frame, pick the wider gap side, chain: `turn <skirt angle>` → `move <clear distance>` → `turn <re-aim angle>` → `move <remaining distance>`. Do all of this without capturing.

### Identifying the exit — read this before every run

**The exit is NOT the wooden door with the lever handle.** That door is closed and latched — it cannot be pushed open by the Roomba. Do not approach it.

**The exit IS the floor-level gap between the large wooden board/panel and the glass partition.** Carpet continues through it into the next space. It looks like an open corridor opening — no handle, no frame to push, just open floor.

How to tell them apart when analysing a frame:
- **Lever-handle wooden door** = reject. It is a dead end.
- **Dark rectangular gap at floor level with carpet visible beyond it** = this is the exit. Go through it.
- **Glass partition / glass wall** = solid, not passable.
- **Server rack or black equipment beside a wall** = not an opening, just furniture.

If you are unsure which feature is the exit, look at the **reference image** (`./door_reference.jpg`) — it shows the exact room layout with the exit gap marked.

### Phase 3 — Thread the door (last ~1 m, no stops)

- **Do not capture.** You already know where the door is. Keep moving.
- Target the **floor-level carpet gap to the LEFT of the wooden board** — not the lever-handle door, not the glass wall.
- Align to the gap centreline. At the threshold, being centred matters more than being perfectly square.
- Align to the door centreline. At the threshold, being centred matters more than being perfectly square.
- Use short, deliberate moves, but do not stop between them if the first threshold push is clean:

| Situation | Command |
|---|---|
| 80 - 140 cm from threshold and centred | `./scripts/pilot_send.sh "move 80"` |
| 60 - 80 cm from threshold and centred | `./scripts/pilot_send.sh "move 60"` |
| 20 - 60 cm from threshold and centred | `./scripts/pilot_send.sh "move 50"` |
| Front at or just across threshold | `./scripts/pilot_send.sh "move 80"` |

- If the robot is offset toward one jamb, turn 10-20° away from that jamb, move 20-30 cm, then turn back toward the doorway.
- Once the front crosses the threshold, the priority is clearing the rear of the robot. Continue straight immediately — do not capture again under any circumstances until you are calling `finish_run.sh`.

---

### Bump recovery

If bumpers fire mid-move:

```bash
./scripts/pilot_send.sh "move -20"   # back off
source ./scripts/capture_frame.sh     # re-assess from image
./scripts/mark_decision.sh
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
./scripts/pilot_send.sh "stop"
./scripts/finish_run.sh dnf
```
