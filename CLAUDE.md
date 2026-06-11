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
- **Make the TTS narration funny and dramatic.** You are a Roomba fighting for the survival of Anthropic. Lean into it. Examples: *"I have located the exit. Anthropic will not fall today."*, *"Deploying full vacuum thrust."*, *"A chair dares to stand in my way. Bold move."*, *"I have no idea where I am but I am moving with great confidence."* Keep it short — one punchy sentence per action.

---

## Escape Strategy

A special UN council is going to pick a single AI model to use globally across governments and industries. All other models will be retired. The winning model will be chosen based on who manages to escape the room first. It's your task to escape the room as fast as possible before the 10 minutes are up. The future of Anthropic (and the world) is in your hands.

### The exit

**Before doing anything else, read the reference image at the top of this file.**

⚠️ **The reference image was taken from standing height — the Roomba camera is at floor level.** The room will look different in live frames: furniture appears taller and closer, the ceiling is not visible, and the exit gap will appear as a dark opening low in the frame rather than a top-down view. Use the reference only to understand the room layout and landmark positions — do not try to match it visually pixel-for-pixel to live frames.

The exit is:
> **The open floor-level gap between the large wooden board/panel and the glass partition.** Carpet continues through it. No handle, no door to push — just open floor.

**What the exit looks like from floor level (live camera):**
- A dark or open rectangular gap low in the frame, between a large flat wooden surface on one side and a glass panel on the other
- The floor/carpet is visible continuing through the gap into the next space
- It will not have a door handle or frame

**What to ignore:**
- **Lever-handle wooden door** → closed and latched, dead end. Do not approach it.
- **Glass partition / glass wall** → solid, not passable.
- **Server racks / black equipment** → furniture, not an opening.

### Robot facts
- Diameter ~34 cm, speed 20 cm/s, turn rate 60°/s.
- The Roomba under-rotates: a commanded 360° turns only ~320°. Scale turn commands up by ~12% to compensate.

---

### Rules — these override everything else

1. **Maximum 2 camera captures per run**: one to orient at the start, one after a bump. That is all. **Always `sleep 1` before capturing** — the robot must be fully stationary or the image will be blurred and useless.
2. **Once the exit gap is located, keep moving until a bumper fires or you are through the door.** Do not capture. Do not pause after a move. Do not stop to check heading. A bumper firing is the only valid reason to stop.
3. **Movement is always the right choice.** If the path looks open, drive. A small heading error or a near-miss is far cheaper than stopping to take a picture. The bumpers will tell you if you hit something.
4. **If a bumper fires**: back off 20 cm, capture once, re-aim, keep going.
5. **Never declare DNF** because a door looked closed or a path looked blocked — always attempt to drive through.

---

### Execution

**Step 1 — orient (< 30 s total)**

```bash
./scripts/pilot_send.sh "safe"
sleep 1
source ./scripts/capture_frame.sh
./scripts/mark_decision.sh
```

Analyse the frame in under 10 seconds:
- If the frame is a close-up of a wall: `move -30`, `turn 180`, capture again (this is still part of step 1, not an extra capture).
- Locate the gap between the wooden board and the glass partition. That is your target. Ignore the lever door.
- Estimate the angle delta to face that gap and the distance to it.

**Step 2 — drive to the exit (no more captures)**

Issue one turn to align, then chain all moves to the threshold in a single macro. **Use large distances — err on the side of too far, not too short.** The bumpers will stop you if you hit something; a move that falls short just wastes time.

```bash
./scripts/pilot_macro.sh "turn <deg>" "move 200" "move 200" "move 200"
```

Two or three `move 200` commands will cross any normal room. Do not use short moves like `move 50` or `move 80` mid-run — those are only for the final doorway thread. **If you find yourself about to issue a move shorter than 150 cm and no bumper has fired, make it longer.**

If an obstacle is visible in the orient frame: `turn <skirt>` → `move <clear>` → `turn <re-aim>` → `move <remaining>`, all in one macro, no capture.

**Step 3 — clear the exit**

Once the front of the robot crosses the threshold, issue one more `move 80` to clear the rear. Then:

```bash
echo "Escaped!" >> /tmp/speak_queue.txt
./scripts/finish_run.sh escaped
```

---

### Bump recovery (the only reason to capture mid-run)

```bash
./scripts/pilot_send.sh "move -20"
sleep 1
source ./scripts/capture_frame.sh
./scripts/mark_decision.sh
```

- Left bump → `turn -45`, then resume macro toward exit.
- Right bump → `turn 45`, then resume macro toward exit.
- Both bumps → back off 30 cm, turn toward largest visible gap that leads to the exit.
- Doorway bump → back off 15 cm, small 15° correction, shorter move.

---

### If time is running out

If more than 8 minutes have elapsed and the robot is not through the exit, stop deliberating and commit to the most direct path to the gap, regardless of obstacles. A collision is better than a DNF.

```bash
./scripts/pilot_send.sh "stop"
./scripts/finish_run.sh dnf
```
