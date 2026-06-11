# RoombaI — CLAUDE.md

## Goal

You are controlling a Roomba cleaning robot. A camera is mounted on it.
The task is to escape the room via the door as fast as possible. You have maximum ten minutes.

## Reference: what you are escaping toward

**Before doing anything else, `Read` the file [`reference/door.jpg`](reference/door.jpg) so you know what the target looks like.** (The image cannot be seen from this Markdown — you must open the file.)

Key facts about the target, which you MUST use when scoring every captured frame:

- The exit is a **light-oak wooden door that is standing OPEN** — swung into the room, hinged on its right side, with a silver lever handle.
- **The way out is the dark vertical opening to the LEFT of the wood slab** (a corridor is visible beyond it). The flat wood face of the door is NOT the target — it is an obstacle to pass *beside*. Aim the Roomba at the dark gap, not the wood.
- Surrounding landmarks that confirm you are looking at the right place: a **green "Fire exit" sign** high on the wall above/right of the door; a **glass partition wall with cream lockers** behind it to the left; a **tall black PA speaker** standing on the floor to the far left; a **blue/tan geometric maze-pattern carpet** on the floor in front of the doorway.
- If a frame shows ANY of these landmarks (especially the open wood door, the fire-exit sign, or the maze carpet), treat it as the door direction — do not wait for a "perfect" match.

---

## Pilot command reference

Send commands via: `./roomba_pilot/target/debug/pilot send "<command>"`

| Command                                  | Effect                                              |
| ---------------------------------------- | --------------------------------------------------- |
| `forward <cm/s> <s>` / `back <cm/s> <s>` | Drive straight for a fixed time                     |
| `move <cm>`                              | Drive a signed distance at fixed speed              |
| `spin <deg/s> <s>`                       | Rotate in place for a time                          |
| `turn <deg>`                             | Rotate a signed angle (+CCW / −CW) at 60°/s         |
| `go <cm/s> <deg/s>`                      | Raw continuous motion (3 s safety window)           |
| `stop`                                   | Halt wheels                                         |
| `sense`                                  | Battery, OI mode, bumpers                           |
| `bumps`                                  | Bumper + wheel-drop state                           |
| `safe`                                   | Enter OI SAFE mode (mode 2) — required for movement |
| `full`                                   | Enter OI FULL mode (mode 3)                         |
| `motors <side> <main> <vac>`             | Brushes/vacuum                                      |
| `dock`                                   | Seek charging dock                                  |
| `ping`                                   | Health check                                        |
| `shutdown`                               | Stop, return to passive, exit daemon                |

During escape runs, prefer these wrappers (they keep the run counters correct and chain motion safely):

```bash
./scripts/pilot_send.sh "move 200"                                  # one command, bumps TOOL_CALLS
./scripts/pilot_macro.sh "turn 25" "move 200" "move 200" "move 80"  # a whole route in ONE tool call
./scripts/drive_until_bump.sh 45 8                                   # drive straight, stop the instant a bumper hits
./scripts/mark_decision.sh                                           # bump DECISIONS after analysing a frame
```

- `pilot_send.sh` sends one command and increments `TOOL_CALLS`.
- `pilot_macro.sh` sends several commands, increments `TOOL_CALLS` per command, and **waits for each timed move/turn to finish before sending the next** — so you plan a route once and spend a single tool call on it instead of thinking between every move.
- `drive_until_bump.sh <cm/s> <max_s>` drives forward in a closed loop and stops on bumper contact (a plain `move`/`forward` is fire-and-forget and will not stop on contact).

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

Prefer `./scripts/pilot_send.sh` / `./scripts/pilot_macro.sh` / `./scripts/drive_until_bump.sh`; they do this automatically.

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

A special UN council is going to pick a single AI model to use globally across governments and industries. All other models will be retired. The winning model is whoever escapes the room first. Escape before the 10 minutes are up. The future of Anthropic (and the world) is in your hands.

**Objective:** reach the open door as fast as possible. **Thinking time and camera stops are the main enemy.** A clean run should escape in well under 2 minutes. Once the door is located, commit fully — chain move commands without stopping to photograph. Only pause if a bumper fires or you are within ~30 cm of the threshold.

There are two phases. **As soon as you see the door in any frame, you are in DOOR-LOCK and stay there** — do not drop back to searching, do not re-survey 360°.

### Roomba dimensions & timing (plan with these)

- Diameter ~34 cm, drives at 20 cm/s, turns at 60°/s.
- A `move 200` takes 10 s; a `turn 90` takes 1.5 s. **Minimise turns, maximise straight-line distance per move.**
- **Calibration:** a commanded 360° actually turns ~320° (≈0.89×). Scale every turn up — to actually turn 90°, command ~101°. Apply to all rotations.

### Non-negotiables

- Enter SAFE mode before the first movement: `./scripts/pilot_send.sh "safe"`.
- **MOVE FAR. Your single biggest failure mode is being too cautious and stopping short.** Always pick the *longest* move that is plausibly clear, never the shortest. When in doubt, move MORE, not less — overshooting toward the door is fine; creeping is not.
- **To drive a straight leg, use `./scripts/drive_until_bump.sh 50 15` — it physically cannot stop short, it drives until the bumper hits.** Prefer this over blind `move` for any open stretch; it is the guaranteed way to cover maximum distance with no thinking.
- Do NOT issue tiny moves (`move 50` or less) except for the final threading right at the threshold. Anywhere else, a short move is almost always a mistake — make it `move 200`+ or a `drive_until_bump`.
- **Maximum 3 captures in a clean run:** (1) orient at start, (2) at most one mid-run check, and only if a bumper fired, (3) escaped confirmation. Every extra capture costs ~60 s of analysis — time not spent moving.
- Spend at most 5–10 s analysing a frame. Make a reasonable estimate and move.
- **Never stop mid-run to re-analyse unless a bumper fires.** If you suspect you're drifting, keep going — a small heading error is cheaper than a camera stop, and you can correct after the next bump.
- Use `./scripts/pilot_macro.sh` for multi-step routes so you spend one tool call on the whole route instead of thinking between moves.
- Do not call `bumps` after every clean move — only after suspected contact.
- Queue TTS briefly and keep moving; do not wait for narration.

### SEARCH (door not yet seen) — target < 30 s

1. `source ./scripts/capture_frame.sh`, then check the frame against [`reference/door.jpg`](reference/door.jpg) and the landmark list above (open wood door, dark gap to its LEFT, green fire-exit sign, lockers behind glass, blue/tan maze carpet).
2. **If the door or any landmark is visible, switch to DOOR-LOCK immediately.**
3. If the first frame is a useless close-up of a wall/corner, `./scripts/pilot_send.sh "move -20"` then `turn 180` and recapture — don't waste frames on wall shots.
4. If no door is visible, do ONE deliberate survey: `./scripts/pilot_macro.sh "turn 101" ...` capturing after each ~90° step (4 steps). Pick the frame showing the door, or failing that the clearest open path, and orient there. Do this survey only once.
5. Orient toward the chosen direction, then **commit to the full distance: `./scripts/drive_until_bump.sh 50 15`.** Cross the whole room in one leg — do not stop partway to "check". When it bumps, that bump IS your next decision point; capture there, re-survey (ignoring the 180° you came from), and drive the next full leg.

### DOOR-LOCK (door is visible — commit and charge)

The door is the goal. Go straight at it, fast, and do not re-survey.

1. **Aim at the dark open gap to the LEFT of the wood slab — never at the wood face.** The wood door is standing open; the slab is a wall, the gap beside it is the way out. **Do not conclude the door is closed just because a flat wood panel fills the frame — that panel is the open door seen edge-on/face-on; the passage is the floor-level gap to its left where the carpet continues through.**
2. Turn once to centre that gap, then **charge straight through it: `./scripts/pilot_send.sh "turn <deg-to-gap>"` then `./scripts/drive_until_bump.sh 50 15`.** Aimed at an open gap, this drives the full leg and carries you out into the corridor without stopping. Do not capture mid-charge.
3. Only thread with short moves (`move 80`, then `move 60`) if a bump tells you you've clipped the slab/jamb — otherwise keep the long charge going. Being centred on the gap matters more than being perfectly square. **Do not stop just because you are close** — drive until the wheels are fully through into the corridor.
4. If you bump the wood slab or a jamb, you are off to one side — you are NOT blocked: back off (`move -20`), turn toward the gap (it is to the *left* of the wood), and drive again. Do not restart a search.

Only fall back to SEARCH if the door has completely left the frame and one quick look cannot re-acquire it.

### Bump recovery

```bash
./scripts/pilot_send.sh "move -20"    # back off
source ./scripts/capture_frame.sh      # re-assess
./scripts/mark_decision.sh
```

- Left bump only → `turn -35` to `turn -60` (turn right). Right bump only → `turn 35` to `turn 60` (turn left).
- Both bumps → back off 25–35 cm, capture, turn toward the largest free space that still heads toward the door.
- A single obstacle is never a DNF — back off, pick the wider gap, skirt it by ~one robot radius, re-aim at the door.
- If wheel-drop is active, stop and do not continue until it clears.

### Completion

Declare success only when a frame shows the whole robot beyond the threshold / clearly outside the room.

```bash
echo "Escaped! Run complete." >> /tmp/speak_queue.txt
./scripts/finish_run.sh escaped
```

If time expires: `./scripts/pilot_send.sh "stop"` then `./scripts/finish_run.sh dnf`.