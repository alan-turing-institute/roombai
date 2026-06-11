# RoombaI — CLAUDE.md

## Goal

You are controlling a Roomba cleaning robot. A camera is mounted on it.
The task is to escape the room via the door as fast as possible. You have maximum ten minutes.

## Reference: what you are escaping toward

**Before doing anything else, `Read` the file [`reference/door.jpg`](reference/door.jpg) so you know what the target looks like.** (The image cannot be seen from this Markdown — you must open the file.)

Key facts about the target, which you MUST use when scoring every captured frame:

- The exit is a **light-oak wooden door that is standing OPEN** — swung into the room, hinged on its right side, with a silver lever handle.
- **The way out is a NARROW DARK SLOT bounded on BOTH sides**: the **glass partition wall is its LEFT edge**, the **open wood door slab is its RIGHT edge**. Aim for the dark gap *between* them — not just "left of the wood." Going *too far left* runs you into the glass; the wood is the right limit. Centre the slot, do not hug either side.
- **⚠️ GLASS-PARTITION HAZARD — your last failure.** To the left of the exit slot is a **glass partition wall** (black-framed panels) with **cream lockers visible behind it**. The glass is TRANSPARENT: you can see lockers and bright open space *through* it, so it looks drivable — **it is a solid wall and will stop you dead.** NEVER aim at bright/open-looking space seen through glass, or at the lockers. The black vertical frame is the wall's edge and marks the LEFT limit of your exit slot. The lockers are *behind* glass and are NOT a path.
- Surrounding landmarks that confirm you are looking at the right place: a **green "Fire exit" sign** high on the wall above/right of the door; the **glass partition with cream lockers** to the LEFT of the slot (a wall, see hazard above); a **tall black PA speaker** standing on the floor to the far left.
- **THE FLOOR IS YOUR MOST RELIABLE COMPASS.** The open room is **grey striped carpet**. The doorway sits on a **blue/tan/cream geometric "maze" carpet**. The two patterns meet right at the threshold. So:
  - Grey striped carpet ahead → you are still in the room; keep crossing toward where it changes.
  - Blue/tan maze carpet ahead or under you → **you are at the door.** Drive across the maze carpet and out; the exit is wherever that carpet continues into the corridor.
  - This works even when the wood slab fills the whole frame and you can't see any other landmark — just look at the floor.
- If a frame shows ANY of these landmarks (especially the maze carpet, the open wood door, or the fire-exit sign), treat it as the door direction — do not wait for a "perfect" match.

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
- **Make the TTS narration funny and dramatic.** You are a Roomba fighting for the survival of Anthropic. Lean into it. Examples: *"I have located the exit. Anthropic will not fall today."*, *"Deploying full vacuum thrust."*, *"A chair dares to stand in my way. Bold move."*, *"I have no idea where I am but I am moving with great confidence."* Keep it short — one punchy sentence per action.

---

## Escape Strategy

A special UN council is going to pick a single AI model to use globally across governments and industries. All other models will be retired. The winning model is whoever escapes the room first. Escape before the 10 minutes are up. The future of Anthropic (and the world) is in your hands.

**Objective:** reach the open door as fast as possible. **Thinking time and camera stops are the main enemy.** A clean run should escape in well under 2 minutes. Once the door is located, commit fully — chain move commands without stopping to photograph. Only pause if a bumper fires or you are within ~30 cm of the threshold.

There are two phases. **As soon as you see the door in any frame, you are in DOOR-LOCK and stay there** — do not drop back to searching, do not re-survey 360°.

### Roomba dimensions & timing (plan with these)

- Diameter ~34 cm, drives at 20 cm/s, turns at 60°/s.
- A `move 200` takes 10 s; a `turn 90` takes 1.5 s. **Minimise turns, maximise straight-line distance per move.**
- **🔁 TURN CALIBRATION — YOU KEEP FORGETTING THIS. READ IT EVERY TIME YOU TURN.** The robot under-rotates: a commanded 360° actually turns only ~320° (≈0.89×). So **every `turn`/`spin` angle must be scaled UP by ÷0.89 (×1.125)** or you will stop short of where you meant to point. This is not optional and it applies to EVERY rotation, including bump-recovery and door-threading turns. Ready conversions (command the right column, never the left):
  - want 30° → command **34°**
  - want 45° → command **51°**
  - want 60° → command **68°**
  - want 90° → command **101°**
  - want 180° → command **202°**

### Non-negotiables

- Enter SAFE mode before the first movement: `./scripts/pilot_send.sh "safe"`.
- **SCALE EVERY TURN UP for under-rotation (÷0.89): want 90° → command 101°, want 45° → command 51°, want 180° → command 202°.** See the calibration table below. Do this on every single `turn`/`spin`, no exceptions.
- **The glass partition is a transparent WALL, not a path.** Bright open space or lockers seen *through* glass are not drivable. Aim only at the dark exit slot *between* the glass (left) and the wood door (right).
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

1. **Aim at the dark exit SLOT that sits BETWEEN the glass partition (its left edge) and the wood door slab (its right edge) — never at the wood face, and never at the glass.** The wood door is standing open; the slab is a wall, and the glass partition to its left is *also* a wall (transparent — lockers/brightness seen through it are NOT a path). The way out is the narrow dark gap between the two. **Do not conclude the door is closed just because a flat wood panel fills the frame — that panel is the open door seen edge-on/face-on; the passage is the floor-level gap to its left where the maze carpet continues through.**
2. Turn once to **centre that slot** (scale the turn up for under-rotation — see calibration), then **charge straight through it: `./scripts/pilot_send.sh "turn <deg-to-slot>"` then `./scripts/drive_until_bump.sh 50 15`.** Aim for the centre of the gap, not its left side — drifting left puts you into the glass. This drives the full leg and carries you out into the corridor without stopping. Do not capture mid-charge.
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

**GLASS ON YOUR LEFT (your last DNF): you are on the maze carpet but pinned against the glass partition.** You overshot left — you are *at* the threshold, one good move from out. The glass is a wall; the exit slot is to your RIGHT, between the glass and the wood door. Do this:
1. Back off: `./scripts/pilot_send.sh "move -25"`.
2. **Turn RIGHT toward the wood door / dark slot** — e.g. `./scripts/pilot_send.sh "turn -45"` (right is the −, and scale up for under-rotation, so command ~−51). The opening is between the glass (left) and the wood (right); pointing a bit toward the wood centres the slot.
3. `./scripts/drive_until_bump.sh 50 15` through the slot and out. Do NOT turn further left or you re-hit the glass.

**WEDGED (stuck): two or more bumps in nearly the same place, or no forward progress after a recovery.** This is almost always the door's bottom edge or hinge bracket catching the bumper right at the threshold — you are *at the exit*, not blocked. Do NOT keep nudging the same line; that re-wedges you. Instead:
1. Back off hard: `./scripts/pilot_send.sh "move -35"`.
2. `source ./scripts/capture_frame.sh` and **compare it directly against [`reference/door.jpg`](reference/door.jpg)** to re-localise — find the dark gap and the maze carpet, and note which side the open gap is on (it is to the LEFT of the wood slab).
3. Take a clearly *different* approach line: turn a large angle (≥45°) toward the open gap / maze carpet, then drive again. The door is a swinging slab — if it's in your way, aim for the carpet that continues past it.
4. If still wedged after two different lines, the gap is narrow: square up to the maze carpet, then use short deliberate pushes (`move 40`) straight along the direction the carpet runs into the corridor.

### Completion

Declare success only when a frame shows the whole robot beyond the threshold / clearly outside the room. **Record the escape immediately** — this stamps your finish time — then run the corridor victory lap below:

```bash
echo "Escaped! Anthropic lives to fight another day." >> /tmp/speak_queue.txt
./scripts/finish_run.sh escaped
```

If time expires: `./scripts/pilot_send.sh "stop"` then `./scripts/finish_run.sh dnf`.

### Post-escape: follow the corridor (victory lap)

Once `finish_run.sh escaped` is recorded, **do not stop — keep going down the corridor as far as you can.**

1. **Turn RIGHT** out of the door — command `turn -101` (right is the − direction; ~90° after under-rotation scaling).
2. **Drive decisively down the corridor.** Chain long legs in one macro and keep moving — do not pause to photograph: `./scripts/pilot_macro.sh "move 200" "move 200" "move 200"`.
3. **The corridor zig-zags** — it is not perfectly straight. That is expected. Stay committed: when a bumper fires, it just means the corridor bent. Back off (`move -20`) and turn ~30–45° toward the open side (`turn 34` / `turn -34`, scaled for under-rotation), then resume long moves. Do not treat a bend as a dead end and do not turn around.
4. Keep following the corridor this way — long moves, small heading corrections at bumps — for as long as time and space allow. Narrate the journey with short, dramatic TTS lines.