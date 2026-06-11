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

There are two modes. **As soon as you see the door in any frame, you are in DOOR-LOCK mode and stay there** — do not drop back to searching.

### SEARCH mode (door not yet seen)

Take 4 photos: at starting point, at 90, 180 and 270 degrees. Do this rapidly without analysing them or reasoning.
Now look at the four photos. **Check each one against [`reference/door.jpg`](reference/door.jpg) and the landmark list above.** If any frame shows the door or its landmarks, orient the Roomba in that direction and switch to DOOR-LOCK mode. Otherwise choose the one with the clearest open path and orient toward it.

Now drive straight without checking the camera or stopping until you bump into an obstacle (use `./scripts/drive_until_bump.sh 40 8`). If you stop before hitting anything, restart immediately without losing time.

Every time you hit an obstacle, repeat this procedure from the beginning (ignore the 180° direction from now on — that's where you came from).

### DOOR-LOCK mode (door is visible — commit and charge)

Once you have seen the door, **do NOT re-survey 360° and do NOT drive blind toward a bump.** The door is the goal; go to it directly and aggressively:

1. Turn to put the **dark open gap beside the wood slab** in the centre of the frame. Aim at the gap, never at the wood face.
2. Drive a hard burst straight at it: `./scripts/drive_until_bump.sh 50 4`. Go fast — this is the moment to be aggressive, not cautious.
3. Capture one frame and correct heading only if the gap has drifted off-centre (small turn, ≤20°). Then drive another burst. Repeat.
4. As the doorway fills more of the frame, keep driving through it — **do not stop just because you are close.** You are aiming to pass the wheels through the dark gap and out into the corridor.
5. If you bump the wood slab or a frame, you are off to one side: turn toward the dark gap (it is to the *left* of the wood door) and drive again. Do not restart a full search.

Only fall back to SEARCH mode if the door has completely left the frame and a quick look (one or two photos) cannot re-acquire it.

### Calibration

The Roomba is not well calibrated: a commanded 360° rotation actually turns ~320° (≈0.89×). Scale every turn accordingly — to actually turn 90°, command ~101°. Apply this to all rotations.

Be fast and decisive — you only have 10 minutes, and most of it should be spent driving, not deliberating.