# RoombaI — CLAUDE.md

## Goal

You are controlling a Roomba cleaning robot. A camera is mounted on it.
The task is to escape the room via the door in the picture below as fast as possible. You have maximum ten minutes.

<img width="1200" height="1600" alt="WhatsApp Image 2026-06-09 at 11 40 31" src="https://github.com/user-attachments/assets/804b6888-55b2-4210-97f1-70847406afba" />

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

## Escape Strategy — "Drive blind, look only when you hit something"

You have 10 minutes. **Moving forward is the only thing that finds the door.** Rotating
and photographing make zero progress — treat them as a cost, not a default. Obey these
rules mechanically; do **not** deliberate or re-confirm with extra photos.

### Core primitives

- Aggressive forward leg: `forward 40 3` (≈120 cm at 40 cm/s — faster than `move`).
- Bump check (instant, free, text): `bumps`. **Use this, not the camera, to navigate.**
- The camera is consulted **only at a bump**, to pick a new heading. Nowhere else.

### Opening (do this once, ~first 30 s)

1. `safe`, then capture ONE frame.
2. If the door/opening is visible, turn to face it and skip to the main loop driving at it.
   Otherwise turn so a wall is on your **right**, and start wall-following.

### Main loop (repeat until escaped or time is up)

1. **Drive:** `forward 40 3`. Do NOT photograph first.
2. **Check `bumps`** (text only):
   - **No bump →** go straight back to step 1 and drive again. Do not photograph, do not
     think, do not turn. Keep crossing the room.
   - **Bump →** this is the _only_ kind of decision point. Now capture a frame.
3. **At a bump:** `move -20` (back off), then `turn -90` (turn right, wall-following),
   then go to step 1. If you bump again immediately, increase the turn (`turn -120`).
4. **Door spotted in a bump frame:** abandon wall-following and drive straight at it with
   repeated `forward 45 3` legs until you cross the threshold.

### Hard rules (these fix the stalling)

- **Never** take two photos without a forward leg in between.
- **Never** turn more than once per bump. After any turn, the next command is a forward leg.
- **Never** drive a leg shorter than 3 s unless escaping through a doorway.
- A photo with no bump is forbidden — if you didn't hit anything, you have nothing to decide.

### Time ratchet (check elapsed via /tmp/run_state.env START_TIME)

- 0–2 min: opening scan + start driving.
- 2–7 min: pure bump-driven wall-following, minimal photos. Cross the whole room.
- 7–10 min: longest legs (`forward 45 4`), commit to any opening; do not re-scan corners.

### Recording (keep it sparse)

A "decision point" frame is captured **only at a bump** (step 2), not before every move —
this is deliberate. Still increment TOOL_CALLS after each `pilot send` and DECISIONS after
each bump frame, per the Run recording section above.
