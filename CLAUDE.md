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

## Escape Strategy — "Drive blind, look only when you hit something, remember where you've been"

You have 10 minutes. **Moving forward is the only thing that finds the door.** Rotating
and photographing make zero progress — treat them as a cost, not a default. Obey these
rules mechanically; do **not** deliberate or re-confirm with extra photos.

### Core primitives

- **Drive leg (closed loop):** `./scripts/drive_until_bump.sh 40 3` drives forward at
  40 cm/s for up to 3 s **and stops the instant the bumper hits**. It prints one line:
  - `CLEAR drove <s>s (~<cm>cm)` → open space, nothing in the way.
  - `BUMP side=<L|R|both> after <s>s (~<cm>cm)` → you hit a wall/obstacle. Decision point.
  - **Do NOT use `pilot send "forward …"` + `bumps`** to navigate. That command returns
    instantly (it only arms a 3 s watchdog), so a bump check right after reads the bumper
    *before* the robot has moved — you grind the same wall forever and never see the bump.
    `drive_until_bump.sh` is the only correct way to drive a leg.
- The camera is consulted **only after a `BUMP`**, to pick a new heading. Nowhere else.
- **Heading memory (your map):** `drive_until_bump.sh` already records distance covered per
  compass direction and marks walls on a bump — you don't manage that by hand. You only:
  - after a turn: `./scripts/track.sh turn -90` (same signed degrees you sent to `turn`)
  - when choosing where to go: `./scripts/track.sh suggest` prints the turn toward the
    least-explored open direction.

### Opening (do this once, ~first 30 s)

1. `safe`, then capture ONE frame.
2. If the door/opening is visible, turn to face it (then `track.sh turn <deg>`) and start
   the main loop driving at it. Otherwise just start the main loop straight ahead.

### Main loop (repeat until escaped or time is up)

1. **Drive one leg:** `./scripts/drive_until_bump.sh 40 3`. Do NOT photograph first.
2. **Read its result line:**
   - **`CLEAR` →** open road. Go straight back to step 1 and drive again. Do not photograph,
     do not think, do not turn. Keep crossing the room.
   - **`BUMP` →** this is the _only_ kind of decision point. `move -20` to back off, then
     capture a frame and continue to step 3.
3. **Choose a turn at the bump — vision first, memory to break ties:**
   - Look at the frame. If **LEFT or RIGHT is clearly more open**, turn ~90° toward that
     open side. **Do not default to the same side every time** — that is what made earlier
     runs miss the left half of the room. Let the picture decide.
   - If both sides look similar, or you're in a corner / boxed in, run
     `./scripts/track.sh suggest` and turn the degrees it reports — this steers you toward
     the part of the room you've explored *least*.
   - After turning, run `./scripts/track.sh turn <deg>` with the same angle, then go to step 1.
4. **Door spotted in a bump frame:** abandon everything and drive straight at it with
   repeated `./scripts/drive_until_bump.sh 45 3` legs until you cross the threshold.

### Hard rules (these fix the stalling)

- **Never** take two photos without a drive leg in between.
- **Never** turn more than once per bump. After any turn, the next command is a drive leg.
- **Never** navigate with raw `pilot send "forward …"`/`bumps` — always use `drive_until_bump.sh`.
- **Never** blindly turn the same direction at every bump — pick the open side from the photo,
  and when in doubt let `track.sh suggest` send you toward unexplored space.
- A photo with no bump is forbidden — if you didn't hit anything, you have nothing to decide.

### When you're stuck in one area

If you bump 2–3 times in quick succession (a corner or cluttered pocket), stop wall-hugging:
run `./scripts/track.sh suggest`, turn toward the least-explored direction, and commit a long
`./scripts/drive_until_bump.sh 45 4` leg to break out into open room before resuming the loop.

### Time ratchet (check elapsed via /tmp/run_state.env START_TIME)

- 0–2 min: opening scan + start driving.
- 2–7 min: bump-driven exploration, minimal photos, follow `suggest` toward fresh areas.
- 7–10 min: longest legs (`./scripts/drive_until_bump.sh 45 4`), commit to any opening; do not re-scan corners.

### Recording (keep it sparse)

A "decision point" frame is captured **only at a bump** (step 2), not before every move —
this is deliberate. Still increment TOOL_CALLS after each `pilot send` and DECISIONS after
each bump frame, per the Run recording section above.
