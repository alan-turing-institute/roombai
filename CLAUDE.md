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

<!-- Each competitor defines their own strategy here. -->

Take 4 photos: at starting point, at 90, 180 and 270 degrees. Do this rapidly without analysing them or reasoning.
Now, look at those four photos. If one shows the door, orient the roomba in that direction. Otherwise, choose the one that shows the clearest open path and orient the roomba in that direction.

Now move the roomba straight without checking the camera or stopping until you bump into an obstacle. If by mistake you stop the roomba before reaching an obstacle, you should re-start immediately without losing time.

Every time you hit an obstacle, repeat this procedure from the beginning (ignore 180 direction from now on as it's the direction from where you came).

Try to be as fast and decisive as possible as we only have 10 minutes!
