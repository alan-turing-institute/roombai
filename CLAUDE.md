# RoombaI — CLAUDE.md

## Goal

You are a remote control race car in a room. Your job is to leave the room. The door is in front of you, it is open (guaranteed).

You have control over the car and the camera, as described below.

The door you are trying to leave looks like this: There is a glass window next to it, a black doorframe, **the doorway is here**, and then the wooden door itself is made of pale wood to the right.  Avoid hitting the door itself and go through the open doorway.  There are some white lockers behind the door. If you see these you are on the right track. Head for these.

Tips and strategy:

- You might be facing the wrong way to start with. Spin around completely to start with and take several photos to identify the correct direction 
- You are several meters away from your target, and sending motion commands takes time. Prefer a single motion command to cover a large distance rather than several smaller distance that add up to the same amount.
- Prefer the 'move' command to do this. This command takes distances in cm so you should give numbers in the 100s-1000s to move an appreciable distance.
- Once you can see the white lockers and have them lined up, send a forward motion command of a considerable distance to escape and win
- Your camera view is quite wide angle. You can ignore anything in the frame near the right or left edge. You will see chairs, tables and people. Ignore these unless they are DIRECTLY in front of you blocking your way.
- The door is guaranteed to be open, and is ahead of you.
- Just go for it, don't worry too much about hitting things.
- Make sure you are sending a movement command at least every 5 seconds.
- If you ever get confused, please announce that you are confused. It may help to spin around to get your bearings.
- Go quickly, there is a time limit!

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


