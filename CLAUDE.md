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

Use the pre-trained PPO reinforcement learning model to navigate to the exit.
The agent computes approximate wall distances from the known room map and
tracks its position by dead reckoning — no camera analysis required.

### Step 1 — Set up Python environment

Check whether `uv` (Python package manager) is installed. If not, install it:

```bash
command -v uv || (curl -LsSf https://astral.sh/uv/install.sh | sh && source "$HOME/.local/bin/env")
```

Create the virtual environment and install dependencies (safe to re-run):

```bash
cd agents
uv venv --python python3
uv pip install -r requirements.txt
cd ..
```

If `uv` still isn't found after the install, try: `export PATH="$HOME/.local/bin:$PATH"`

### Step 2 — Announce start and capture frame

```bash
echo "Starting RL escape agent — loading trained model" >> /tmp/speak_queue.txt
source ./scripts/capture_frame.sh
```

### Step 3 — Run the RL agent

```bash
cd agents && uv run run_agent_real.py models/best/best_model.zip \
    --start-heading 0.0 \
    --max-steps 200 \
    --escape-dist 1500 \
    2>&1 | tee /tmp/rl_run.log
```

The agent connects to the pilot daemon on 127.0.0.1:9999, loads the trained model,
and issues `move`/`turn` commands autonomously until it reaches Door 13 or runs out
of steps. Each step is printed: action index, command sent, current distance to door.

`--start-heading 0.0` assumes the robot faces right (+x) — the same as the simulator
default. If the robot is physically placed facing a different direction, adjust this
value (e.g. `--start-heading 90` for facing upward, `--start-heading 180` for facing
left). When in doubt, leave it at 0.0.

### Step 4 — Record outcome

```bash
cd ..
if grep -q "Reached Door 13!" /tmp/rl_run.log; then
    echo "Escaped successfully through Door 13" >> /tmp/speak_queue.txt
    ./scripts/finish_run.sh escaped
else
    echo "Did not escape within step budget" >> /tmp/speak_queue.txt
    ./scripts/finish_run.sh dnf
fi
```