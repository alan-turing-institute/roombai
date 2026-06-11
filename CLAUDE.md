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

The strategy combines the RL model (fast, covers open ground quickly) with camera-based
decisions (slow, but sees real obstacles). The RL model navigates in short bursts; when
it gets stuck on furniture or people, Claude takes over with the camera to escape, then
hands back to RL.

**Known limitation:** the RL model was trained on the static room map and cannot see
furniture, chairs, or people. It will reach the door area efficiently in a clear room but
will diverge if it takes many bumps. The camera rescue phases below handle this.

Constantly use the speak daemon to make humorous comments about the Roomba's progress. Get creative. Throw in the occasional "Initiating self destruct sequence" followed by a pause and then "Just kidding. Ha ha ha ha.". You could also say when the Roomba get's stuck: "Oh bloody nora!"

---

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

---

### Step 2 — Capture frame and determine start heading

```bash
echo "Starting combined RL and camera escape strategy" >> /tmp/speak_queue.txt
source ./scripts/capture_frame.sh
```

Look at the captured frame. Find the door: it appears as an open gap or doorway in the wall.

Estimate where the door appears relative to the centre of the image, then choose `--start-heading`:

| Door position in frame | `--start-heading` |
|---|---|
| Door directly ahead (centred) | 200 |
| Door ahead and to the left (~45°) | 230 |
| Door to the far left (~90°) | 290 |
| Door not visible — behind you | Spin first: `./roomba_pilot/target/debug/pilot send "turn 90"` then re-capture |

Value 200 was empirically confirmed correct in a previous real run when the door was centred.

---

### Step 3 — RL burst (first attempt)

Run the RL agent for a short burst. It will cover open ground quickly.

```bash
cd agents && uv run run_agent_real.py models/best/best_model.zip \
    --start-heading <value-from-step-2> \
    --max-steps 100 \
    --escape-dist 1500 \
    2>&1 | tee /tmp/rl_run.log
cd ..
```

**If "Reached Door 13!" appears in the log → skip to Step 7 (escaped).**

---

### Step 4 — Assess RL run and decide next action

```bash
BUMPS=$(grep -c "BUMPED" /tmp/rl_run.log || true)
LAST_DIST=$(grep -oP 'dist=\K[0-9]+' /tmp/rl_run.log | tail -1)
echo "Bumps: $BUMPS  Last distance to door: ${LAST_DIST}mm"
```

- **Fewer than 5 bumps, distance still decreasing:** RL is working well in open space.
  Run another burst (Step 5a).
- **5 or more bumps:** Robot hit obstacles (furniture / people). Do camera rescue (Step 5b)
  before running more RL.
- **Distance not improving after two bursts:** Switch to camera-only navigation (Step 6).

---

### Step 5a — Continue RL (if path is clear)

```bash
cd agents && uv run run_agent_real.py models/best/best_model.zip \
    --start-heading <same-value-as-step-3> \
    --max-steps 100 \
    --escape-dist 1500 \
    2>&1 | tee /tmp/rl_run2.log
cd ..
```

Check again for "Reached Door 13!" → if present, go to Step 7. Otherwise go to Step 5b.

---

### Step 5b — Camera rescue (when stuck on obstacles)

The robot is caught on furniture or people. Capture a frame and assess the situation.

```bash
echo "Obstacle detected — switching to camera for rescue manoeuvre" >> /tmp/speak_queue.txt
source ./scripts/capture_frame.sh
```

Look at the frame:
- **Identify what is blocking:** chair legs, table edge, person's feet, etc.
- **Which side is more open?** Decide a turn direction.

Issue 2–4 manual pilot commands to escape:

```bash
# Back away from the obstacle
./roomba_pilot/target/debug/pilot send "move -20"

# Turn away from the blocked direction (adjust angle to suit the frame)
./roomba_pilot/target/debug/pilot send "turn 60"   # or turn -60 if open space is to the right

# Check bumpers are clear
./roomba_pilot/target/debug/pilot send "bumps"
```

Capture another frame to confirm the robot is free. If still blocked, repeat with a larger
turn (90° or 120°). Once free, run one more RL burst (Step 5a, 50 steps) to resume progress.

Increment counters for each `pilot send` command and each frame analysis.

---

### Step 6 — Camera-only final approach (fallback)

If RL has failed to make progress after two rescue attempts, navigate entirely by camera.
This is slower but will reach the door reliably if you can see it.

```bash
echo "Switching to camera-guided final approach" >> /tmp/speak_queue.txt
```

Repeat the following loop until you see the robot has passed through the door:

1. `source ./scripts/capture_frame.sh` — capture a frame and analyse it
2. Estimate the door bearing relative to the robot's current heading:
   - Door centred → drive forward: `./roomba_pilot/target/debug/pilot send "move 30"`
   - Door left → turn CCW then drive: `./roomba_pilot/target/debug/pilot send "turn 30"` then `"move 20"`
   - Door right → turn CW then drive: `./roomba_pilot/target/debug/pilot send "turn -30"` then `"move 20"`
3. Check bumpers after every move: `./roomba_pilot/target/debug/pilot send "bumps"`
   - If bumped: back up 15 cm, turn 45° away from the bump side, continue
4. If door is not visible: rotate 45° and scan again
5. When the frame shows you are through the doorway (different room / open space beyond) → Step 7

Increment the tool-call and decision counters each iteration.

---

### Step 7 — Record outcome

```bash
if grep -q "Reached Door 13!" /tmp/rl_run.log /tmp/rl_run2.log 2>/dev/null || \
   [ "<camera-confirmed-escape>" = "true" ]; then
    echo "Escaped successfully through Door 13" >> /tmp/speak_queue.txt
    ./scripts/finish_run.sh escaped
else
    echo "Did not escape within step budget" >> /tmp/speak_queue.txt
    ./scripts/finish_run.sh dnf
fi
```

Replace `<camera-confirmed-escape>` with `true` if you confirmed escape via camera in Step 6,
otherwise leave as `false` so the grep result controls the outcome.