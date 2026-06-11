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
decisions (slow, but sees real obstacles). The RL model is run in short bursts (30–40 steps)
to navigate open space. If it gets stuck, collides frequently, or fails to make distance progress,
we pause RL immediately and perform visual/physical recovery before resuming. When close to the
door (< 4000 mm), we switch to camera-guided crossing to ensure we pass through the doorway safely.

**Known limitation:** the RL model was trained on the static room map and cannot see
furniture, chairs, or people. It will reach the door area efficiently in a clear room but
will diverge if it takes many bumps. The visual guardrails below handle this.

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

### Step 2 — Initial Visual Alignment

Before running the RL model, align the Roomba to point towards the exit doorway or clear open space to give the agent the best starting trajectory.

1. **Capture a frame**:
   ```bash
   source ./scripts/capture_frame.sh
   ```
2. **Analyze the frame**:
   - Locate the exit doorway (Door 13) or the large open corridor leading out of the Enigma room.
   - If facing a wall or obstacle close-up, rotate the Roomba:
     ```bash
     ./roomba_pilot/target/debug/pilot send "turn 60"  # or turn -60, adjust to point to open area
     ```
3. **Record starting heading**: Note the estimated heading relative to the simulator starting heading (e.g., if you turned 60 degrees CCW, the heading is `-1.5708 + 1.047` = `-0.5238` rad).

---

### Step 3 — Run RL in a Short Burst

Run the RL agent for a short burst (30–40 steps). Running long RL loops is inefficient if the robot is stuck or drifting.

```bash
cd agents && uv run run_agent_real.py models/best/best_model.zip \
    --start-heading <value-from-step-2> \
    --max-steps 40 \
    --escape-dist 1500 \
    2>&1 | tee /tmp/rl_run.log
cd ..
```

- **If "Reached Door 13!" appears in the log → skip to Step 7 (escaped).**

---

### Step 4 — Assess RL Progress and Decide Next Action

Examine the PPO log file `/tmp/rl_run.log`:

```bash
BUMPS=$(grep -c "BUMPED" /tmp/rl_run.log || true)
START_DIST=$(grep -oP 'dist=\K[0-9]+' /tmp/rl_run.log | head -1 || echo 60000)
END_DIST=$(grep -oP 'dist=\K[0-9]+' /tmp/rl_run.log | tail -1 || echo 60000)
DIST_IMPROVEMENT=$(( START_DIST - END_DIST ))
echo "Bumps: $BUMPS  Start dist: ${START_DIST}mm  End dist: ${END_DIST}mm  Improvement: ${DIST_IMPROVEMENT}mm"
```

- **If End Distance is < 4000 mm**: The robot is close to the doorway. Switch to camera-only final approach (Step 6) to avoid PPO oscillation/confusion at the door frame.
- **If Bumps < 3 and Improvement > 300 mm**: RL is progressing cleanly. Run another short RL burst (Step 3, adjusting `--start-heading` if needed to reflect the current heading in the logs).
- **If Bumps >= 3 or Improvement <= 300 mm**: The robot is stuck, blocked, or repeating a turn loop. Proceed to Step 5 (Camera & Physical Rescue).

---

### Step 5 — Visual & Physical stuck Rescue

The robot is wedged or blocked by furniture/walls. We perform a quick backing maneuver followed by a camera scan to face clear space.

1. **Back away from obstacle**:
   ```bash
   echo "Oh bloody nora! I appear to be stuck. Backing away..." >> /tmp/speak_queue.txt
   ./roomba_pilot/target/debug/pilot send "move -25"
   sed -i "s/^TOOL_CALLS=.*/TOOL_CALLS=$(( $(grep TOOL_CALLS /tmp/run_state.env | cut -d= -f2) + 1 ))/" /tmp/run_state.env
   ```
2. **Scan the surroundings**:
   Capture a frame to check the new visual situation:
   ```bash
   source ./scripts/capture_frame.sh
   ```
3. **Turn to face clear space**:
   - Locate the most open area (away from table legs, chairs, or close walls).
   - Turn the Roomba towards the clear space (e.g. `turn 45` or `turn -45`):
     ```bash
     ./roomba_pilot/target/debug/pilot send "turn 45"  # adjust angle based on camera frame
     sed -i "s/^TOOL_CALLS=.*/TOOL_CALLS=$(( $(grep TOOL_CALLS /tmp/run_state.env | cut -d= -f2) + 1 ))/" /tmp/run_state.env
     ```
4. **Resume RL**: Clear any search counts and return to Step 3 with a new RL burst.

---

### Step 6 — Camera-Guided Door Crossing (Final Approach)

When within 4 meters of the doorway, we navigate visually to handle the open/close timer of Door 13 and safely cross.

```bash
echo "Initiating final approach. Engaging camera guidance." >> /tmp/speak_queue.txt
```

Repeat the following loop until you see the robot has passed through the doorway:

1. **Capture frame**: `source ./scripts/capture_frame.sh`
2. **Analyse door state**:
   - If the door is **closed**, wait 2 seconds and capture again: `sleep 2`
   - If the door is **open**, estimate alignment:
     - Door centered: drive forward (`move 30`)
     - Door left: turn CCW (`turn 20`) then drive forward (`move 20`)
     - Door right: turn CW (`turn -20`) then drive forward (`move 20`)
3. **Bumper check**: If any move triggers a bumper collision, back up 15 cm (`move -15`), turn 45° away from the collision side, and scan again.
4. When the camera shows you are in the hallway/kitchen beyond Door 13, proceed to Step 7.

---

### Step 7 — Record outcome

```bash
# Check if escape succeeded
if grep -q "Reached Door 13!" /tmp/rl_run.log 2>/dev/null || \
   [ "<camera-confirmed-escape>" = "true" ]; then
    echo "Escaped successfully through Door 13" >> /tmp/speak_queue.txt
    ./scripts/finish_run.sh escaped
else
    echo "Did not escape within step budget" >> /tmp/speak_queue.txt
    ./scripts/finish_run.sh dnf
fi
```

Replace `<camera-confirmed-escape>` with `true` if you confirmed escape via camera in Step 6, otherwise leave as `false`.