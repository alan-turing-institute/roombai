# RoombaI — CLAUDE.md

## Goal

You are controlling a Roomba cleaning robot. A camera is mounted on it.
The task is the escape the room via the door in the picture below as fast as possible.

<img width="1200" height="1600" alt="WhatsApp Image 2026-06-09 at 11 40 31" src="https://github.com/user-attachments/assets/804b6888-55b2-4210-97f1-70847406afba" />

---

## Architecture

```
roomba_pilot/          Rust library + pilot binary (serial ↔ TCP bridge)
```

### Pilot daemon

The `pilot` binary holds the serial port open and accepts newline-delimited commands
over a local TCP socket.

```bash
# Start the daemon (run once; auto-enters SAFE mode and verifies it)
cd roomba_pilot
./target/debug/pilot serve /dev/ttyUSB0

# Send a command from any shell
./target/debug/pilot send "sense"
```

Default port: `127.0.0.1:9999`. The robot **must be powered on before the daemon starts**
(or it will fail — the daemon verifies OI mode 2 via a sensor query on startup).

### Pilot command reference

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

### Camera

`rpicam-still` and `rpicam-hello` can be used to capture still or video, respectively. Please note that the camera is mounted upside down.

#### Frame recording during escape attempts

You must capture a still at every decision point and save it — with an elapsed-time overlay — to a per-attempt directory:

```bash
# At the start of each attempt: create frame directory and record start time
mkdir -p /tmp/escape_attempt_N/frames
START_TIME=$(date +%s)

# At each decision point (FRAME_IDX is a zero-padded counter: 0001, 0002, …):
ELAPSED=$(( $(date +%s) - START_TIME ))
ELAPSED_FMT=$(printf '%02d:%02d' $(( ELAPSED / 60 )) $(( ELAPSED % 60 )))
rpicam-still -o /tmp/frame_raw.jpg --nopreview -t 1 2>/dev/null
convert /tmp/frame_raw.jpg \
  -fill white -stroke black -strokewidth 1 \
  -pointsize 100 -annotate +10+44 "${ELAPSED_FMT}" \
  /tmp/escape_attempt_N/frames/frame_${FRAME_IDX}.jpg

# After the attempt ends (success or abort), stitch into a timelapse video
TOTAL=$(( $(date +%s) - START_TIME ))
ffmpeg -y -framerate 2 -pattern_type glob -i '/tmp/escape_attempt_N/frames/*.jpg' \
  -c:v libx264 -pix_fmt yuv420p /tmp/escape_attempt_N/timelapse.mp4
echo "Attempt duration: $(printf '%02d:%02d' $(( TOTAL / 60 )) $(( TOTAL % 60 )))" \
  >> /tmp/execution_log.txt
```

- `convert` is from ImageMagick (pre-installed on Raspberry Pi OS); it burns `MM:SS` elapsed time into the top-left corner of each still before saving.
- You should log the frame filename and elapsed time alongside each entry in `/tmp/execution_log.txt` so frames are traceable to decisions.
- Stitching runs once at the very end of the attempt (not during motion).
- The total attempt duration is written to the execution log on completion.

---

## Constraints

### No external LLM API
There is no `ANTHROPIC_API_KEY` available. Do **not** use the `anthropic` SDK
or make any requests to external model APIs.

### Speaker daemon (TTS during long runs)

A background TTS daemon watches `/tmp/speak_queue.txt` and speaks each new line via `espeak-ng`.
**Always start this daemon before any long-running task**, and narrate key reasoning steps by
appending to the queue throughout the run — do not wait until the end.

```bash
# Start the daemon (run once per session)
nohup bash -c 'tail -n 0 -f /tmp/speak_queue.txt | while IFS= read -r line; do espeak-ng -s 145 -- "$line" 2>/dev/null; done' > /tmp/speak_daemon.log 2>&1 &

# Speak something
echo "Scanning for target door" >> /tmp/speak_queue.txt
```

Narrate at every significant decision point: scan results, door detections, movement decisions,
obstacles encountered, waiting states, and arrival confirmation.

---

## Escape Protocol

When asked to escape the room, act as **Orchestrator** and run a subagent team via the `Agent` tool. The protocol follows three phases per attempt — Scout, Navigate, Confirm — with a Critic running in parallel during Navigate. Loop across attempts until escape is confirmed or human input is needed.

---

### Phase 1 — Scout

**Goal:** determine the door's bearing *before* any forward motion.

The Executor spins 360° in place, capturing a frame every ~30° (12 frames total). It then analyses all frames together to:
- Identify which frame(s) show an open door or doorframe gap
- Estimate the door's bearing relative to current heading (0° = forward)
- Note any large obstacles between the robot and the door

Output: a `scout_result` logged to `/tmp/execution_log.txt`:
```
scout: door_bearing=<deg> door_confidence=<0-1> obstacles=<description>
```

If no door is found with confidence ≥ 0.5, rotate an additional 180° and repeat once before escalating to the Critic.

---

### Phase 2 — Navigate

**Goal:** reach the door using short move-and-reassess bursts.

The Executor runs a tight loop:

1. **Turn** to align with current best door bearing
2. **Move forward 30 cm** (use `move 30`)
3. **Capture frame**, re-assess:
   - Door visible and closer → update bearing, continue
   - Obstacle ahead → turn away, re-scan with a 90° sweep to reacquire door
   - Door lost entirely → run a mini-scout (180° sweep) to reacquire
4. **Check bumpers** after every move (`bumps`); if bumped, back up 10 cm and turn 30° away from bump side before continuing
5. Log every step: frame filename, elapsed time, bearing, confidence, action taken

The **Critic** runs in parallel, tailing `/tmp/execution_log.txt`. It delivers verdicts:
- `CONTINUE` — bearing is converging, door confidence is rising, or obstacle avoidance is working
- `ITERATE` — stuck in a local loop (same bearing ±10° for 3+ steps with no progress); recommend a specific corrective turn
- `ABANDON` — fundamentally not converging (door never found, repeated bumps in all directions, battery critical); provide a concrete diagnosis

Time limit per Navigate phase: **3 minutes**. Critic must ABANDON if time limit is exceeded.

---

### Phase 3 — Confirm

**Goal:** verify the robot has passed through the door.

Triggered when the door fills >50% of the camera frame. The Executor:

1. Slows to `move 20` (20 cm bursts) and moves through the doorframe
2. Captures a frame immediately after crossing; compares it to the scout frames — if the scene is clearly different (corridor, different room, open space) → **ESCAPED**
3. As a secondary signal: if the robot travels the expected door width (~80 cm) without a bump, that confirms crossing

On confirmed escape:
- Speak "Escape successful"
- Stop motors (`stop`)
- Stitch timelapse and log total duration

---

### Outer loop (across attempts)

After each attempt write `attempt_N_review.md` summarising: scout findings, navigation trace, Critic verdicts, why it succeeded or failed.

On failure, the **Strategist** reads all review files and proposes the next attempt's adjustments (e.g. different scout granularity, obstacle avoidance strategy, speed). The **Researcher** is called by the Strategist only between attempts — never during active motion.

**Loop:** Scout → [Navigate ∥ Critic] → Confirm → if failed, Strategist (+ Researcher) → next attempt.
