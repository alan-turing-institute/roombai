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

---

## Run recording (mandatory for every attempt)

Every attempt **must** produce a self-contained run directory that is uploaded to OneDrive after the run. This is non-negotiable — it feeds the competition leaderboard.

### Run directory structure

```
/tmp/escape_attempt_N/
  ├── frames/
  │     ├── frame_0001.jpg   (still with elapsed-time overlay)
  │     ├── frame_0002.jpg
  │     └── …
  ├── timelapse.mp4
  └── metadata.json
```

### 1. Start of attempt — initialise

```bash
# Determine attempt number from existing directories
ATTEMPT_N=$(( $(ls -d /tmp/escape_attempt_* 2>/dev/null | wc -l) + 1 ))
ATTEMPT_DIR=/tmp/escape_attempt_${ATTEMPT_N}
mkdir -p "${ATTEMPT_DIR}/frames"

# Read pilot identity from git (branch name = pilot name)
PILOT_NAME=$(git -C "$(git rev-parse --show-toplevel)" rev-parse --abbrev-ref HEAD)
COMMIT_HASH=$(git -C "$(git rev-parse --show-toplevel)" rev-parse --short HEAD)

# Start timer and zero counters
START_TIME=$(date +%s)
TOOL_CALLS=0
DECISIONS=0
```

Increment `TOOL_CALLS` by 1 each time a `pilot send` command is issued.  
Increment `DECISIONS` by 1 each time a camera frame is analysed to make a navigation decision.

### 2. At every decision point — capture a frame

```bash
# FRAME_IDX is a zero-padded counter: 0001, 0002, …
ELAPSED=$(( $(date +%s) - START_TIME ))
ELAPSED_FMT=$(printf '%02d:%02d' $(( ELAPSED / 60 )) $(( ELAPSED % 60 )))
rpicam-still -o /tmp/frame_raw.jpg --nopreview -t 1 2>/dev/null
convert /tmp/frame_raw.jpg \
  -fill white -stroke black -strokewidth 1 \
  -pointsize 100 -annotate +10+44 "${ELAPSED_FMT}" \
  "${ATTEMPT_DIR}/frames/frame_${FRAME_IDX}.jpg"
```

`convert` is from ImageMagick (pre-installed on Raspberry Pi OS). It burns `MM:SS` elapsed time into the top-left corner of each still.

### 3. End of attempt — stitch timelapse and write metadata

```bash
# Stitch frames into timelapse
ffmpeg -y -framerate 2 -pattern_type glob -i "${ATTEMPT_DIR}/frames/*.jpg" \
  -c:v libx264 -pix_fmt yuv420p "${ATTEMPT_DIR}/timelapse.mp4"

# Write metadata  (OUTCOME is "escaped" or "dnf")
DURATION_S=$(( $(date +%s) - START_TIME ))
cat > "${ATTEMPT_DIR}/metadata.json" <<EOF
{
  "name":       "${PILOT_NAME}",
  "commit":     "${COMMIT_HASH}",
  "date":       "$(date +%Y-%m-%d)",
  "duration_s": ${DURATION_S},
  "tool_calls": ${TOOL_CALLS},
  "decisions":  ${DECISIONS},
  "outcome":    "${OUTCOME}"
}
EOF
```

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

## Escape Strategy

<!-- Each competitor defines their own strategy here. -->
