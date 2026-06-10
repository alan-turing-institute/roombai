# RoombaI — Escape Room Competition

A competition where each participant programs a Claude AI agent to control a Roomba robot and escape a room through an open door as fast as possible. The robot has a camera mounted on it and Claude controls it by issuing pilot commands.

Each attempt lasts **10 minutes maximum**. If the robot hasn't escaped by then, the run is recorded as DNF.

---

## How it works

1. You write an escape strategy in `CLAUDE.md` on your own branch
2. You run `./run.sh` — it resets state, starts all daemons, launches Claude, and uploads results automatically
3. Your run data (frames, metadata) is uploaded to Azure Blob Storage
4. A leaderboard is built from everyone's uploaded results

---

## Joining the competition

### 1. First-time setup

Clone the repo and run the setup script **once** on the Raspberry Pi:

```bash
git clone <repo-url>
cd roombai
./setup.sh
```

This will:
- Install system dependencies (`rclone`, `imagemagick`, `espeak-ng`)
- Build the Rust pilot binary
- Prompt you for your Azure Blob Storage account name and SAS token

### 2. Create your branch

Name your branch after yourself — this is how you appear on the leaderboard:

```bash
git checkout main
git checkout -b yourname/attempt-1
```

### 3. Write your escape strategy

Open `CLAUDE.md` and fill in the `## Escape Strategy` section at the bottom with your approach. This is the only section you should edit — everything else is shared infrastructure.

See [Strategy example](#strategy-example) below for inspiration.

### 4. Run

Power on the Roomba, then:

```bash
./run.sh
```

The script will walk you through 7 steps:
1. Reset — wipes `/tmp` and Claude memory
2. Roomba power check — confirms it's on before connecting
3. TTS daemon — starts voice narration
4. Pilot daemon — connects to the Roomba over serial, waits for SAFE mode
5. Run init — creates your attempt directory and state file
6. Claude — launches and runs your `/escape` strategy
7. Upload — sends all artifacts to Azure Blob Storage

### 5. Iterate

After each attempt, commit your updated strategy and create a new branch for the next try:

```bash
git add CLAUDE.md
git commit -m "improve door detection"
git checkout -b yourname/attempt-2
```

---

## What gets recorded

Every run automatically produces a directory at `/tmp/escape_attempt_<commit>/` containing:

```
escape_attempt_<commit>/
  ├── frames/
  │     ├── frame_0001.jpg    ← camera still with MM:SS overlay
  │     ├── frame_0002.jpg
  │     └── …
  └── metadata.json           ← pilot name, commit, duration, tool calls, outcome
```

`metadata.json` example:
```json
{
  "name":       "alice/attempt-2",
  "commit":     "a1b2c3d",
  "date":       "2026-06-09",
  "duration_s": 34,
  "tool_calls": 18,
  "decisions":  9,
  "outcome":    "escaped"
}
```

This is uploaded to Azure Blob Storage at the end of every run (success or DNF) and fed into the leaderboard.

---

## Repository layout

```
roombai/
  ├── run.sh                  ← single entry point for a competition run
  ├── setup.sh                ← one-time Pi setup
  ├── CLAUDE.md               ← Claude's instructions (edit Escape Strategy only)
  ├── roomba_pilot/           ← Rust library + pilot binary (serial ↔ TCP bridge)
  └── scripts/
        ├── init_run.sh       ← creates attempt dir and state file (called by run.sh)
        ├── capture_frame.sh  ← captures a timestamped still (called by Claude)
        ├── finish_run.sh     ← writes metadata (called by Claude)
        ├── reset_run.sh      ← wipes /tmp and Claude memory (called by run.sh)
        └── upload_run.sh     ← uploads attempt dir to Azure Blob (called by run.sh)
```

---

## Pilot command reference

Commands are sent via: `./roomba_pilot/target/debug/pilot send "<command>"`

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
| `ping` | Health check |
| `shutdown` | Stop, return to passive, exit daemon |

---

## Strategy example

Here is an example of a complete `## Escape Strategy` section you could write. This is just one approach — you are free to design your own.

```markdown
## Escape Strategy

When asked to escape the room, act as Orchestrator and run the following loop:

### Phase 1 — Scout
Spin 360° in place, capturing a frame every 30° (12 frames total).
Analyse all frames to find the door:
- Look for a rectangular gap in the wall, change in flooring, or open space
- Estimate door bearing relative to current heading (0° = forward)
- Log: `scout: door_bearing=<deg> confidence=<0-1>`

If confidence < 0.5 after a full rotation, rotate another 180° and retry once.

### Phase 2 — Navigate
Tight loop until the door fills >50% of the frame:

1. Turn to align with door bearing: `pilot send "turn <deg>"`
2. Move forward 30 cm: `pilot send "move 30"`
3. Capture frame, re-assess door position and bearing
4. Check bumpers after every move: `pilot send "bumps"`
   - If bumped: back up 10 cm, turn 30° away from bump side, continue
5. If door is lost: run a 180° mini-scout to reacquire

Narrate each decision: `echo "moving toward door at 45 degrees" >> /tmp/speak_queue.txt`

### Phase 3 — Confirm escape
When the door fills the frame:
- Slow to 20 cm bursts through the doorframe
- Capture a frame on the other side — if the scene is clearly different, escaped
- Call `./scripts/finish_run.sh escaped`

### If stuck or time limit exceeded (3 minutes)
Call `./scripts/finish_run.sh dnf` and stop.
```

---

## Tips

- **Branch name = leaderboard name** — pick something recognisable, e.g. `alice/attempt-1`
- **Narrate decisions** — `echo "your message" >> /tmp/speak_queue.txt` speaks aloud during the run, useful for debugging without looking at the screen
- **Each frame is timestamped** — the frames show exactly what the robot saw and when
- **DNF runs still upload** — partial data is better than nothing; the leaderboard tracks all attempts
- **Do not call any external LLM APIs** — there is no `ANTHROPIC_API_KEY` available
