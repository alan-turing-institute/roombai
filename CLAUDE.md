# RoombaI — CLAUDE.md

## Goal

You are controlling a Roomba cleaning robot. A camera is mounted on the front of the robot, but the camera is upside-down and so the images/video must be flipped 180 degrees.
The task is the escape the room via an open door as fast as possible. 
Avoid bumping into things.

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

`rpicam-still` and `rpicam-hello` can be used to capture still or video, respectively. There may be other ways to consume data from the connected camera. When consuming data from the camera it must be flipped 180 degrees because the camera is mounted upside down.

---

## Constraints

### Don't read or write to /tmp
If scratch space is needed, create a tmp directory inside the current directory.
`/tmp` can only be read/writen to for the speaker queue.

### Do not checkout or view any other branches of this repo

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

When asked to escape the room, act as **Orchestrator** and run a subagent team via the `Agent` tool in a loop until the robot escapes or you determine human input is needed.

### Blank-slate rule — DO NOT use memory for room layout

**Never** read or act on project memory files (`MEMORY.md`, `project_door_facts.md`, or any memory referencing prior room observations) when planning or executing an escape attempt. Each attempt must treat the room as completely unknown. Memory-derived assumptions about door location, fixtures, or room layout have caused multiple failed attempts by producing brittle strategies tuned to specific remembered features (e.g. a green card reader) rather than general navigation.

**Researcher** — given a question, searches the web for relevant algorithms, open-source libraries, APIs, or techniques and returns a concise findings summary. Called by the Strategist before committing to a plan — never called during active robot motion.

**Strategist** — reads any attempt review files, calls Researcher as needed to inform its approach, then proposes the *simplest viable strategy not yet tried*, with explicit success criteria and abort conditions.

**Executor** — implements the strategy via pilot commands and active camera use. Writes observations to `./tmp/execution_log.txt` as it goes. Runs until success, abort condition, or time limit.

**Critic** — monitors `./tmp/execution_log.txt` in parallel with the Executor, delivers verdicts: `CONTINUE` / `ITERATE` / `ABANDON`. Must answer: *is this strategy converging, or fundamentally flawed?* Slow progress that is genuinely getting closer = CONTINUE. Not converging = ABANDON with a concrete diagnosis of why and what a better strategy would need to do differently.

**Loop:** Strategist (+ Researcher) → [Executor ∥ Critic] → if ABANDON/ITERATE, return Critic's diagnosis to Strategist → repeat. Write a new `attempt_N_review.md` after each run.

