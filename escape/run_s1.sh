#!/usr/bin/env bash
# One-command launcher for the S1 characterization session (see RUNBOOK_S1.md).
#
#   ./run_s1.sh                      # port /dev/ttyUSB0, output /tmp/s1
#   ./run_s1.sh /dev/ttyUSB0 /tmp/s1 # explicit
#   ./run_s1.sh /dev/ttyUSB0 /tmp/s1 --from C   # resume from a phase
#
# It builds, ensures the TTS daemon is up, runs preflight reminders, then
# launches the runner. Ctrl-C stops the wheels within one tick.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # escape/
REPO="$(cd "$HERE/.." && pwd)"                          # repo root (has speak_daemon.sh)
PORT="${1:-/dev/ttyUSB0}"
OUT="${2:-/tmp/s1}"
EXTRA=("${@:3}")

echo "== S1 characterization =="

# 1. Build first, so a compile error never wastes robot time.
echo "building (release)..."
( cd "$HERE" && cargo build --release -p characterize )
BIN="$HERE/target/release/characterize"

# 2. TTS daemon (idempotent).
if ! pgrep -f speak_daemon.sh >/dev/null 2>&1; then
  echo "starting TTS daemon..."
  nohup "$REPO/speak_daemon.sh" >/tmp/speak_daemon.log 2>&1 &
  sleep 1
fi
echo "S1 ready" >> /tmp/speak_queue.txt

# 3. Preflight checks + reminders.
if pgrep -f "pilot serve" >/dev/null 2>&1; then
  echo "!! a 'pilot serve' process is running and owns $PORT — stop it first (pilot shutdown)."
fi
if [ ! -e "$PORT" ]; then
  echo "!! serial port $PORT not found — is the robot connected/powered?"
fi
command -v rpicam-vid >/dev/null 2>&1 || echo "!! rpicam-vid not found — video will be skipped (sensors still logged)."
command -v espeak-ng >/dev/null 2>&1 || echo "!! espeak-ng not found — narration will be silent (text still printed)."

cat <<EOF

PREFLIGHT (see RUNBOOK_S1.md):
  - Roomba powered ON, on its start spot, facing a wall ~1.5-2 m ahead.
  - ~1 m clear to the LEFT for the phase-D arc; stand clear of the lane.
  - serial: $PORT    output: $OUT
EOF
read -r -p "Press Enter to start (Ctrl-C to abort)... "

# 4. Run the session.
"$BIN" --port "$PORT" --out "$OUT" "${EXTRA[@]}"

cat <<EOF

Session finished. Data in $OUT:
  session.jsonl  (20 Hz sensor + odometry log)
  summary.json   (the 770 capability/odometry truth table)
  video.h264     (motion phases; play: ffplay, or wrap to mp4)
  notes_template.md  (jot context now, while fresh)

Copy it off the Pi:  scp -r <pi-host>:$OUT .
EOF
