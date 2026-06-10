#!/usr/bin/env bash
# run.sh — full competition run: reset, start daemons, launch Claude, upload.
#
# Usage:
#   ./run.sh [serial_port]
#
# Default serial port: /dev/ttyUSB0

set -euo pipefail

SERIAL="${1:-/dev/ttyUSB0}"
REPO_ROOT="$(git rev-parse --show-toplevel)"

# ── Step 1: clear /tmp and Claude memory ─────────────────────────────────────
echo ""
echo "━━━ Step 1/8 — Resetting state ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
source "${REPO_ROOT}/scripts/reset_run.sh"

# ── Step 2: Roomba power check ───────────────────────────────────────────────
echo ""
echo "━━━ Step 2/8 — Roomba power check ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Is the Roomba powered on and connected to ${SERIAL}? [y/N]"
read -r answer
if [[ ! "$answer" =~ ^[Yy]$ ]]; then
    echo "Power on the Roomba and re-run this script."
    exit 1
fi

# ── Step 3: TTS daemon ───────────────────────────────────────────────────────
echo ""
echo "━━━ Step 3/8 — Starting TTS daemon ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
# Kill any stale TTS daemon leftover from a previous run
pkill -f 'tail -n 0 -f /tmp/speak_queue.txt' 2>/dev/null || true
touch /tmp/speak_queue.txt
nohup bash -c 'tail -n 0 -f /tmp/speak_queue.txt | while IFS= read -r line; do espeak-ng -s 145 -- "$line" 2>/dev/null; done' \
  > /tmp/speak_daemon.log 2>&1 &
echo "✓ TTS daemon started"

# ── Step 4: pilot daemon ─────────────────────────────────────────────────────
echo ""
echo "━━━ Step 4/8 — Starting pilot daemon ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
# Kill any stale pilot daemon still holding the serial port from a previous run
if fuser "$SERIAL" >/dev/null 2>&1; then
    echo "Port ${SERIAL} busy — killing stale process holding it..."
    fuser -k "$SERIAL" 2>/dev/null || true
    sleep 1
fi
cd "${REPO_ROOT}/roomba_pilot"
nohup ./target/debug/pilot serve "$SERIAL" > /tmp/pilot_daemon.log 2>&1 &
PILOT_PID=$!
echo "Pilot daemon started (PID ${PILOT_PID}), waiting for SAFE mode..."

for i in $(seq 1 15); do
    sleep 1
    if grep -q "robot in SAFE mode" /tmp/pilot_daemon.log 2>/dev/null; then
        echo "✓ Robot in SAFE mode"
        break
    fi
    if ! kill -0 "$PILOT_PID" 2>/dev/null; then
        echo "✗ Pilot daemon exited unexpectedly. Check /tmp/pilot_daemon.log"
        exit 1
    fi
    if [ "$i" -eq 15 ]; then
        echo "✗ Timed out waiting for SAFE mode. Check /tmp/pilot_daemon.log"
        exit 1
    fi
done

# ── Step 5: initialise run directory ────────────────────────────────────────
echo ""
echo "━━━ Step 5/8 — Initialising run ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
cd "${REPO_ROOT}"
bash "${REPO_ROOT}/scripts/init_run.sh"

# ── Step 6: launch Claude (hard 10-minute time limit) ────────────────────────
echo ""
echo "━━━ Step 6/8 — Launching Claude (10-minute limit) ━━━━━━━━━━━━━━━━━━━━━━"
TIME_LIMIT_S=600   # 10 minutes
echo "  Claude session is limited to $((TIME_LIMIT_S / 60)) minutes."
echo "  After the limit it is stopped, the timelapse is stitched, and uploaded."

# Run Claude under `timeout`: SIGTERM at the limit, SIGKILL 10 s later if it
# hasn't exited. Don't let a non-zero exit abort the script — we still want to
# finalise and upload whatever the run produced.
#
# --foreground is REQUIRED here: run.sh is a script, so without it `timeout`
# puts claude in its own process group (not the terminal's foreground group),
# and claude's TUI hangs on SIGTTIN the moment it reads the TTY. --foreground
# keeps claude in the foreground group so it can drive the terminal.
CLAUDE_RC=0
timeout --foreground -k 10 "${TIME_LIMIT_S}" claude "/escape" || CLAUDE_RC=$?

if [ "$CLAUDE_RC" -eq 124 ]; then
    echo "⏱  10-minute limit reached — Claude session stopped."
    RUN_OUTCOME="timeout"
elif [ "$CLAUDE_RC" -ne 0 ]; then
    echo "  Claude exited with code ${CLAUDE_RC}."
    RUN_OUTCOME="dnf"
else
    echo "✓ Claude session ended on its own."
    RUN_OUTCOME="dnf"
fi

# ── Step 7: stop the robot + finalise (stitch timelapse video) ───────────────
echo ""
echo "━━━ Step 7/8 — Finalising attempt ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
# Halt the robot in case it was mid-motion when Claude was stopped.
( cd "${REPO_ROOT}/roomba_pilot" && ./target/debug/pilot send "stop" >/dev/null 2>&1 ) || true

# Claude normally calls finish_run.sh itself at the end of /escape (which stitches
# the timelapse and writes metadata). If it was stopped early it won't have, so
# we stitch the video here — but only if it doesn't already exist, to avoid
# clobbering a clean outcome Claude already recorded.
ATTEMPT_DIR=""
# shellcheck disable=SC1091
[ -f /tmp/run_state.env ] && source /tmp/run_state.env
if [ -n "${ATTEMPT_DIR}" ] && [ ! -f "${ATTEMPT_DIR}/timelapse.mp4" ]; then
    echo "  No timelapse found — stitching it now (outcome: ${RUN_OUTCOME})..."
    bash "${REPO_ROOT}/scripts/finish_run.sh" "${RUN_OUTCOME}" || \
        echo "  ⚠ finish_run.sh failed — uploading whatever exists."
else
    echo "✓ Timelapse already produced by Claude."
fi

# ── Step 8: upload run artifacts to Azure Blob ───────────────────────────────
echo ""
echo "━━━ Step 8/8 — Uploading run to Azure Blob Storage ━━━━━━━━━━━━━━━━━━━━━"
LATEST_ATTEMPT=$(ls -dt /tmp/escape_attempt_* 2>/dev/null | head -1 || true)
if [ -z "$LATEST_ATTEMPT" ]; then
    echo "  No attempt directory found in /tmp — nothing to upload."
else
    "${REPO_ROOT}/scripts/upload_run.sh" "$LATEST_ATTEMPT"
fi
