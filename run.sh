#!/usr/bin/env bash
# start_run.sh — full pre-run setup: reset state, start pilot daemon, launch Claude.
#
# Usage:
#   ./start_run.sh [serial_port]
#
# Default serial port: /dev/ttyUSB0

set -euo pipefail

SERIAL="${1:-/dev/ttyUSB0}"
REPO_ROOT="$(git rev-parse --show-toplevel)"

# ── Step 1: reset ────────────────────────────────────────────────────────────
echo ""
echo "━━━ Step 1/3 — Resetting state ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
source "${REPO_ROOT}/scripts/reset_run.sh"

# ── Step 2: Roomba check ─────────────────────────────────────────────────────
echo ""
echo "━━━ Step 2/4 — Roomba power check ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Is the Roomba powered on and connected to ${SERIAL}? [y/N]"
read -r answer
if [[ ! "$answer" =~ ^[Yy]$ ]]; then
    echo "Power on the Roomba and re-run this script."
    exit 1
fi

# ── Step 3: TTS daemon ───────────────────────────────────────────────────────
echo ""
echo "━━━ Step 3/5 — Starting TTS daemon ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
touch /tmp/speak_queue.txt
nohup bash -c 'tail -n 0 -f /tmp/speak_queue.txt | while IFS= read -r line; do espeak-ng -s 145 -- "$line" 2>/dev/null; done' \
  > /tmp/speak_daemon.log 2>&1 &
echo "✓ TTS daemon started"

# ── Step 4: pilot daemon ─────────────────────────────────────────────────────
echo ""
echo "━━━ Step 4/5 — Starting pilot daemon ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
cd "${REPO_ROOT}/roomba_pilot"
nohup ./target/debug/pilot serve "$SERIAL" > /tmp/pilot_daemon.log 2>&1 &
PILOT_PID=$!
echo "Pilot daemon started (PID ${PILOT_PID}), waiting for SAFE mode..."

# Wait until the daemon confirms SAFE mode (or fail after 15s)
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

# ── Step 4: launch Claude and run /escape ───────────────────────────────────
echo ""
echo "━━━ Step 5/6 — Launching Claude ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Pilot log: /tmp/pilot_daemon.log"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
cd "${REPO_ROOT}"
claude "/escape"

# ── Step 5: upload run artifacts to OneDrive ─────────────────────────────────
echo ""
echo "━━━ Step 6/6 — Uploading run to OneDrive ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
LATEST_ATTEMPT=$(ls -dt /tmp/escape_attempt_* 2>/dev/null | head -1)
if [ -z "$LATEST_ATTEMPT" ]; then
    echo "  No attempt directory found in /tmp — nothing to upload."
else
    "${REPO_ROOT}/scripts/upload_run.sh" "$LATEST_ATTEMPT"
fi
