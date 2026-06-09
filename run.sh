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
echo "━━━ Step 1/6 — Resetting state ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
source "${REPO_ROOT}/scripts/reset_run.sh"

# ── Step 2: Roomba power check ───────────────────────────────────────────────
echo ""
echo "━━━ Step 2/6 — Roomba power check ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Is the Roomba powered on and connected to ${SERIAL}? [y/N]"
read -r answer
if [[ ! "$answer" =~ ^[Yy]$ ]]; then
    echo "Power on the Roomba and re-run this script."
    exit 1
fi

# ── Step 3: TTS daemon ───────────────────────────────────────────────────────
echo ""
echo "━━━ Step 3/6 — Starting TTS daemon ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
# Kill any stale TTS daemon leftover from a previous run
pkill -f 'tail -n 0 -f /tmp/speak_queue.txt' 2>/dev/null || true
touch /tmp/speak_queue.txt
nohup bash -c 'tail -n 0 -f /tmp/speak_queue.txt | while IFS= read -r line; do espeak-ng -s 145 -- "$line" 2>/dev/null; done' \
  > /tmp/speak_daemon.log 2>&1 &
echo "✓ TTS daemon started"

# ── Step 4: pilot daemon ─────────────────────────────────────────────────────
echo ""
echo "━━━ Step 4/6 — Starting pilot daemon ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
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
echo "━━━ Step 5/6 — Initialising run ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
cd "${REPO_ROOT}"
bash "${REPO_ROOT}/scripts/init_run.sh"

# ── Step 6: launch Claude ────────────────────────────────────────────────────
echo ""
echo "━━━ Step 6/7 — Launching Claude ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
claude "/escape"

# ── Step 6: upload run artifacts to Azure Blob ───────────────────────────────
echo ""
echo "━━━ Step 7/7 — Uploading run to Azure Blob Storage ━━━━━━━━━━━━━━━━━━━━━"
LATEST_ATTEMPT=$(ls -dt /tmp/escape_attempt_* 2>/dev/null | head -1 || true)
if [ -z "$LATEST_ATTEMPT" ]; then
    echo "  No attempt directory found in /tmp — nothing to upload."
else
    "${REPO_ROOT}/scripts/upload_run.sh" "$LATEST_ATTEMPT"
fi
