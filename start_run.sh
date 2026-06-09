#!/usr/bin/env bash
# start_run.sh — full pre-run setup: reset state, start pilot daemon.
#
# Usage:
#   ./start_run.sh [serial_port]
#
# Default serial port: /dev/ttyUSB0
# After this script exits, open Claude Code and run /escape.

set -euo pipefail

SERIAL="${1:-/dev/ttyUSB0}"
REPO_ROOT="$(git rev-parse --show-toplevel)"

# ── Step 1: reset ────────────────────────────────────────────────────────────
echo ""
echo "━━━ Step 1/3 — Resetting state ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
source "${REPO_ROOT}/reset_run.sh"

# ── Step 2: Roomba check ─────────────────────────────────────────────────────
echo ""
echo "━━━ Step 2/3 — Roomba power check ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Is the Roomba powered on and connected to ${SERIAL}? [y/N]"
read -r answer
if [[ ! "$answer" =~ ^[Yy]$ ]]; then
    echo "Power on the Roomba and re-run this script."
    exit 1
fi

# ── Step 3: pilot daemon ─────────────────────────────────────────────────────
echo ""
echo "━━━ Step 3/3 — Starting pilot daemon ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
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

# ── Ready ────────────────────────────────────────────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✓ All systems go. Open Claude Code and run /escape"
echo "  Pilot log: /tmp/pilot_daemon.log"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
