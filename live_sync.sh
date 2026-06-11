#!/usr/bin/env bash
# live_sync.sh — Fast 1-second sync of current frame + state from Pi to Mac.
#
# Run in the background before starting a supervised session:
#   ! bash live_sync.sh &
#
# Files synced to runs/live/:
#   roomba_current.jpg   latest camera frame
#   roomba_state.json    robot mode, heading, pos, obstacles, door detection
#   roomba_log.txt       full run log
#
# Kill this process to stop (kill %1 if backgrounded, or Ctrl-C if foreground).

PI_HOST="${2:-hackweek26@10.10.100.185}"
PI_PASS="${3:-aipi}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# First argument overrides the target directory; default is runs/live
if [[ -n "$1" ]]; then
    LIVE_DIR="$1"
else
    LIVE_DIR="$SCRIPT_DIR/runs/live"
fi

mkdir -p "$LIVE_DIR"
echo "[live_sync] syncing Pi → $LIVE_DIR every 1 s"
echo "[live_sync] stop with: kill \$!"

rsync_pi() {
    sshpass -p "$PI_PASS" rsync -qaz "$@" 2>/dev/null || true
}

n=0
while true; do
    rsync_pi \
        "$PI_HOST:/tmp/roomba_current.jpg" \
        "$PI_HOST:/tmp/roomba_state.json" \
        "$PI_HOST:/tmp/roomba_log.txt" \
        "$LIVE_DIR/"
    rsync_pi --ignore-existing \
        "$PI_HOST:/tmp/roomba_frames/" \
        "$LIVE_DIR/frames/"

    # Print a one-line heartbeat every 10 syncs so the terminal stays useful
    n=$(( n + 1 ))
    if (( n % 10 == 0 )); then
        mode=$(python3 -c "import json,sys; d=json.load(open('$LIVE_DIR/roomba_state.json')); print(d.get('mode','?'))" 2>/dev/null || echo "?")
        pos=$(python3 -c "import json,sys; d=json.load(open('$LIVE_DIR/roomba_state.json')); print(f\"({d.get('pos_x',0):.0f},{d.get('pos_y',0):.0f})cm\")" 2>/dev/null || echo "?")
        hdg=$(python3 -c "import json,sys; d=json.load(open('$LIVE_DIR/roomba_state.json')); print(f\"{d.get('heading',0):.0f}°\")" 2>/dev/null || echo "?")
        bumps=$(python3 -c "import json,sys; d=json.load(open('$LIVE_DIR/roomba_state.json')); print(d.get('bumps',0))" 2>/dev/null || echo "?")
        echo "[live_sync $(date +%H:%M:%S)] mode=$mode  pos=$pos  hdg=$hdg  bumps=$bumps"
    fi

    sleep 1
done
