#!/usr/bin/env bash
# sync_run_lynge.sh — Fast live sync of Pi run data to a local runs/ directory.
#
# Polls every 5 s so scan frames (kept for only ~40 s on Pi) are captured
# before they roll off the Pi's 20-frame rolling window.
#
# Usage:
#   bash sync_run_lynge.sh <local_run_dir> <pi_host> <pi_password>
#
# To stop: create the file  <local_run_dir>/.sync_stop
#
set -uo pipefail

RUN_DIR="$1"
PI_HOST="$2"
PI_PASS="$3"
STOP_FILE="$RUN_DIR/.sync_stop"

mkdir -p "$RUN_DIR/frames" "$RUN_DIR/maps"
rm -f "$STOP_FILE"

rsync_pi() {
    sshpass -p "$PI_PASS" rsync -az "$@" 2>/dev/null || true
}

echo "[sync] started — writing to $RUN_DIR (5 s interval)"

while [[ ! -f "$STOP_FILE" ]]; do
    TS=$(date +%H:%M:%S)

    # Logs: always re-download (they grow during the run)
    rsync_pi \
        "$PI_HOST:/tmp/roomba_log.txt" \
        "$PI_HOST:/tmp/roomba_state.json" \
        "$PI_HOST:/tmp/roomba_current.jpg" \
        "$PI_HOST:/tmp/pilot.log" \
        "$PI_HOST:/tmp/speak_queue.txt" \
        "$RUN_DIR/"

    # Frames: incremental — Pi keeps only last 20, so poll fast to capture scan frames
    rsync_pi --ignore-existing \
        "$PI_HOST:/tmp/roomba_frames/" \
        "$RUN_DIR/frames/"

    # Maps: incremental (new timestamped file each save)
    rsync_pi --ignore-existing \
        "$PI_HOST:/tmp/roomba_maps/" \
        "$RUN_DIR/maps/"

    frame_count=$(ls "$RUN_DIR/frames/" 2>/dev/null | wc -l | tr -d ' ')
    log_lines=$(wc -l < "$RUN_DIR/roomba_log.txt" 2>/dev/null || echo 0)
    echo "[$TS][sync] frames=$frame_count  log_lines=$log_lines"

    # 5-second interval — check stop file every second
    for _ in 1 2 3 4 5; do
        [[ -f "$STOP_FILE" ]] && break
        sleep 1
    done
done

echo "[sync] stop detected — final sync…"

rsync_pi \
    "$PI_HOST:/tmp/roomba_log.txt" \
    "$PI_HOST:/tmp/roomba_state.json" \
    "$PI_HOST:/tmp/roomba_current.jpg" \
    "$PI_HOST:/tmp/pilot.log" \
    "$PI_HOST:/tmp/speak_queue.txt" \
    "$RUN_DIR/"
rsync_pi --ignore-existing \
    "$PI_HOST:/tmp/roomba_frames/" \
    "$RUN_DIR/frames/"
rsync_pi --ignore-existing \
    "$PI_HOST:/tmp/roomba_maps/" \
    "$RUN_DIR/maps/"

frame_count=$(ls "$RUN_DIR/frames/" 2>/dev/null | wc -l | tr -d ' ')
log_lines=$(wc -l < "$RUN_DIR/roomba_log.txt" 2>/dev/null || echo 0)
echo "[sync] done — frames=$frame_count  log_lines=$log_lines  data at: $RUN_DIR"
