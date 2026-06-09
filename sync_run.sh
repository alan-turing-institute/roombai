#!/usr/bin/env bash
# sync_run.sh — Live sync of Pi run data to a local runs/<timestamp>/ directory.
#
# Runs every 60 s in the background while explore.py is executing on the Pi.
# Frames are downloaded incrementally (already-downloaded files are skipped).
# All logs are re-downloaded every cycle because they grow during the run.
#
# Usage:
#   bash sync_run.sh <local_run_dir> <pi_host> <pi_password>
#
# To stop: create the file  <local_run_dir>/.sync_stop
# (run.md does this automatically when the SSH session ends)
#
set -uo pipefail

RUN_DIR="$1"
PI_HOST="$2"
PI_PASS="$3"
STOP_FILE="$RUN_DIR/.sync_stop"

mkdir -p "$RUN_DIR/frames"
rm -f "$STOP_FILE"

rsync_pi() {
    sshpass -p "$PI_PASS" rsync -az --ignore-missing-args "$@" 2>/dev/null || true
}

echo "[sync] started — writing to $RUN_DIR"
echo "[sync] frames: incremental (skip existing)   logs: always refreshed"

while [[ ! -f "$STOP_FILE" ]]; do
    TS=$(date +%H:%M:%S)

    # ── Logs: always re-download (they grow during the run) ────────────────
    rsync_pi \
        "$PI_HOST:/tmp/roomba_log.txt" \
        "$PI_HOST:/tmp/roomba_state.json" \
        "$PI_HOST:/tmp/roomba_current.jpg" \
        "$PI_HOST:/tmp/pilot.log" \
        "$PI_HOST:/tmp/speak_queue.txt" \
        "$RUN_DIR/"

    # ── Frames: incremental — never re-download a file already present ─────
    # frame_XXXXX.jpg files are written once and never modified on the Pi,
    # so --ignore-existing is safe and avoids re-transferring large batches.
    rsync_pi --ignore-existing \
        "$PI_HOST:/tmp/roomba_frames/" \
        "$RUN_DIR/frames/"

    # ── Maps: re-download (updated when the run ends) ─────────────────────
    rsync_pi \
        "$PI_HOST:/tmp/roomba_map*.png" \
        "$PI_HOST:/tmp/roomba_map*.json" \
        "$RUN_DIR/" 2>/dev/null || true

    frame_count=$(ls "$RUN_DIR/frames/" 2>/dev/null | wc -l | tr -d ' ')
    log_lines=$(wc -l < "$RUN_DIR/roomba_log.txt" 2>/dev/null || echo 0)
    echo "[$TS][sync] frames=$frame_count  log_lines=$log_lines"

    # Sleep in 5-second increments so the stop file is noticed quickly
    for _ in $(seq 1 12); do
        [[ -f "$STOP_FILE" ]] && break
        sleep 5
    done
done

echo "[sync] stop file detected — running final sync…"

# ── Final sync: one last pass to catch everything written at run end ────────
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

rsync_pi \
    "$PI_HOST:/tmp/roomba_map*.png" \
    "$PI_HOST:/tmp/roomba_map*.json" \
    "$RUN_DIR/" 2>/dev/null || true

frame_count=$(ls "$RUN_DIR/frames/" 2>/dev/null | wc -l | tr -d ' ')
log_lines=$(wc -l < "$RUN_DIR/roomba_log.txt" 2>/dev/null || echo 0)
echo "[sync] done — frames=$frame_count  log_lines=$log_lines  data at: $RUN_DIR"
