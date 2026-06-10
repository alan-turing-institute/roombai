#!/usr/bin/env bash
# capture_frame.sh — capture a timestamped still and save it to the attempt directory.
#
# Usage (called by Claude at every decision point):
#   source ./scripts/capture_frame.sh
#   DECISIONS=$(( DECISIONS + 1 ))
#
# Reads and updates state from /tmp/run_state.env.

set -euo pipefail

STATE_FILE="/tmp/run_state.env"

if [ ! -f "$STATE_FILE" ]; then
    echo "Error: state file not found at ${STATE_FILE}. Was init_run.sh run?"
    exit 1
fi

# Load state
source "$STATE_FILE"

ELAPSED=$(( $(date +%s) - START_TIME ))
ELAPSED_FMT=$(printf '%02d:%02d' $(( ELAPSED / 60 )) $(( ELAPSED % 60 )))
FRAME_IDX_PAD=$(printf '%04d' "$FRAME_IDX")

rpicam-still -o /tmp/frame_raw.jpg --nopreview -t 1 2>/dev/null
convert /tmp/frame_raw.jpg \
  -fill white -stroke black -strokewidth 1 \
  -pointsize 100 -annotate +10+44 "${ELAPSED_FMT}" \
  "${ATTEMPT_DIR}/frames/frame_${FRAME_IDX_PAD}.jpg"

echo "${ATTEMPT_DIR}/frames/frame_${FRAME_IDX_PAD}.jpg [${ELAPSED_FMT}]" >> /tmp/execution_log.txt

# Update counters in state file
FRAME_IDX=$(( FRAME_IDX + 1 ))
sed -i "s/^FRAME_IDX=.*/FRAME_IDX=${FRAME_IDX}/" "$STATE_FILE"
