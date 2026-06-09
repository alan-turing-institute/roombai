#!/usr/bin/env bash
# capture_frame.sh — capture a timestamped still and save it to the current attempt directory.
#
# Usage:
#   source capture_frame.sh   (must be sourced to share ATTEMPT_DIR, START_TIME, FRAME_IDX)
#
# Expects these variables to be set in the calling environment:
#   ATTEMPT_DIR   — path to the current attempt directory
#   START_TIME    — unix timestamp of attempt start (from init_run.sh)
#   FRAME_IDX     — current frame counter (will be incremented after capture)

set -euo pipefail

ELAPSED=$(( $(date +%s) - START_TIME ))
ELAPSED_FMT=$(printf '%02d:%02d' $(( ELAPSED / 60 )) $(( ELAPSED % 60 )))
FRAME_IDX_PAD=$(printf '%04d' "$FRAME_IDX")

rpicam-still -o /tmp/frame_raw.jpg --nopreview -t 1 2>/dev/null
convert /tmp/frame_raw.jpg \
  -fill white -stroke black -strokewidth 1 \
  -pointsize 100 -annotate +10+44 "${ELAPSED_FMT}" \
  "${ATTEMPT_DIR}/frames/frame_${FRAME_IDX_PAD}.jpg"

echo "${ATTEMPT_DIR}/frames/frame_${FRAME_IDX_PAD}.jpg [${ELAPSED_FMT}]" >> /tmp/execution_log.txt

FRAME_IDX=$(( FRAME_IDX + 1 ))
