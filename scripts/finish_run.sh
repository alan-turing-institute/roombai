#!/usr/bin/env bash
# finish_run.sh — stitch timelapse and write metadata at the end of an attempt.
#
# Usage (called by Claude at end of run):
#   ./scripts/finish_run.sh escaped   # or: ./scripts/finish_run.sh dnf
#
# Reads all state from /tmp/run_state.env.

set -euo pipefail

OUTCOME="${1:-dnf}"
STATE_FILE="/tmp/run_state.env"

if [ ! -f "$STATE_FILE" ]; then
    echo "Error: state file not found at ${STATE_FILE}. Was init_run.sh run?"
    exit 1
fi

# Load state
source "$STATE_FILE"

DURATION_S=$(( $(date +%s) - START_TIME ))

# Stitch timelapse
echo "Stitching timelapse..."
ffmpeg -y -framerate 2 -pattern_type glob -i "${ATTEMPT_DIR}/frames/*.jpg" \
  -c:v libx264 -pix_fmt yuv420p "${ATTEMPT_DIR}/timelapse.mp4" 2>/dev/null
echo "✓ Timelapse saved to ${ATTEMPT_DIR}/timelapse.mp4"

# Write metadata
cat > "${ATTEMPT_DIR}/metadata.json" <<EOF
{
  "name":       "${PILOT_NAME}",
  "commit":     "${COMMIT_HASH}",
  "date":       "$(date +%Y-%m-%d)",
  "duration_s": ${DURATION_S},
  "tool_calls": ${TOOL_CALLS},
  "decisions":  ${DECISIONS},
  "outcome":    "${OUTCOME}"
}
EOF
echo "✓ Metadata written to ${ATTEMPT_DIR}/metadata.json"

DURATION_FMT=$(printf '%02d:%02d' $(( DURATION_S / 60 )) $(( DURATION_S % 60 )))
echo "outcome=${OUTCOME} duration=${DURATION_FMT} tool_calls=${TOOL_CALLS} decisions=${DECISIONS}" \
  >> /tmp/execution_log.txt
echo "Attempt complete — outcome: ${OUTCOME}, duration: ${DURATION_FMT}"
