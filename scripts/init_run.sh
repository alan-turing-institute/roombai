#!/usr/bin/env bash
# init_run.sh — initialise a new escape attempt directory and write run state.
#
# Called by run.sh before launching Claude. Writes all state to
# /tmp/run_state.env so it is available to capture_frame.sh and finish_run.sh
# regardless of subshell boundaries.

set -euo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"
STATE_FILE="/tmp/run_state.env"

shopt -s nullglob
existing_attempts=(/tmp/escape_attempt_*)
ATTEMPT_N=$(( ${#existing_attempts[@]} + 1 ))
ATTEMPT_DIR="/tmp/escape_attempt_${ATTEMPT_N}"
mkdir -p "${ATTEMPT_DIR}/frames"

PILOT_NAME=$(git -C "$REPO_ROOT" rev-parse --abbrev-ref HEAD)
COMMIT_HASH=$(git -C "$REPO_ROOT" rev-parse --short HEAD)
START_TIME=$(date +%s)

cat > "$STATE_FILE" <<EOF
ATTEMPT_DIR=${ATTEMPT_DIR}
PILOT_NAME=${PILOT_NAME}
COMMIT_HASH=${COMMIT_HASH}
START_TIME=${START_TIME}
TOOL_CALLS=0
DECISIONS=0
FRAME_IDX=1
EOF

echo "attempt_dir=${ATTEMPT_DIR}" >> /tmp/execution_log.txt
echo "pilot=${PILOT_NAME} commit=${COMMIT_HASH}" >> /tmp/execution_log.txt
echo "✓ Run initialised: ${ATTEMPT_DIR} (pilot: ${PILOT_NAME}, commit: ${COMMIT_HASH})"
