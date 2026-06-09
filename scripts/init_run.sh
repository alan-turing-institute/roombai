#!/usr/bin/env bash
# init_run.sh — initialise a new escape attempt directory and counters.
#
# Usage:
#   source init_run.sh   (must be sourced to export variables to the calling shell)
#
# Exports:
#   ATTEMPT_DIR, PILOT_NAME, COMMIT_HASH, START_TIME, TOOL_CALLS, DECISIONS, FRAME_IDX

set -euo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"

ATTEMPT_N=$(( $(ls -d /tmp/escape_attempt_* 2>/dev/null | wc -l) + 1 ))
export ATTEMPT_DIR=/tmp/escape_attempt_${ATTEMPT_N}
mkdir -p "${ATTEMPT_DIR}/frames"

export PILOT_NAME=$(git -C "$REPO_ROOT" rev-parse --abbrev-ref HEAD)
export COMMIT_HASH=$(git -C "$REPO_ROOT" rev-parse --short HEAD)
export START_TIME=$(date +%s)
export TOOL_CALLS=0
export DECISIONS=0
export FRAME_IDX=1

echo "attempt_dir=${ATTEMPT_DIR}" >> /tmp/execution_log.txt
echo "pilot=${PILOT_NAME} commit=${COMMIT_HASH}" >> /tmp/execution_log.txt
echo "Run initialised: ${ATTEMPT_DIR} (pilot: ${PILOT_NAME}, commit: ${COMMIT_HASH})"
