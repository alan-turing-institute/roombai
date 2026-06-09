#!/usr/bin/env bash
# reset_run.sh — wipe Claude project memory and /tmp run artifacts before a fresh attempt.
#
# Usage (must be sourced so the echo is visible in the current shell):
#   source reset_run.sh
#   . reset_run.sh

set -euo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"

# Derive the ~/.claude/projects key from the repo path (slashes → dashes)
PROJECT_KEY=$(echo "$REPO_ROOT" | sed 's|/|-|g')
CLAUDE_PROJECT_DIR="${HOME}/.claude/projects/${PROJECT_KEY}"

echo "=== RoombaI pre-run reset ==="

# 1. Clear Claude project memory
if [ -d "$CLAUDE_PROJECT_DIR" ]; then
    rm -f "${CLAUDE_PROJECT_DIR}"/*.jsonl
    echo "✓ Claude project memory cleared ($CLAUDE_PROJECT_DIR)"
else
    echo "  Claude project memory dir not found — skipping ($CLAUDE_PROJECT_DIR)"
fi

<<<<<<< HEAD
# 2. Clear RoombaI run artifacts from /tmp (leave system sockets/dirs alone)
rm -rf /tmp/escape_attempt_* \
       /tmp/run_state.env \
       /tmp/speak_queue.txt \
       /tmp/speak_daemon.log \
       /tmp/pilot_daemon.log \
       /tmp/execution_log.txt \
       /tmp/frame_*.jpg
echo "✓ Cleared RoombaI run artifacts from /tmp"
=======
# 2. Clear everything in /tmp (depth-first so dirs are emptied before deletion)
find /tmp -mindepth 1 -depth -delete 2>/dev/null || true
echo "✓ Cleared /tmp"
>>>>>>> 6dfcb022e2b6b842b1d5f10be6efe20f007695b0

echo ""
echo "Reset complete."
