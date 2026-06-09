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

# 2. Clear escape attempt directories from /tmp
ATTEMPT_DIRS=( $(ls -d /tmp/escape_attempt_* 2>/dev/null || true) )
if [ ${#ATTEMPT_DIRS[@]} -gt 0 ]; then
    rm -rf "${ATTEMPT_DIRS[@]}"
    echo "✓ Removed ${#ATTEMPT_DIRS[@]} attempt director(ies) from /tmp"
else
    echo "  No attempt directories found in /tmp — skipping"
fi

# 3. Clear execution log
if [ -f /tmp/execution_log.txt ]; then
    rm -f /tmp/execution_log.txt
    echo "✓ Cleared /tmp/execution_log.txt"
fi

# 4. Clear TTS queue
if [ -f /tmp/speak_queue.txt ]; then
    truncate -s 0 /tmp/speak_queue.txt
    echo "✓ Cleared /tmp/speak_queue.txt"
fi

echo ""
echo "Ready. Start the pilot daemon and run /escape."
