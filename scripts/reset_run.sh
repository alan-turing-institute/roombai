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

# 2. Clear everything in /tmp
find /tmp -mindepth 1 -delete 2>/dev/null || true
echo "✓ Cleared /tmp"

echo ""
echo "Reset complete."
