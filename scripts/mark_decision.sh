#!/usr/bin/env bash
# mark_decision.sh — increment the DECISIONS counter in /tmp/run_state.env.
#
# Usage (after analysing a frame):
#   ./scripts/mark_decision.sh

set -euo pipefail
STATE_FILE="/tmp/run_state.env"

if [ -f "$STATE_FILE" ]; then
  d=$(grep '^DECISIONS=' "$STATE_FILE" | cut -d= -f2 || echo 0)
  tmp="${STATE_FILE}.tmp"; grep -v '^DECISIONS=' "$STATE_FILE" > "$tmp" || true
  echo "DECISIONS=$(( ${d:-0} + 1 ))" >> "$tmp"; mv "$tmp" "$STATE_FILE"
fi
