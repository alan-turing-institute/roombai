#!/usr/bin/env bash
# pilot_send.sh — send one pilot command and increment TOOL_CALLS.
#
# Usage:
#   ./scripts/pilot_send.sh "move 200"
#   ./scripts/pilot_send.sh "safe"
#
# Prints the pilot's reply line. Bumps TOOL_CALLS in /tmp/run_state.env.

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PILOT="${ROOT}/roomba_pilot/target/debug/pilot"
STATE_FILE="/tmp/run_state.env"

CMD="${1:?usage: pilot_send.sh \"<command>\"}"

"$PILOT" send "$CMD"

if [ -f "$STATE_FILE" ]; then
  tc=$(grep '^TOOL_CALLS=' "$STATE_FILE" | cut -d= -f2 || echo 0)
  tmp="${STATE_FILE}.tmp"; grep -v '^TOOL_CALLS=' "$STATE_FILE" > "$tmp" || true
  echo "TOOL_CALLS=$(( ${tc:-0} + 1 ))" >> "$tmp"; mv "$tmp" "$STATE_FILE"
fi
