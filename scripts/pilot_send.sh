#!/usr/bin/env bash
# Send one pilot command and increment TOOL_CALLS in /tmp/run_state.env.

set -euo pipefail

if [ "$#" -lt 1 ]; then
    echo "usage: $0 <pilot command...>" >&2
    exit 2
fi

REPO_ROOT="$(git rev-parse --show-toplevel)"
STATE_FILE="/tmp/run_state.env"
CMD="$*"

"${REPO_ROOT}/roomba_pilot/target/debug/pilot" send "$CMD"

if [ -f "$STATE_FILE" ]; then
    tmp="${STATE_FILE}.$$"
    awk -F= '
        BEGIN { OFS = FS }
        $1 == "TOOL_CALLS" { $2 = $2 + 1 }
        { print }
    ' "$STATE_FILE" > "$tmp"
    mv "$tmp" "$STATE_FILE"
fi
