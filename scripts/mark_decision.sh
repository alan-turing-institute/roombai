#!/usr/bin/env bash
# Increment DECISIONS in /tmp/run_state.env.

set -euo pipefail

STATE_FILE="/tmp/run_state.env"

if [ ! -f "$STATE_FILE" ]; then
    echo "Error: state file not found at ${STATE_FILE}. Was init_run.sh run?" >&2
    exit 1
fi

tmp="${STATE_FILE}.$$"
awk -F= '
    BEGIN { OFS = FS }
    $1 == "DECISIONS" { $2 = $2 + 1 }
    { print }
' "$STATE_FILE" > "$tmp"
mv "$tmp" "$STATE_FILE"
