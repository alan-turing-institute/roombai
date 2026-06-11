#!/usr/bin/env bash
# capture_model.sh — SessionStart hook. Reads the hook JSON on stdin and
# persists the active model into /tmp/run_state.env so finish_run.sh can
# record which Claude Code model drove the run.

set -euo pipefail

STATE_FILE="/tmp/run_state.env"
PAYLOAD="$(cat)"

MODEL_ID="$(printf '%s' "$PAYLOAD" | jq -r '.model.id // "unknown"')"
MODEL_NAME="$(printf '%s' "$PAYLOAD" | jq -r '.model.display_name // "unknown"')"

# Append (or refresh) the model fields in the shared state file.
if [ -f "$STATE_FILE" ]; then
    sed -i '/^MODEL_ID=/d;/^MODEL_NAME=/d' "$STATE_FILE"
fi
{
    echo "MODEL_ID=\"${MODEL_ID}\""
    echo "MODEL_NAME=\"${MODEL_NAME}\""
} >> "$STATE_FILE"
