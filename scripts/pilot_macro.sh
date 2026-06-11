#!/usr/bin/env bash
# pilot_macro.sh — send several pilot commands back-to-back in ONE tool call,
# waiting for each timed motion to actually finish before sending the next.
#
# Why this exists: every motion command (move/turn/forward/back/spin) is
# fire-and-forget — `pilot send` returns instantly while the robot keeps moving
# until its watchdog deadline. Sending the next command immediately would clobber
# the one in flight. This computes each command's duration client-side and sleeps
# for it (plus a small settle buffer) so a chained route executes correctly.
#
# Usage:
#   ./scripts/pilot_macro.sh "turn 25" "move 200" "move 200" "move 80"
#
# Increments TOOL_CALLS once per command. This is the preferred way to drive a
# planned route to the door: one tool call, no thinking between moves.

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PILOT="${ROOT}/roomba_pilot/target/debug/pilot"
STATE_FILE="/tmp/run_state.env"

MOVE_SPEED_CM_S=20.0     # must match pilot's MOVE_SPEED_CM_S
TURN_RATE_DEG_S=60.0     # must match pilot's TURN_RATE_DEG_S
SETTLE=0.4               # buffer after a timed move so motion fully stops

# Duration (seconds) a command will keep the robot moving.
cmd_duration() {
  local verb="$1" a="${2:-0}" b="${3:-0}"
  case "$verb" in
    move)            awk -v v="$a" -v s="$MOVE_SPEED_CM_S" 'BEGIN{printf "%.2f", (v<0?-v:v)/s}' ;;
    turn)            awk -v v="$a" -v r="$TURN_RATE_DEG_S" 'BEGIN{printf "%.2f", (v<0?-v:v)/r}' ;;
    forward|back)    awk -v s="$b" 'BEGIN{printf "%.2f", (s<0?0:s)}' ;;
    spin)            awk -v s="$b" 'BEGIN{printf "%.2f", (s<0?0:s)}' ;;
    go)              echo "3.0" ;;
    *)               echo "0" ;;   # safe/stop/bumps/sense/full/dock/ping/motors
  esac
}

for CMD in "$@"; do
  # shellcheck disable=SC2086
  set -- $CMD
  verb="$1"
  echo "→ $CMD"
  "$PILOT" send "$CMD"

  if [ -f "$STATE_FILE" ]; then
    tc=$(grep '^TOOL_CALLS=' "$STATE_FILE" | cut -d= -f2 || echo 0)
    tmp="${STATE_FILE}.tmp"; grep -v '^TOOL_CALLS=' "$STATE_FILE" > "$tmp" || true
    echo "TOOL_CALLS=$(( ${tc:-0} + 1 ))" >> "$tmp"; mv "$tmp" "$STATE_FILE"
  fi

  dur=$(cmd_duration "$verb" "${2:-0}" "${3:-0}")
  wait_for=$(awk -v d="$dur" -v s="$SETTLE" 'BEGIN{ printf "%.2f", (d>0 ? d+s : 0.2) }')
  sleep "$wait_for"
done
