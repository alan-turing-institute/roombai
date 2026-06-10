#!/usr/bin/env bash
# drive_until_bump.sh — drive forward and STOP the instant the bumper hits.
#
# Why this exists: `pilot send "forward ..."` is fire-and-forget — it arms a 3s
# watchdog and returns immediately, so checking `bumps` right after reads the
# bumper at t=0 (before the robot has moved) and never sees the wall. This drives
# in a closed loop: it polls the bumper while moving and stops on contact.
#
# Usage:
#   ./scripts/drive_until_bump.sh <cm_per_sec> <max_seconds>
#   e.g. ./scripts/drive_until_bump.sh 40 3
#
# Prints exactly one result line the model reads:
#   BUMP side=<L|R|both> after <s>s (~<cm>cm)   -> decision point, photograph & turn
#   CLEAR drove <s>s (~<cm>cm)                   -> open, drive again
#
# Side effects: updates the heading map via track.sh (fwd <actual cm>, and block on
# bump), and bumps TOOL_CALLS once for the leg.

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PILOT="${ROOT}/roomba_pilot/target/debug/pilot"
TRACK="${ROOT}/scripts/track.sh"
STATE_FILE="/tmp/run_state.env"

SPEED="${1:?usage: drive_until_bump.sh <cm_per_sec> <max_seconds>}"
MAXSEC="${2:?usage: drive_until_bump.sh <cm_per_sec> <max_seconds>}"

POLL=0.15                       # seconds between bumper reads
REARM_EVERY=12                  # re-send `go` every ~1.8s to keep the 3s watchdog alive
iters=$(awk -v m="$MAXSEC" -v p="$POLL" 'BEGIN{printf "%d", m/p}')

start_go() { "$PILOT" send "go ${SPEED} 0" >/dev/null; }

start_go
bumped=""; i=0
while [ "$i" -lt "$iters" ]; do
  b="$("$PILOT" send "bumps" 2>/dev/null || true)"
  l=$(printf '%s' "$b" | grep -o 'bumpL=[01]' | cut -d= -f2)
  r=$(printf '%s' "$b" | grep -o 'bumpR=[01]' | cut -d= -f2)
  if [ "${l:-0}" = "1" ] || [ "${r:-0}" = "1" ]; then
    if [ "${l:-0}" = "1" ] && [ "${r:-0}" = "1" ]; then bumped="both";
    elif [ "${l:-0}" = "1" ]; then bumped="L"; else bumped="R"; fi
    break
  fi
  i=$(( i + 1 ))
  if [ $(( i % REARM_EVERY )) -eq 0 ]; then start_go; fi
  sleep "$POLL"
done
"$PILOT" send "stop" >/dev/null

# Distance actually travelled (cm) = speed * elapsed.
elapsed=$(awk -v i="$i" -v p="$POLL" 'BEGIN{printf "%.1f", i*p}')
cm=$(awk -v s="$SPEED" -v e="$elapsed" 'BEGIN{printf "%d", s*e}')

# Update heading map and run counters.
"$TRACK" fwd "$cm" >/dev/null || true
if [ -f "$STATE_FILE" ]; then
  tc=$(grep '^TOOL_CALLS=' "$STATE_FILE" | cut -d= -f2 || echo 0)
  tmp="${STATE_FILE}.tmp"; grep -v '^TOOL_CALLS=' "$STATE_FILE" > "$tmp" || true
  echo "TOOL_CALLS=$(( ${tc:-0} + 1 ))" >> "$tmp"; mv "$tmp" "$STATE_FILE"
fi

if [ -n "$bumped" ]; then
  "$TRACK" block >/dev/null || true
  echo "BUMP side=${bumped} after ${elapsed}s (~${cm}cm)"
else
  echo "CLEAR drove ${elapsed}s (~${cm}cm)"
fi
