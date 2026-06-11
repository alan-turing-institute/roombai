#!/usr/bin/env bash
# supervise.sh — Live supervision driver.
#
# Starts live_sync.sh in the background (Pi→Mac every 1s) and then polls
# roomba_state.json every 2 s.  Emits one structured line whenever:
#   - robot mode changes (EXPLORE ↔ APPROACH ↔ WAIT)
#   - door detection status changes (False → True or True → False)
#   - robot reports stuck=True
#   - every 30 s as a heartbeat (for periodic image review)
#
# Each emitted line is a Monitor event that wakes Claude for analysis.
# Claude reads the current frame + state and posts a suggestion.

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIVE_DIR="$REPO/runs/live"
mkdir -p "$LIVE_DIR"

# ── Start file sync in background ─────────────────────────────────────────────
bash "$REPO/live_sync.sh" >/dev/null 2>&1 &
SYNC_PID=$!
trap "kill $SYNC_PID 2>/dev/null; exit 0" EXIT INT TERM

echo "SUPERVISE started — sync PID=$SYNC_PID — waiting for run data…"

prev_mode="?"
prev_door="False"
n=0

# ── Poll loop ─────────────────────────────────────────────────────────────────
while true; do
    sleep 2
    n=$(( n + 1 ))

    if [[ ! -f "$LIVE_DIR/roomba_state.json" ]]; then
        # Run not started yet — heartbeat every 30 s so Claude knows we're waiting
        (( n % 15 == 0 )) && echo "WAITING: no run data yet (start run on Pi first)"
        continue
    fi

    # Parse state.json into pipe-delimited fields
    state_line=$(python3 -c "
import json, sys
try:
    d = json.load(open('$LIVE_DIR/roomba_state.json'))
    door = d.get('last_door') or {}
    yd = (d.get('last_detection') or [])
    person = any(x.get('class_name') == 'person' for x in yd)
    print('|'.join([
        d.get('mode', '?'),
        str(d.get('frames_captured', 0)),
        str(door.get('door_visible', False)),
        str(door.get('door_open', False)),
        str(door.get('door_position') or 'N/A'),
        str(door.get('door_distance_cm') or 'N/A'),
        str(round(door.get('confidence', 0), 2)),
        str(d.get('stuck', False)),
        str(d.get('bumps', 0)),
        str(round(d.get('heading', 0))),
        str(round(d.get('pos_x', 0))),
        str(round(d.get('pos_y', 0))),
        str(d.get('nearest_cm') or 'N/A'),
        str(person),
    ]))
except Exception as e:
    print('ERR|0|False|False|N/A|N/A|0|False|0|0|0|0|N/A|False')
" 2>/dev/null)

    IFS='|' read -r MODE FRAME DOOR DOOR_OPEN DOOR_POS DOOR_DIST DOOR_CONF \
                      STUCK BUMPS HDG PX PY NEAR PERSON <<< "$state_line"

    # Decide whether to emit (Claude wakes up and analyses image + state)
    emit=0
    reason=""
    [[ "$MODE"   != "$prev_mode" ]] && emit=1 && reason="mode→$MODE"
    [[ "$DOOR"   != "$prev_door" ]] && emit=1 && reason="$reason door→$DOOR"
    [[ "$STUCK"  == "True"       ]] && emit=1 && reason="$reason STUCK"
    [[ "$PERSON" == "True" && "$prev_door" != "True" ]] && emit=1 && reason="$reason PERSON"
    (( n % 15 == 0 ))              && emit=1 && reason="heartbeat"

    if [[ $emit -eq 1 ]]; then
        echo "ANALYSE reason=${reason# } mode=$MODE frame=$FRAME door=$DOOR(open=$DOOR_OPEN,${DOOR_POS},${DOOR_DIST}cm,conf=$DOOR_CONF) stuck=$STUCK bumps=$BUMPS hdg=${HDG}deg pos=(${PX},${PY})cm near=${NEAR}cm person=$PERSON"
        prev_mode="$MODE"
        prev_door="$DOOR"
    fi
done
