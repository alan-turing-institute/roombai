#!/usr/bin/env bash
# push_strategy.sh — Push a strategy override to the Pi during a live run.
#
# The robot picks up the change within the next mover loop cycle (~6 s).
# No run restart is needed for mode/bearing changes.
#
# Usage:
#   bash push_strategy.sh APPROACH 80.0 "door spotted left at ~120cm"
#   bash push_strategy.sh EXPLORE  ""   "returning to exploration"
#   bash push_strategy.sh WAIT     ""   "waiting for door to open"
#
# Or with raw JSON:
#   bash push_strategy.sh '{"mode":"APPROACH","door_bearing":80.0,"notes":"..."}'
#
# Optional --restart flag: kills and restarts explore.py on the Pi after pushing.
# Use this when the robot is in a bad state and needs a clean start.
# WARNING: Hailo models reload on restart (~15-30 s before movement begins).
#
#   bash push_strategy.sh EXPLORE "" "fresh start" --restart

PI_HOST="hackweek26@10.10.100.185"
PI_PASS="aipi"
REPO="/home/hackweek26/roombai"

ssh_pi() {
    sshpass -p "$PI_PASS" ssh -o StrictHostKeyChecking=no "$PI_HOST" "$@"
}

RESTART=0
ARGS=()
for arg in "$@"; do
    if [[ "$arg" == "--restart" ]]; then
        RESTART=1
    else
        ARGS+=("$arg")
    fi
done

if [[ ${#ARGS[@]} -eq 0 ]]; then
    echo "Usage: bash push_strategy.sh MODE [BEARING] [NOTES] [--restart]"
    echo "       bash push_strategy.sh '{\"mode\":\"APPROACH\",...}' [--restart]"
    exit 1
fi

# Raw JSON path (first arg starts with '{')
if [[ "${ARGS[0]}" == {* ]]; then
    JSON="${ARGS[0]}"
else
    MODE="${ARGS[0]}"
    BEARING="${ARGS[1]:-null}"
    NOTES="${ARGS[2]:-}"
    [[ "$BEARING" == "" ]] && BEARING="null"
    # Escape any double-quotes in notes
    NOTES="${NOTES//\"/\\\"}"
    JSON=$(printf '{"mode":"%s","door_bearing":%s,"notes":"%s"}' "$MODE" "$BEARING" "$NOTES")
fi

echo "[push] → $JSON"
ssh_pi "echo '$JSON' > /tmp/roomba_strategy.json"
echo "[push] delivered — robot picks up within ~6 s"

if [[ $RESTART -eq 1 ]]; then
    echo "[push] --restart: stopping explore.py and restarting run…"
    # Kill any running explore.py; pilot daemon keeps running.
    ssh_pi "pkill -f explore.py 2>/dev/null || true"
    sleep 2
    # Launch a fresh run in the background (run.sh activates venv, checks models, starts explore.py).
    ssh_pi "source $REPO/venv/bin/activate 2>/dev/null; source ~/yolo_new/bin/activate; \
            cd $REPO && nohup bash run.sh > /tmp/run_output.log 2>&1 &"
    echo "[push] restart launched — tail /tmp/run_output.log on Pi for startup progress"
fi
