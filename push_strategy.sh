#!/usr/bin/env bash
# push_strategy.sh — Push a strategy override to the Pi during a live run.
#
# The robot picks up the change within the next mover loop cycle (~6 s).
#
# Usage:
#   bash push_strategy.sh APPROACH 80.0 "door spotted left at ~120cm"
#   bash push_strategy.sh EXPLORE  ""   "false positive"   --force
#   bash push_strategy.sh WAIT     ""   "waiting for door to open"
#
# Flags:
#   --force     Set force_override=true. Required to cancel an active APPROACH
#               (e.g. after confirming a door detection was a false positive).
#               Without --force, EXPLORE pushes are silently ignored while
#               APPROACH is active.
#   --restart   Kill and restart explore.py on the Pi after pushing.
#               WARNING: Hailo models reload (~15-30 s before movement).
#
# Raw JSON also accepted:
#   bash push_strategy.sh '{"mode":"APPROACH","door_bearing":80.0,"notes":"..."}'

PI_HOST="hackweek26@10.10.100.185"
PI_PASS="aipi"
REPO="/home/hackweek26/roombai"

ssh_pi() {
    sshpass -p "$PI_PASS" ssh -o StrictHostKeyChecking=no -o PubkeyAuthentication=no "$PI_HOST" "$@"
}

RESTART=0
FORCE=0
ARGS=()
for arg in "$@"; do
    case "$arg" in
        --restart) RESTART=1 ;;
        --force)   FORCE=1 ;;
        *)         ARGS+=("$arg") ;;
    esac
done

if [[ ${#ARGS[@]} -eq 0 ]]; then
    echo "Usage: bash push_strategy.sh MODE [BEARING] [NOTES] [--force] [--restart]"
    echo "       bash push_strategy.sh '{\"mode\":\"APPROACH\",...}' [--force]"
    exit 1
fi

# Raw JSON path (first arg starts with '{')
if [[ "${ARGS[0]}" == {* ]]; then
    JSON="${ARGS[0]}"
    # Inject force_override if requested and not already present
    if [[ $FORCE -eq 1 && "$JSON" != *force_override* ]]; then
        JSON="${JSON%\}},\"force_override\":true}"
    fi
else
    MODE="${ARGS[0]}"
    BEARING="${ARGS[1]:-null}"
    NOTES="${ARGS[2]:-}"
    [[ "$BEARING" == "" ]] && BEARING="null"
    NOTES="${NOTES//\"/\\\"}"
    FORCE_FIELD=""
    [[ $FORCE -eq 1 ]] && FORCE_FIELD=',"force_override":true'
    JSON=$(printf '{"mode":"%s","door_bearing":%s,"notes":"%s"%s}' \
           "$MODE" "$BEARING" "$NOTES" "$FORCE_FIELD")
fi

echo "[push] → $JSON"
ssh_pi "echo '$JSON' > /tmp/roomba_strategy.json"
echo "[push] delivered — robot picks up within ~6 s"
[[ $FORCE -eq 1 ]] && echo "[push] force_override=true — will cancel active APPROACH"

if [[ $RESTART -eq 1 ]]; then
    echo "[push] --restart: stopping explore.py and restarting run…"
    ssh_pi "pkill -f explore.py 2>/dev/null || true"
    sleep 2
    ssh_pi "source ~/yolo_new/bin/activate && \
            cd $REPO && nohup bash run.sh > /tmp/run_output.log 2>&1 &"
    echo "[push] restart launched — tail /tmp/run_output.log on Pi for startup progress"
fi
