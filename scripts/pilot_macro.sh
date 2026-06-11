#!/usr/bin/env bash
# Send a short sequence of pilot commands, count them, and wait for timed
# motion commands to complete before sending the next command.
#
# Usage:
#   ./scripts/pilot_macro.sh "turn 30" "move 220" "turn -8" "move 80"

set -euo pipefail

if [ "$#" -lt 1 ]; then
    echo "usage: $0 \"<pilot command>\" [\"<pilot command>\" ...]" >&2
    exit 2
fi

REPO_ROOT="$(git rev-parse --show-toplevel)"
SEND="${REPO_ROOT}/scripts/pilot_send.sh"

sleep_for_command() {
    local cmd="$1"
    # shellcheck disable=SC2206
    local parts=($cmd)
    local op="${parts[0]:-}"

    case "$op" in
        move)
            [ "${#parts[@]}" -eq 2 ] || return 0
            awk -v cm="${parts[1]}" 'BEGIN { if (cm < 0) cm = -cm; printf "%.2f\n", (cm / 20.0) + 0.20 }'
            ;;
        turn)
            [ "${#parts[@]}" -eq 2 ] || return 0
            awk -v deg="${parts[1]}" 'BEGIN { if (deg < 0) deg = -deg; printf "%.2f\n", (deg / 60.0) + 0.20 }'
            ;;
        forward|back|spin)
            [ "${#parts[@]}" -eq 3 ] || return 0
            awk -v sec="${parts[2]}" 'BEGIN { printf "%.2f\n", sec + 0.20 }'
            ;;
        *)
            return 0
            ;;
    esac
}

for cmd in "$@"; do
    "$SEND" "$cmd"
    delay="$(sleep_for_command "$cmd" || true)"
    if [ -n "${delay:-}" ]; then
        sleep "$delay"
    fi
done
