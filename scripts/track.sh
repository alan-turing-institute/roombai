#!/usr/bin/env bash
# track.sh — lightweight heading + coverage memory for the escape run.
#
# No odometry is available, so we dead-reckon ONLY heading (in-place turns are
# reliable) plus a coarse per-direction "how much have I driven this way" tally.
# This is the robot's map: 8 compass sectors (45 deg each) relative to start.
#
# Usage (cheap, call inline after the matching pilot command):
#   ./scripts/track.sh fwd <cm>     # drove <cm> on current heading -> add to coverage
#   ./scripts/track.sh turn <deg>   # rotated <deg> (+CCW / -CW) -> update heading
#   ./scripts/track.sh block        # bumped: mark current heading sector blocked
#   ./scripts/track.sh suggest      # print the turn (deg) toward the least-explored open sector
#
# State lives in /tmp/run_state.env: HEADING, COV0..COV7, BLOCKED.

set -euo pipefail
STATE_FILE="/tmp/run_state.env"
[ -f "$STATE_FILE" ] || { echo "ERR no state file"; exit 1; }
source "$STATE_FILE"

# Defaults for keys that may not exist yet.
HEADING="${HEADING:-0}"
for i in 0 1 2 3 4 5 6 7; do eval "COV$i=\${COV$i:-0}"; done
BLOCKED="${BLOCKED:-}"

set_key() {  # set_key NAME VALUE  (replace if present, else append; portable, no sed -i)
  local name="$1" val="$2" tmp="${STATE_FILE}.tmp"
  grep -v "^${name}=" "$STATE_FILE" > "$tmp" || true
  echo "${name}=${val}" >> "$tmp"
  mv "$tmp" "$STATE_FILE"
}

sector() {  # echo sector index 0..7 for a heading
  awk -v h="$1" 'BEGIN{ h=(h%360+360)%360; print int((h+22.5)/45)%8 }'
}

cmd="${1:-}"
case "$cmd" in
  fwd)
    cm="${2:?usage: fwd <cm>}"
    s=$(sector "$HEADING")
    cur=$(eval "echo \$COV$s")
    new=$(awk -v a="$cur" -v b="$cm" 'BEGIN{print a+b}')
    set_key "COV$s" "$new"
    echo "OK cov[sector $s @ ${HEADING}deg] += ${cm} -> ${new}"
    ;;
  turn)
    deg="${2:?usage: turn <deg>}"
    new=$(awk -v h="$HEADING" -v d="$deg" 'BEGIN{print ((h+d)%360+360)%360}')
    set_key "HEADING" "$new"
    echo "OK heading ${HEADING} -> ${new}"
    ;;
  block)
    s=$(sector "$HEADING")
    case " $BLOCKED " in *" $s "*) : ;; *) BLOCKED="${BLOCKED} $s"; set_key "BLOCKED" "\"${BLOCKED# }\"";; esac
    echo "OK sector $s (@${HEADING}deg) marked blocked: [${BLOCKED# }]"
    ;;
  suggest)
    # Pick the least-covered, non-blocked sector; print the signed shortest turn to face it.
    best_s=-1; best_cov=-1
    for i in 0 1 2 3 4 5 6 7; do
      case " $BLOCKED " in *" $i "*) continue;; esac
      c=$(eval "echo \$COV$i")
      if [ "$best_s" -eq -1 ] || awk -v a="$c" -v b="$best_cov" 'BEGIN{exit !(a<b)}'; then
        best_s=$i; best_cov=$c
      fi
    done
    if [ "$best_s" -eq -1 ]; then echo "TURN 180  # all sectors blocked, reverse"; exit 0; fi
    target=$(( best_s * 45 ))
    delta=$(awk -v t="$target" -v h="$HEADING" 'BEGIN{d=((t-h)%360+540)%360-180; print d}')
    echo "TURN ${delta}  # toward least-explored sector ${best_s} (${target}deg), only ${best_cov}cm covered"
    ;;
  *)
    echo "usage: track.sh {fwd <cm>|turn <deg>|block|suggest}"; exit 1;;
esac
