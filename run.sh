#!/usr/bin/env bash
# run.sh — RoombaI full startup script.  Run this ON the Raspberry Pi.
#
# Usage:  bash run.sh [--greet]
#   --greet   Enable human greeting mode (80 % / 20 % phrase split)
#
# What it does:
#   0. Install model tools + download missing HEFs  (install_models.sh)
#   1. Pre-flight vision-model check   (ensure_models() in model_setup.py)
#   2. Start pilot daemon              (serial ↔ TCP bridge, if not already up)
#   3. Start TTS daemon                (espeak-ng speaker, if not already up)
#   4. Run explore.py                  (blocks until the run ends)
#
set -euo pipefail

# ── Activate Python virtual environment ───────────────────────────────────────
source ~/yolo_new/bin/activate

# ── Resolve repo root (works regardless of cwd) ───────────────────────────────
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PILOT_BIN="$REPO_DIR/roomba_pilot/target/debug/pilot"
SERIAL_PORT="/dev/ttyUSB0"
LOG_FILE="/tmp/roomba_log.txt"
PILOT_LOG="/tmp/pilot.log"
TTS_LOG="/tmp/speak_daemon.log"
SPEAK_QUEUE="/tmp/speak_queue.txt"

# ── Parse arguments ───────────────────────────────────────────────────────────
GREET_FLAG="--greet"
for arg in "$@"; do
    case "$arg" in
        --greet) GREET_FLAG="--greet" ;;
        *) echo "Unknown argument: $arg" >&2; exit 1 ;;
    esac
done

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║            RoombaI  —  pre-flight                ║"
if [[ -n "$GREET_FLAG" ]]; then
echo "║            greeting mode: ON                     ║"
fi
echo "╚══════════════════════════════════════════════════╝"
echo ""

# ── Helper ────────────────────────────────────────────────────────────────────
step() { echo ""; echo "── $* ──────────────────────────────────────────────"; }
ok()   { echo "  ✓ $*"; }
fail() { echo "  ✗ $*" >&2; exit 1; }

# ── Step 0: Clear stale /tmp data from previous runs ─────────────────────────
# Do this unconditionally so leftover frame files never corrupt the new run.
step "Step 0 — Clearing stale /tmp run data"
# Kill the old TTS daemon BEFORE deleting speak_queue.txt — if we delete the
# file while tail -f is following it, the daemon tracks the old inode and
# misses all speak() calls in the new run.
pkill -f speak_queue.txt 2>/dev/null || true
rm -rf /tmp/roomba_frames /tmp/roomba_maps /tmp/roomba_state.json \
       /tmp/roomba_log.txt /tmp/roomba_events.json /tmp/speak_queue.txt \
       /tmp/pilot.log 2>/dev/null || true
ok "stale /tmp data cleared"

# ── Step 0a: Python dependency check + auto-install ──────────────────────────
step "Step 0a — Python dependency check"
if ! python3 "$REPO_DIR/check_deps.py"; then
    fail "Critical Python dependencies missing and could not be installed. See above."
fi

# ── Step 0b: Install tools + download missing HEF files ───────────────────────
step "Step 0b — Model installer (hailo-model-zoo tools + missing HEFs)"
bash "$REPO_DIR/install_models.sh"

# Re-run dependency check after installs in case something was just added
step "Step 0c — Re-check dependencies after model install"
if ! python3 "$REPO_DIR/check_deps.py"; then
    fail "Critical dependencies still missing after install attempt."
fi

# ── Step 1: Vision model check ────────────────────────────────────────────────
step "Step 1 — Vision model check (validate all HEFs via Hailo SDK)"
cd "$REPO_DIR"
python3 - <<'PYEOF'
import sys
from model_setup import ensure_models
results = ensure_models(abort_if_required_missing=True)
ready   = [k for k, v in results.items() if v]
missing = [k for k, v in results.items() if not v]
print(f"  Ready  : {ready}")
if missing:
    print(f"  Missing: {missing}  (optional — OpenCV door detection still runs)")
PYEOF
ok "Model check complete"

# ── Step 2: Pilot daemon ──────────────────────────────────────────────────────
step "Step 2 — Pilot daemon  (serial ↔ TCP bridge)"
if pgrep -x pilot > /dev/null 2>&1; then
    ok "pilot daemon already running  ($(pgrep -x pilot))"
else
    if [[ ! -x "$PILOT_BIN" ]]; then
        echo "  Building pilot binary…"
        cd "$REPO_DIR/roomba_pilot"
        cargo build 2>&1 | tail -5
        cd "$REPO_DIR"
    fi
    if [[ ! -e "$SERIAL_PORT" ]]; then
        fail "$SERIAL_PORT not found — is the Roomba powered on and connected?"
    fi
    nohup "$PILOT_BIN" serve "$SERIAL_PORT" > "$PILOT_LOG" 2>&1 &
    PILOT_PID=$!
    # Give daemon time to start and verify OI mode
    sleep 3
    if ! pgrep -x pilot > /dev/null 2>&1; then
        echo "  pilot log:" >&2
        tail -20 "$PILOT_LOG" >&2
        fail "Pilot daemon failed to start"
    fi
    ok "Pilot daemon started  (PID $PILOT_PID)"
fi

# ── Step 3: TTS daemon ────────────────────────────────────────────────────────
step "Step 3 — TTS daemon  (espeak-ng speaker)"
if pgrep -f "speak_queue.txt" > /dev/null 2>&1; then
    ok "TTS daemon already running"
else
    touch "$SPEAK_QUEUE"
    # shellcheck disable=SC2016
    # -d hw:0,0 targets the Jabra USB speaker (card 0).
    # Without this espeak-ng defaults to HDMI and produces no audible output.
    nohup bash -c \
        'tail -n 0 -f "$1" | while IFS= read -r line; do
             espeak-ng -s 145 -d hw:0,0 -- "$line" 2>/dev/null
         done' _ "$SPEAK_QUEUE" \
        > "$TTS_LOG" 2>&1 &
    ok "TTS daemon started"
fi

# ── Step 4: Run explorer ──────────────────────────────────────────────────────
step "Step 4 — Starting explore.py ${GREET_FLAG:-(no greet)}"
echo ""
echo "  Log:  tail -f $LOG_FILE"
echo "  State: cat /tmp/roomba_state.json"
echo "  Frames: ls /tmp/roomba_frames/"
echo ""
echo "  Press Ctrl-C to stop the run."
echo "  (The pilot daemon keeps running; kill it with:  pkill pilot)"
echo ""

cd "$REPO_DIR"
# explore.py runs model check again internally — any models that loaded above
# will already be in MODEL_SPECS so init_all_models() is effectively instant.
# 10-minute hard time limit: timeout sends SIGTERM, then SIGKILL after 5 s.
exec timeout --kill-after=5 600 python3 explore.py $GREET_FLAG
