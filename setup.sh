#!/usr/bin/env bash
# setup.sh — one-time setup for RoombaI on a fresh Raspberry Pi.
#
# Run once after cloning the repo:
#   ./setup.sh
#
# Safe to re-run — all steps are idempotent.

set -euo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  RoombaI — one-time setup"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# ── Step 1: system dependencies ──────────────────────────────────────────────
echo ""
echo "━━━ Step 1/4 — Installing system dependencies ━━━━━━━━━━━━━━━━━━━━━━━━━━"
sudo apt-get update -q
sudo apt-get install -y \
    rclone \
    ffmpeg \
    imagemagick \
    espeak-ng
echo "✓ Dependencies installed"

# ── Step 2: Rust + pilot binary ───────────────────────────────────────────────
echo ""
echo "━━━ Step 2/4 — Building pilot binary ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if ! command -v cargo &>/dev/null; then
    echo "Rust not found — installing via rustup..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source "$HOME/.cargo/env"
fi
cd "${REPO_ROOT}/roomba_pilot"
cargo build
echo "✓ Pilot binary built at roomba_pilot/target/debug/pilot"
cd "${REPO_ROOT}"

# ── Step 3: rclone Azure Blob config ─────────────────────────────────────────
echo ""
echo "━━━ Step 3/4 — Configuring rclone (Azure Blob Storage) ━━━━━━━━━━━━━━━━━"
if rclone listremotes | grep -q "^roombai:"; then
    echo "  rclone remote 'roombai' already configured — skipping"
    echo "  (to reconfigure: rclone config, delete 'roombai', then re-run this script)"
else
    echo ""
    echo "  You will need:"
    echo "    - Azure Storage Account name"
    echo "    - Azure Storage Account access key (or SAS token)"
    echo "    - Container name (e.g. 'attempts')"
    echo ""
    rclone config
    # Create the container if it doesn't exist
    if rclone listremotes | grep -q "^roombai:"; then
        rclone mkdir roombai:attempts 2>/dev/null || true
        echo "✓ rclone remote 'roombai' configured"
    else
        echo "  ⚠ rclone remote 'roombai' not found after config — uploads will not work."
        echo "    Re-run ./setup.sh or run 'rclone config' manually."
    fi
fi

# ── Step 4: verify ────────────────────────────────────────────────────────────
echo ""
echo "━━━ Step 4/4 — Verifying installation ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
ok=true
for cmd in rclone ffmpeg convert espeak-ng cargo; do
    if command -v "$cmd" &>/dev/null; then
        echo "  ✓ $cmd"
    else
        echo "  ✗ $cmd — not found"
        ok=false
    fi
done
if [ -f "${REPO_ROOT}/roomba_pilot/target/debug/pilot" ]; then
    echo "  ✓ pilot binary"
else
    echo "  ✗ pilot binary — not found (build may have failed)"
    ok=false
fi

echo ""
if $ok; then
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "✓ Setup complete. Run ./run.sh to start a competition attempt."
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
else
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "⚠ Setup finished with errors. Fix the above before running ./run.sh"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    exit 1
fi
