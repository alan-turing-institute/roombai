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

# ── Step 3: SAS token file ────────────────────────────────────────────────────
echo ""
echo "━━━ Step 3/4 — Azure Blob SAS token ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
SAS_TOKEN_FILE="${HOME}/.secrets/roombai_sas"
mkdir -p "${HOME}/.secrets"
if [ -f "$SAS_TOKEN_FILE" ]; then
    echo "  SAS token file already exists at ${SAS_TOKEN_FILE} — skipping"
    echo "  (to update: replace the contents of that file and re-run)"
else
    echo ""
    echo "  Paste your Azure Blob SAS URL below (from the Azure portal)."
    echo "  It should start with: https://<account>.blob.core.windows.net/?sv=..."
    echo ""
    read -rp "  SAS URL: " sas_url
    echo "$sas_url" > "$SAS_TOKEN_FILE"
    chmod 600 "$SAS_TOKEN_FILE"
    echo "✓ SAS token saved to ${SAS_TOKEN_FILE}"
fi
echo ""
echo "  Also set your storage account name in scripts/upload_run.sh (AZURE_ACCOUNT)."

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
