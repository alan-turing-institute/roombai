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

# ── Step 3: Azure credentials ─────────────────────────────────────────────────
echo ""
echo "━━━ Step 3/4 — Azure Blob credentials ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
mkdir -p "${HOME}/.secrets"
chmod 700 "${HOME}/.secrets"

ACCOUNT_FILE="${HOME}/.secrets/roombai_account"
SAS_FILE="${HOME}/.secrets/roombai_sas"

if [ -f "$ACCOUNT_FILE" ]; then
    echo "  Account name already saved at ${ACCOUNT_FILE} — skipping"
    echo "  (to update: overwrite the file and re-run)"
else
    echo ""
    read -rp "  Storage account name (e.g. myaccount): " azure_account
    echo "$azure_account" > "$ACCOUNT_FILE"
    chmod 600 "$ACCOUNT_FILE"
    echo "✓ Account name saved to ${ACCOUNT_FILE}"
fi

if [ -f "$SAS_FILE" ]; then
    echo "  SAS token already saved at ${SAS_FILE} — skipping"
    echo "  (to update: overwrite the file and re-run)"
else
    echo ""
    echo "  Paste the SAS token query string from the Azure portal."
    echo "  This is the part AFTER the '?' — e.g: sv=2026-02-06&ss=b&srt=co&sp=wlctfx&..."
    echo "  (do NOT include the leading '?' or the full URL)"
    echo ""
    read -rp "  SAS token: " sas_token
    # Strip leading ? if accidentally included
    sas_token="${sas_token#\?}"
    echo "$sas_token" > "$SAS_FILE"
    chmod 600 "$SAS_FILE"
    echo "✓ SAS token saved to ${SAS_FILE}"
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
