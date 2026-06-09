#!/usr/bin/env bash
# upload_run.sh — upload a completed escape attempt directory to Azure Blob Storage.
#
# Usage:
#   ./upload_run.sh <attempt_dir>
#
# ── One-time setup ────────────────────────────────────────────────────────────
# Run ./setup.sh which will prompt for and save:
#   ~/.secrets/roombai_account  — storage account name (e.g. myaccount)
#   ~/.secrets/roombai_sas      — SAS token query string only (the part after the ?)
#                                 e.g. sv=2026-02-06&ss=b&srt=co&sp=wlctfx&...
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

ACCOUNT_FILE="${HOME}/.secrets/roombai_account"
SAS_FILE="${HOME}/.secrets/roombai_sas"
CONTAINER="attempts"

if [ $# -ne 1 ]; then
    echo "Usage: $0 <attempt_dir>"
    exit 1
fi

ATTEMPT_DIR="${1%/}"

if [ ! -d "$ATTEMPT_DIR" ]; then
    echo "Error: directory '$ATTEMPT_DIR' does not exist"
    exit 1
fi

if ! command -v rclone &>/dev/null; then
    echo "Error: rclone not found. Install with: sudo apt install rclone -y"
    exit 1
fi

for f in "$ACCOUNT_FILE" "$SAS_FILE"; do
    if [ ! -f "$f" ]; then
        echo "Error: missing secrets file: ${f}"
        echo "  Re-run ./setup.sh to configure Azure credentials."
        exit 1
    fi
done

AZURE_ACCOUNT=$(cat "$ACCOUNT_FILE" | tr -d '[:space:]')
SAS_TOKEN=$(cat "$SAS_FILE" | tr -d '[:space:]')

# Strip leading ? if someone accidentally included it
SAS_TOKEN="${SAS_TOKEN#\?}"

# Build the full SAS URL rclone expects
SAS_URL="https://${AZURE_ACCOUNT}.blob.core.windows.net/?${SAS_TOKEN}"

ATTEMPT_NAME=$(basename "$ATTEMPT_DIR")
REMOTE=":azureblob,account=${AZURE_ACCOUNT},sas_url=${SAS_URL}:${CONTAINER}"

echo "Uploading ${ATTEMPT_NAME} → ${CONTAINER}/${ATTEMPT_NAME} ..."
rclone copy "$ATTEMPT_DIR" "${REMOTE}/${ATTEMPT_NAME}" --progress

echo "Done."
