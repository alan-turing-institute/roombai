#!/usr/bin/env bash
# upload_run.sh — upload a completed escape attempt directory to Azure Blob Storage.
#
# Usage:
#   ./upload_run.sh <attempt_dir>
#   ./upload_run.sh /tmp/escape_attempt_3
#
# ── One-time setup ────────────────────────────────────────────────────────────
# 1. Generate a SAS token in the Azure portal (Blob + Container + Object,
#    Write + List + Create permissions) and save it to a file:
#      mkdir -p ~/.secrets
#      echo "https://youraccount.blob.core.windows.net/?sv=..." > ~/.secrets/roombai_sas
#      chmod 600 ~/.secrets/roombai_sas
#
# 2. Set the storage account name below (AZURE_ACCOUNT).
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

AZURE_ACCOUNT="<your-storage-account-name>"
SAS_TOKEN_FILE="${HOME}/.secrets/roombai_sas"
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

if [ ! -f "$SAS_TOKEN_FILE" ]; then
    echo "Error: SAS token file not found at ${SAS_TOKEN_FILE}"
    echo "  Generate a SAS token in the Azure portal and save it:"
    echo "    mkdir -p ~/.secrets"
    echo "    echo 'https://...' > ${SAS_TOKEN_FILE}"
    echo "    chmod 600 ${SAS_TOKEN_FILE}"
    exit 1
fi

SAS_TOKEN=$(cat "$SAS_TOKEN_FILE" | tr -d '[:space:]')
ATTEMPT_NAME=$(basename "$ATTEMPT_DIR")
REMOTE=":azureblob,account=${AZURE_ACCOUNT},sas_url=${SAS_TOKEN}:${CONTAINER}"

echo "Uploading ${ATTEMPT_NAME} → ${CONTAINER}/${ATTEMPT_NAME} ..."
rclone copy "$ATTEMPT_DIR" "${REMOTE}/${ATTEMPT_NAME}" --progress

echo "Done."
