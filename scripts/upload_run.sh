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

# Configure the rclone remote via environment variables rather than an inline
# connection string. The SAS token contains ':' in its se/st timestamps
# (e.g. se=2029-12-31T22:36:03Z), and rclone's connection-string syntax splits
# on ':', which mangles the URL ("container name in SAS URL ... do not match").
# Env-var remote config sidesteps that parsing entirely.
export RCLONE_CONFIG_ROOMBAI_TYPE="azureblob"
export RCLONE_CONFIG_ROOMBAI_ACCOUNT="${AZURE_ACCOUNT}"
export RCLONE_CONFIG_ROOMBAI_SAS_URL="https://${AZURE_ACCOUNT}.blob.core.windows.net/?${SAS_TOKEN}"

ATTEMPT_NAME=$(basename "$ATTEMPT_DIR")

echo "Uploading ${ATTEMPT_NAME} → ${CONTAINER}/${ATTEMPT_NAME} ..."
# NOTE: the SAS token must include READ permission (r) in addition to write/
# list/create — rclone HEADs the destination on init, which requires read.
rclone copy "$ATTEMPT_DIR" "ROOMBAI:${CONTAINER}/${ATTEMPT_NAME}" --progress

echo "Done."
