#!/usr/bin/env bash
# upload_run.sh — upload a completed escape attempt directory to Azure Blob Storage.
#
# Usage:
#   ./upload_run.sh <attempt_dir>
#   ./upload_run.sh /tmp/escape_attempt_3
#
# Requires rclone configured with a remote named "roombai".
#
# ── One-time setup ────────────────────────────────────────────────────────────
# 1. Install rclone:
#      sudo apt install rclone -y
#
# 2. Configure the Azure Blob remote:
#      rclone config
#      > n  (new remote)
#      > name: roombai
#      > storage: azureblob
#      > account: <your storage account name>
#      > key: <your storage account access key>
#        (or use a SAS token instead of a key)
#
# 3. Create the container in Azure (if it doesn't exist):
#      rclone mkdir roombai:attempts
#
# 4. Verify:
#      rclone lsd roombai:attempts
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

REMOTE="roombai:attempts"

if [ $# -ne 1 ]; then
    echo "Usage: $0 <attempt_dir>"
    exit 1
fi

ATTEMPT_DIR="${1%/}"  # strip trailing slash if present

if [ ! -d "$ATTEMPT_DIR" ]; then
    echo "Error: directory '$ATTEMPT_DIR' does not exist"
    exit 1
fi

if ! command -v rclone &>/dev/null; then
    echo "Error: rclone not found. Install with: sudo apt install rclone -y"
    exit 1
fi

ATTEMPT_NAME=$(basename "$ATTEMPT_DIR")
echo "Uploading ${ATTEMPT_NAME} → ${REMOTE}/${ATTEMPT_NAME} ..."
rclone copy "$ATTEMPT_DIR" "${REMOTE}/${ATTEMPT_NAME}" --progress

echo "Done."
