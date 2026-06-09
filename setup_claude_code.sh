#!/usr/bin/env bash
# setup_claude_code.sh — Install Claude Code CLI on the Pi and configure it
# to use the ~/yolo_new Python virtual environment.
#
# Safe to run multiple times — skips steps that are already done.
# Non-fatal: if Claude Code cannot be installed the run continues normally.
#
set -uo pipefail

VENV="$HOME/yolo_new"
CLAUDE_SETTINGS="$HOME/.claude/settings.json"

ok()   { echo "  ✓ $*"; }
warn() { echo "  ⚠ $*"; }
info() { echo "  · $*"; }

# ── 1. Install Claude Code ────────────────────────────────────────────────────
if command -v claude &>/dev/null; then
    ok "Claude Code already installed  ($(claude --version 2>/dev/null || echo unknown))"
else
    info "Installing Claude Code (native ARM64 binary)…"
    if curl -fsSL https://claude.ai/install.sh | bash 2>/dev/null; then
        # The installer may put the binary in ~/.claude/local/claude or similar
        # Ensure it's on PATH for this session
        export PATH="$HOME/.claude/local:$PATH"
        if command -v claude &>/dev/null; then
            ok "Claude Code installed  ($(claude --version 2>/dev/null))"
        else
            warn "Installer ran but 'claude' not found on PATH — may need shell restart"
        fi
    else
        warn "Claude Code install failed — run will continue without it"
        warn "To install manually:  curl -fsSL https://claude.ai/install.sh | bash"
        exit 0   # non-fatal
    fi
fi

# ── 2. Configure Claude Code to use ~/yolo_new venv for this project ─────────
# We do NOT touch ~/.claude/settings.json (global) because that would hijack
# the venv for every other project on the Pi.
#
# Instead, write a project-level .claude/settings.json inside the repo that
# is only active when Claude Code is launched from /home/hackweek26/roombai.
# Claude Code merges project settings on top of user settings, so this is
# scoped correctly.

if [[ ! -d "$VENV" ]]; then
    warn "venv not found at $VENV — skipping venv configuration"
    exit 0
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_SETTINGS="$REPO_DIR/.claude/settings.local.json"
VENV_BIN="$VENV/bin"
SYSTEM_PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

mkdir -p "$(dirname "$PROJECT_SETTINGS")"

# Merge with any existing project settings.json; preserve other keys.
python3 - <<PYEOF
import json, pathlib

path = pathlib.Path("$PROJECT_SETTINGS")
data = json.loads(path.read_text()) if path.exists() else {}

data.setdefault("env", {})
data["env"]["VIRTUAL_ENV"] = "$VENV"
data["env"]["PATH"]        = "$VENV_BIN:$SYSTEM_PATH"

path.write_text(json.dumps(data, indent=2))
print(f"  · Updated {path}")
PYEOF

ok "Claude Code configured for this project to use venv at $VENV"
ok "(project-scoped — other projects on the Pi are unaffected)"
info "Start Claude Code from the repo root:  cd $REPO_DIR && claude"
