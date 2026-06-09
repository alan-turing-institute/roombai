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

# ── 2. Venv integration ───────────────────────────────────────────────────────
# Claude Code inherits the shell environment, so no settings file needs to be
# touched. Each user simply activates their own venv before launching claude:
#
#   source ~/my_venv/bin/activate && claude
#
# run.sh already activates ~/yolo_new at the top, so Claude Code launched from
# within that script automatically gets the right Python. Writing venv paths
# into any settings.json (global or project-level) would interfere with other
# users who share this repo directory and use different venvs.

ok "Claude Code uses whichever venv is active in the shell — no config needed"
info "To use with yolo_new:  source ~/yolo_new/bin/activate && claude"
info "(run.sh activates ~/yolo_new automatically, so Claude Code launched"
info " from within a run already has the correct Python environment)"
