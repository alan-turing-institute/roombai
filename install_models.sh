#!/usr/bin/env bash
# install_models.sh — Install Hailo model-zoo tools and download missing HEF files.
# Safe to run multiple times — skips anything already present.
set -uo pipefail

source ~/yolo_new/bin/activate

MODEL_DIR="/usr/share/hailo-models"

ok()   { echo "  ✓ $*"; }
warn() { echo "  ⚠ $*"; }
info() { echo "  · $*"; }

pip_install() {
    local pkg="$1"
    pip3 install "$pkg" -q --break-system-packages 2>/dev/null \
    || pip3 install "$pkg" -q 2>/dev/null \
    || { warn "pip install $pkg failed (may already be available)"; return 0; }
}

echo ""
echo "── install_models: Python tooling ──────────────────────────────────────"

if python3 -c "import numba" 2>/dev/null; then
    ok "numba already available"
else
    info "Installing numba via apt…"
    DEBIAN_FRONTEND=noninteractive sudo -n apt-get install -y python3-numba 2>/dev/null \
    && ok "numba installed via apt" \
    || warn "numba install failed"
fi

HAILO_ZOO_LOCAL="$HOME/hailo-model-zoo"
if python3 -c "import hailo_model_zoo" 2>/dev/null; then
    ok "hailo-model-zoo already installed"
elif [[ -d "$HAILO_ZOO_LOCAL" ]]; then
    info "Installing hailo-model-zoo from local repo…"
    pip_install "$HAILO_ZOO_LOCAL"
    python3 -c "import hailo_model_zoo" 2>/dev/null && ok "hailo-model-zoo installed" || warn "failed"
else
    pip_install hailo-model-zoo
    python3 -c "import hailo_model_zoo" 2>/dev/null && ok "hailo-model-zoo installed" || warn "failed"
fi

if python3 -c "import ultralytics" 2>/dev/null; then
    ok "ultralytics already available"
else
    info "Installing ultralytics…"
    pip_install ultralytics
    python3 -c "import ultralytics" 2>/dev/null && ok "ultralytics installed" || warn "failed"
fi

mkdir -p "$MODEL_DIR"

echo ""
echo "── install_models: HEF downloads (none — yolo_det ships with SDK) ──────"
echo ""
