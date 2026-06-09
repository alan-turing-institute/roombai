#!/usr/bin/env bash
# install_models.sh — Install Hailo model-zoo tools and download missing HEF files.
#
# Run automatically by run.sh before every exploration run.
# Safe to run multiple times — skips anything already present.
#
# Models managed:
#   yolo_seg  yolov8n_seg              instance segmentation (masks per object)
#   midas     midas_v2_1_small         monocular depth map (single frame)
#   fast_scnn fast_scnn                fast semantic segmentation (floor/obstacles)
#   deeplab   deeplabv3_plus_mobilenet_v2  high-quality semantic segmentation
#
# The required yolo_det model (yolov8s_h8l.hef) ships with the Hailo SDK and
# is not managed here.
#
set -uo pipefail   # no -e so individual failures don't abort the whole script

MODEL_DIR="/usr/share/hailo-models"
HW_ARCH="hailo8l"

# ── Helpers ───────────────────────────────────────────────────────────────────
ok()   { echo "  ✓ $*"; }
warn() { echo "  ⚠ $*"; }
info() { echo "  · $*"; }

MIN_HEF_BYTES=500000   # valid HEFs are several MB; reject tiny/empty files

hef_ok() {
    local f="$1"
    [[ -f "$f" ]] && [[ $(stat -c%s "$f" 2>/dev/null || echo 0) -gt $MIN_HEF_BYTES ]]
}

# Try to install a pip package, handling both old and new pip environments.
pip_install() {
    local pkg="$1"
    pip3 install "$pkg" -q --break-system-packages 2>/dev/null \
    || pip3 install "$pkg" -q 2>/dev/null \
    || { warn "pip install $pkg failed (may already be available)"; return 0; }
}

# ── Step 1: Python tooling ────────────────────────────────────────────────────
echo ""
echo "── install_models: Python tooling ──────────────────────────────────────"

if python3 -c "import hailo_model_zoo" 2>/dev/null; then
    ok "hailo-model-zoo already installed"
else
    info "Installing hailo-model-zoo…"
    pip_install hailo-model-zoo
    if python3 -c "import hailo_model_zoo" 2>/dev/null; then
        ok "hailo-model-zoo installed"
    else
        warn "hailo-model-zoo install failed — downloads will rely on ONNX fallback"
    fi
fi

if python3 -c "import ultralytics" 2>/dev/null; then
    ok "ultralytics already installed"
else
    info "Installing ultralytics (needed for ONNX export fallback)…"
    pip_install ultralytics
    python3 -c "import ultralytics" 2>/dev/null \
        && ok "ultralytics installed" \
        || warn "ultralytics install failed — ONNX fallback unavailable"
fi

mkdir -p "$MODEL_DIR"

# ── Step 2: Download missing HEF files ───────────────────────────────────────
echo ""
echo "── install_models: HEF downloads ───────────────────────────────────────"

# Format: "internal_key|zoo_name|target_hef_filename"
MODELS=(
    "yolo_seg|yolov8n_seg|yolov8n_seg_h8l.hef"
    "midas|midas_v2_1_small|midas_v2_1_small_h8l.hef"
    "fast_scnn|fast_scnn|fast_scnn_h8l.hef"
    "deeplab|deeplabv3_plus_mobilenet_v2|deeplabv3_plus_mobilenetv2_cityscapes_h8l.hef"
)

for entry in "${MODELS[@]}"; do
    IFS='|' read -r key zoo_name hef_file <<< "$entry"
    target="$MODEL_DIR/$hef_file"

    if hef_ok "$target"; then
        ok "$key: $hef_file  ($(( $(stat -c%s "$target") / 1024 / 1024 )) MB)"
        continue
    fi

    info "$key: downloading $zoo_name (arch=$HW_ARCH)…"
    downloaded=0

    # Method 1: hailomz CLI (installed by hailo-model-zoo pip package)
    for arch in "$HW_ARCH" hailo8; do
        if command -v hailomz &>/dev/null; then
            if hailomz download "$zoo_name" --hw-arch "$arch" \
                    --output-dir "$MODEL_DIR" 2>/dev/null; then
                downloaded=1; break
            fi
        fi
    done

    # Method 2: hailo_model_zoo Python module
    if [[ $downloaded -eq 0 ]]; then
        for arch in "$HW_ARCH" hailo8; do
            if python3 -m hailo_model_zoo.main download "$zoo_name" \
                    --hw-arch "$arch" --output-dir "$MODEL_DIR" 2>/dev/null; then
                downloaded=1; break
            fi
        done
    fi

    # After any successful download, find the file and rename to our expected name
    if [[ $downloaded -eq 1 ]]; then
        # Search for any .hef containing the zoo name (download may add suffixes)
        found=$(find "$MODEL_DIR" -maxdepth 2 -name "*${zoo_name}*.hef" \
                    ! -name "$hef_file" 2>/dev/null | head -1)
        if [[ -n "$found" ]]; then
            mv "$found" "$target"
            info "renamed $(basename "$found") → $hef_file"
        fi
    fi

    # Verify result
    if hef_ok "$target"; then
        ok "$key: downloaded  ($(( $(stat -c%s "$target") / 1024 / 1024 )) MB)"
    else
        warn "$key: could not download $zoo_name automatically."
        echo "       Manual steps:"
        echo "         hailomz download $zoo_name --hw-arch $HW_ARCH \\"
        echo "             --output-dir $MODEL_DIR"
        echo "       Or place the HEF manually at:"
        echo "         $target"
    fi
done

# ── Step 3: Summary ───────────────────────────────────────────────────────────
echo ""
echo "── install_models: summary ──────────────────────────────────────────────"
all_present=1
for entry in "${MODELS[@]}"; do
    IFS='|' read -r key zoo_name hef_file <<< "$entry"
    target="$MODEL_DIR/$hef_file"
    if hef_ok "$target"; then
        ok "$key"
    else
        warn "$key  MISSING  →  $target"
        all_present=0
    fi
done

echo ""
if [[ $all_present -eq 1 ]]; then
    echo "  All optional models present. Full vision pipeline available."
else
    echo "  Some models missing. Run will continue with available models."
    echo "  OpenCV door detection always runs regardless."
fi
echo ""
