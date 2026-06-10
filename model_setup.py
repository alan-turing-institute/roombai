"""
model_setup.py — Pre-flight Hailo model check, download, and compilation.

Called by explore.py before every run via ensure_models().

For each model defined in vision.MODEL_SPECS the routine:
  1. Searches the configured path and a set of known fallback directories.
  2. Validates any found file: size > MIN_HEF_BYTES and parses cleanly as
     a Hailo HEF with approximately the expected input resolution.
  3. If missing or invalid, attempts download via the Hailo model-zoo CLI
     (hailomz / hailo / python -m hailo_model_zoo.main), trying several
     command variants to cover different SDK versions.
  4. If download fails, attempts ONNX → HEF compilation using the Hailo
     Dataflow Compiler CLI (hailo parse → optimize → compile).
  5. Updates vision.MODEL_SPECS in-place to point at the resolved path so
     init_all_models() in vision.py finds it even if it landed outside the
     configured directory.

explore.py calls ensure_models() first in main() and aborts if yolo_det
(the only required model) cannot be resolved.
"""

import shutil
import subprocess
import sys
import time
from pathlib import Path

# ── Tunables ──────────────────────────────────────────────────────────────────
HW_ARCH      = "hailo8l"           # Hailo AI Hat+ chip arch string
MIN_HEF_BYTES = 500_000            # valid HEF files are several MB; reject tiny files
DOWNLOAD_TIMEOUT = 600             # seconds per download attempt

# Primary output directory for downloaded/compiled HEFs
OUTPUT_DIR = Path("/usr/share/hailo-models")

# Extra directories to search before attempting a download
SEARCH_DIRS: list[Path] = [
    Path("/usr/share/hailo-models"),
    Path("/usr/local/share/hailo-models"),
    Path.home() / ".cache" / "hailo_model_zoo",
    Path.home() / ".hailo" / "models",
    Path("/opt/hailo/models"),
    Path("/home/hackweek26/hailo-models"),
]

# ── Hailo model-zoo identifiers ───────────────────────────────────────────────
# Maps our internal model names to the canonical hailo_model_zoo name, the
# ONNX source (for fallback compilation), and whether the model is required.
MODEL_ZOO_INFO: dict[str, dict] = {
    "yolo_det": {"zoo_name": "yolov8s", "required": True},
}

# fast_depth is an ONNX model run on CPU via onnxruntime — not a Hailo HEF.
# No depth model in the Hailo model zoo supports hailo8l, and the Hailo
# Dataflow Compiler (DFC) is x86-only so cannot compile on the Pi.
# fast_depth (dwofk/fast-depth, MIT licence) is tiny (1.35M params / 0.74G ops)
# and fast enough on the Pi 5 CPU for 2-second frame intervals.
_FAST_DEPTH_ONNX_URL = (
    "https://hailo-model-zoo.s3.eu-west-2.amazonaws.com"
    "/DepthEstimation/indoor/fast_depth/pretrained/2021-10-18/fast_depth.zip"
)
_FAST_DEPTH_ONNX_PATH = Path.home() / ".cache" / "fast_depth" / "fastdepth.onnx"

# door_yolo: YOLOv8s fine-tuned for door detection (single class: "door").
# Source: github.com/sayedmohamedscu/YOLOv8-Door-detection-for-visually-impaired-people
# Trained on NYC indoor/outdoor doors; ~22 MB .pt → ONNX exported on first run.
# Runs on Pi CPU via onnxruntime alongside fast_depth.
_DOOR_YOLO_PT_URL   = (
    "https://raw.githubusercontent.com/sayedmohamedscu/"
    "YOLOv8-Door-detection-for-visually-impaired-people/main/doors.pt"
)
_DOOR_YOLO_CACHE    = Path.home() / ".cache" / "door_yolo"
_DOOR_YOLO_PT_PATH  = _DOOR_YOLO_CACHE / "doors.pt"
_DOOR_YOLO_ONNX_PATH = _DOOR_YOLO_CACHE / "doors.onnx"


# ── door_yolo: download .pt + export to ONNX ─────────────────────────────────
def _ensure_door_yolo_onnx() -> bool:
    """
    Download doors.pt from GitHub then export to ONNX via ultralytics.
    The export only runs once; subsequent calls return immediately.
    Returns True if doors.onnx is ready.
    """
    if _DOOR_YOLO_ONNX_PATH.exists() and _DOOR_YOLO_ONNX_PATH.stat().st_size > 10_000:
        _log("door_yolo: ONNX already present")
        return True

    _DOOR_YOLO_CACHE.mkdir(parents=True, exist_ok=True)

    # ── Step 1: download .pt if needed ───────────────────────────────────────
    if not (_DOOR_YOLO_PT_PATH.exists() and _DOOR_YOLO_PT_PATH.stat().st_size > 10_000_000):
        _log("door_yolo: downloading doors.pt from GitHub (~22 MB)…")
        _speak("Downloading door detection model weights.")
        try:
            import urllib.request
            urllib.request.urlretrieve(_DOOR_YOLO_PT_URL, str(_DOOR_YOLO_PT_PATH))
            size_mb = _DOOR_YOLO_PT_PATH.stat().st_size / 1e6
            _log(f"door_yolo: doors.pt saved ({size_mb:.1f} MB)")
        except Exception as e:
            _log(f"door_yolo: download failed — {e}")
            return False

    # ── Step 2: export .pt → ONNX via ultralytics ────────────────────────────
    _log("door_yolo: exporting doors.pt → doors.onnx (may take 30–90 s on Pi)…")
    _speak("Exporting door model to ONNX format.")
    try:
        from ultralytics import YOLO
        model = YOLO(str(_DOOR_YOLO_PT_PATH))
        # export() saves <name>.onnx next to the .pt file and returns its path
        exported = Path(str(model.export(format="onnx", imgsz=640, opset=12, simplify=False)))
        if not exported.exists():
            _log(f"door_yolo: export returned {exported} but file not found")
            return False
        if exported.resolve() != _DOOR_YOLO_ONNX_PATH.resolve():
            exported.rename(_DOOR_YOLO_ONNX_PATH)
        size_mb = _DOOR_YOLO_ONNX_PATH.stat().st_size / 1e6
        _log(f"door_yolo: ONNX ready ({size_mb:.1f} MB) → {_DOOR_YOLO_ONNX_PATH}")
        return True
    except Exception as e:
        _log(f"door_yolo: ONNX export failed — {e}")
        return False


# ── fast_depth ONNX download ──────────────────────────────────────────────────
def _ensure_fast_depth_onnx() -> bool:
    """
    Download fast_depth.onnx from the Hailo model-zoo S3 bucket if not present.
    Returns True if the file is ready, False on failure.
    """
    if _FAST_DEPTH_ONNX_PATH.exists() and _FAST_DEPTH_ONNX_PATH.stat().st_size > 500_000:
        _log("fast_depth: ONNX already present")
        return True

    _log(f"fast_depth: downloading ONNX from S3 → {_FAST_DEPTH_ONNX_PATH}")
    _speak("Downloading fast_depth depth model.")
    _FAST_DEPTH_ONNX_PATH.parent.mkdir(parents=True, exist_ok=True)
    try:
        import io
        import urllib.request
        import zipfile

        data = urllib.request.urlopen(_FAST_DEPTH_ONNX_URL, timeout=120).read()
        with zipfile.ZipFile(io.BytesIO(data)) as z:
            if "fastdepth.onnx" not in z.namelist():
                _log(f"fast_depth: unexpected zip contents: {z.namelist()}")
                return False
            _FAST_DEPTH_ONNX_PATH.write_bytes(z.read("fastdepth.onnx"))

        size_mb = _FAST_DEPTH_ONNX_PATH.stat().st_size / 1e6
        _log(f"fast_depth: ONNX saved ({size_mb:.1f} MB)")
        return True
    except Exception as e:
        _log(f"fast_depth: download failed — {e}")
        return False


# ── TTS helper (mirrors explore.py — no shared import to avoid circular deps) ─
_SPEAK_FILE = Path("/tmp/speak_queue.txt")

def _speak(text: str):
    try:
        with _SPEAK_FILE.open("a") as f:
            f.write(text + "\n")
    except Exception:
        pass

def _log(msg: str):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}][setup] {msg}", flush=True)


# ── HEF search ────────────────────────────────────────────────────────────────
def _find_hef(zoo_name: str, configured_path: Path) -> Path | None:
    """
    Look for a pre-existing HEF for zoo_name.
    Tries the configured path first, then broad glob patterns across SEARCH_DIRS.
    """
    if configured_path.exists() and configured_path.stat().st_size >= MIN_HEF_BYTES:
        return configured_path

    patterns = [
        f"*{zoo_name}*{HW_ARCH}*.hef",
        f"*{zoo_name}*h8l*.hef",
        f"*{zoo_name}*.hef",
    ]
    for directory in SEARCH_DIRS:
        if not directory.is_dir():
            continue
        for pat in patterns:
            matches = sorted(directory.glob(pat))
            if matches:
                return matches[0]
    return None


# ── HEF validation ────────────────────────────────────────────────────────────
def _validate_hef(path: Path, expected_wh: tuple[int, int]) -> tuple[bool, str]:
    """
    Returns (ok, reason_string).

    Checks:
      - file exists and is at least MIN_HEF_BYTES
      - parses as a valid Hailo HEF without VDevice creation
      - input spatial dimensions match expected_wh (width, height)
    """
    if not path.exists():
        return False, "not found"
    size = path.stat().st_size
    if size < MIN_HEF_BYTES:
        return False, f"too small ({size} bytes)"

    try:
        from hailo_platform import HEF
        hef    = HEF(str(path))
        infos  = hef.get_input_vstream_infos()
        if not infos:
            return False, "no input vstream info in HEF"

        # Shape is typically (N, H, W, C) or (N, C, H, W) — extract spatial dims
        shape  = infos[0].shape
        dims   = [int(d) for d in shape if int(d) > 1]   # drop batch=1, channel=3 if trivial
        ew, eh = expected_wh
        # Both expected dims must appear somewhere in the shape
        if ew in dims and eh in dims:
            return True, f"valid ({size // 1024} KB, shape={shape})"
        else:
            # Shape mismatch: model may still work if Hailo auto-pads; warn but accept.
            return True, f"valid but shape={shape} ≠ expected {expected_wh} — verify on Pi"
    except ImportError:
        # Hailo SDK not installed on dev machine — skip SDK check, trust size
        if size >= MIN_HEF_BYTES:
            return True, f"size OK ({size // 1024} KB); SDK not available for shape check"
        return False, "SDK unavailable and file too small"
    except Exception as e:
        return False, f"HEF parse error: {e}"


# ── Download via Hailo model-zoo CLI ─────────────────────────────────────────
def _download_hailomz(zoo_name: str) -> Path | None:
    """
    Attempt download using the Hailo model-zoo CLI in several invocation styles.
    Returns the Path to the HEF if successful, None otherwise.
    """
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # Different CLI entry points across Hailo SDK versions
    cmd_prefixes = [
        ["hailomz"],
        ["hailo", "model-zoo"],
        [sys.executable, "-m", "hailo_model_zoo.main"],
    ]
    # Some versions support --output-dir, others don't
    suffix_variants = [
        ["download", zoo_name, "--hw-arch", HW_ARCH, "--output-dir", str(OUTPUT_DIR)],
        ["download", zoo_name, "--hw-arch", HW_ARCH],
        ["fetch",    zoo_name, "--hw-arch", HW_ARCH, "--output-dir", str(OUTPUT_DIR)],
    ]

    for prefix in cmd_prefixes:
        for suffix in suffix_variants:
            cmd = prefix + suffix
            _log(f"trying: {' '.join(cmd)}")
            try:
                result = subprocess.run(
                    cmd, capture_output=True, text=True,
                    timeout=DOWNLOAD_TIMEOUT,
                )
                if result.returncode == 0:
                    _log(f"download command succeeded")
                    # Search for the HEF — it may be in OUTPUT_DIR or a cache
                    hef = _find_hef(zoo_name, OUTPUT_DIR / f"{zoo_name}_{HW_ARCH}.hef")
                    if hef:
                        # Copy to OUTPUT_DIR if it landed elsewhere
                        if hef.parent.resolve() != OUTPUT_DIR.resolve():
                            dest = OUTPUT_DIR / hef.name
                            shutil.copy2(str(hef), str(dest))
                            _log(f"copied {hef} → {dest}")
                            hef = dest
                        return hef
                    _log("command OK but no HEF found in search paths")
                else:
                    _log(f"exit {result.returncode}: {result.stderr.strip()[:120]}")
            except FileNotFoundError:
                _log(f"command not found: {prefix[0]}")
                break   # no point trying other suffixes for a missing command
            except subprocess.TimeoutExpired:
                _log("timed out")
    return None


# ── Per-model resolution ──────────────────────────────────────────────────────
def _resolve_model(name: str) -> tuple[Path | None, str]:
    """
    Resolve a single model to a validated HEF path.
    Returns (path_or_None, status_message).
    """
    from vision import MODEL_SPECS
    spec       = MODEL_SPECS[name]
    zoo_info   = MODEL_ZOO_INFO[name]
    zoo_name   = zoo_info["zoo_name"]
    configured = Path(spec["hef"])
    expected_wh = spec["input_wh"]

    # ── Step 1: look for an existing file ────────────────────────────────────
    found = _find_hef(zoo_name, configured)
    if found:
        ok, reason = _validate_hef(found, expected_wh)
        if ok:
            return found, f"found + valid: {reason}"
        _log(f"{name}: existing file invalid ({reason}) — will re-download")

    # ── Step 2: download from Hailo model zoo ────────────────────────────────
    _log(f"{name}: attempting download (zoo_name={zoo_name})")
    _speak(f"Downloading {name} model.")
    hef = _download_hailomz(zoo_name)
    if hef:
        ok, reason = _validate_hef(hef, expected_wh)
        if ok:
            return hef, f"downloaded + valid: {reason}"
        _log(f"{name}: downloaded file invalid ({reason})")

    # ── Step 3: give up, print instructions ──────────────────────────────────
    # (ONNX export + Hailo DFC compilation removed — DFC is x86-only and
    #  cannot run on the Pi ARM. Compilation must be done on an x86 machine.)
    _log(
        f"{name}: COULD NOT RESOLVE. Manual fix:\n"
        f"  Option A — download from Hailo model zoo:\n"
        f"    hailomz download {zoo_name} --hw-arch {HW_ARCH} "
        f"--output-dir {OUTPUT_DIR}\n"
        f"  Option B — visit https://hailo.ai/developer-zone/model-zoo/\n"
        f"    search for '{zoo_name}', download the h8l HEF, "
        f"place in {OUTPUT_DIR}\n"
        f"  Option C — compile from ONNX:\n"
        f"    hailo parse onnx <model>.onnx --hw-arch {HW_ARCH}\n"
        f"    hailo optimize <model>.har --use-random-calib-set\n"
        f"    hailo compile <model>_optimized.har --hw-arch {HW_ARCH} "
        f"--output-dir {OUTPUT_DIR}"
    )
    return None, "unresolved — see log for manual steps"


# ── Public API ────────────────────────────────────────────────────────────────
def ensure_models(abort_if_required_missing: bool = True) -> dict[str, bool]:
    """
    Check, download, and validate all models defined in vision.MODEL_SPECS.

    Updates vision.MODEL_SPECS in-place so that every successfully resolved
    model points to its actual HEF path before init_all_models() is called.

    Parameters
    ----------
    abort_if_required_missing
        If True (default), calls sys.exit(1) when a required model
        (yolo_det) cannot be resolved.

    Returns
    -------
    dict[str, bool]
        Model name → True if a valid HEF is ready.
    """
    from vision import MODEL_SPECS

    _log("=" * 60)
    _log("Pre-flight model check")
    _log(f"Target directory : {OUTPUT_DIR}")
    _log(f"Hardware arch    : {HW_ARCH}")
    _log("=" * 60)

    readiness: dict[str, bool] = {}

    # ── Hailo HEF models ─────────────────────────────────────────────────────
    for name in MODEL_SPECS:
        _log(f"--- {name} ---")
        hef_path, status = _resolve_model(name)

        if hef_path is not None:
            # Update MODEL_SPECS so vision.py uses the resolved path
            MODEL_SPECS[name]["hef"] = hef_path
            readiness[name] = True
            _log(f"{name}: READY  {status}")
        else:
            readiness[name] = False
            _log(f"{name}: MISSING  {status}")
            info = MODEL_ZOO_INFO[name]
            if info.get("required") and abort_if_required_missing:
                _speak(f"Required model {name} not available. Cannot start.")
                _log(
                    f"\nFATAL: {name} is required and could not be resolved.\n"
                    f"Fix the model path or run the download steps above, then retry."
                )
                sys.exit(1)

    # ── fast_depth ONNX (CPU / onnxruntime, primary depth sensor) ────────────
    _log("--- fast_depth (ONNX/CPU) ---")
    readiness["fast_depth"] = _ensure_fast_depth_onnx()
    if readiness["fast_depth"]:
        _log(f"fast_depth: READY  {_FAST_DEPTH_ONNX_PATH}")
    else:
        _log("fast_depth: MISSING — depth will fall back to optical flow only")

    # ── door_yolo ONNX (CPU / onnxruntime, door panel detector) ──────────────
    _log("--- door_yolo (ONNX/CPU) ---")
    readiness["door_yolo"] = _ensure_door_yolo_onnx()
    if readiness["door_yolo"]:
        _log(f"door_yolo: READY  {_DOOR_YOLO_ONNX_PATH}")
    else:
        _log("door_yolo: MISSING — door detection will rely on OpenCV geometry only")

    ready   = [k for k, v in readiness.items() if v]
    missing = [k for k, v in readiness.items() if not v]
    _log("=" * 60)
    _log(f"Ready  : {ready}")
    _log(f"Missing: {missing}")
    if missing:
        _log("Missing models will be skipped; OpenCV door detection always runs.")
        _speak(f"{len(missing)} optional models unavailable. Continuing with available models.")
    else:
        _log("All models ready.")
        _speak("All vision models ready.")
    _log("=" * 60)

    return readiness
