#!/usr/bin/env python3
"""
check_deps.py — Pre-flight dependency check for RoombaI.

For each required Python package:
  1. Try to import it.
  2. If missing, attempt installation (pip, apt, or local path as appropriate).
  3. Re-try the import.
  4. If still missing: mark as FAILED and report clearly.

Exit codes:
  0 — all critical dependencies satisfied (warnings for optional ones are OK)
  1 — one or more critical dependencies could not be resolved

Run inside the ~/yolo_new virtual environment (run.sh does this automatically).
"""

import importlib
import subprocess
import sys
from pathlib import Path

# ── Dependency table ──────────────────────────────────────────────────────────
# Each entry: (import_name, pip_name_or_None, apt_name_or_None,
#              local_path_or_None, critical)
#
# Resolution order: pip → apt → local_path
# If pip_name is None the pip step is skipped.
# If apt_name  is None the apt step is skipped.
# If local_path is not None it is tried as a final pip install source.
#
DEPS = [
    # import_name        pip_name              apt_name              local_path                          critical
    ("cv2",              "opencv-python",      "python3-opencv",     None,                               True),
    ("numpy",            "numpy",              "python3-numpy",      None,                               True),
    ("matplotlib",       "matplotlib",         "python3-matplotlib", None,                               True),
    ("hailo_platform",   None,                 None,                 None,                               True),   # Hailo SDK — system only
    ("numba",            "numba",              "python3-numba",      None,                               False),  # needed by hailo_model_zoo
    ("hailo_model_zoo",  None,                 None,                 str(Path.home()/"hailo-model-zoo"), False),
    ("ultralytics",      "ultralytics",        None,                 None,                               False),
    ("PIL",              "Pillow",             "python3-pil",        None,                               False),
]


# ── Helpers ───────────────────────────────────────────────────────────────────
def can_import(name: str) -> bool:
    try:
        importlib.import_module(name)
        return True
    except ImportError:
        return False


def pip_install(pkg: str) -> bool:
    result = subprocess.run(
        [sys.executable, "-m", "pip", "install", pkg, "-q"],
        capture_output=True,
    )
    return result.returncode == 0


def apt_install(pkg: str) -> bool:
    result = subprocess.run(
        ["sudo", "-n", "apt-get", "install", "-y", pkg],
        capture_output=True,
        env={**__import__("os").environ, "DEBIAN_FRONTEND": "noninteractive"},
    )
    return result.returncode == 0


def resolve(import_name, pip_name, apt_name, local_path) -> tuple[bool, str]:
    """Try every available install method. Return (success, method_used)."""
    if pip_name:
        print(f"    pip install {pip_name} …", flush=True)
        if pip_install(pip_name) and can_import(import_name):
            return True, f"pip:{pip_name}"

    if apt_name:
        print(f"    apt-get install {apt_name} …", flush=True)
        if apt_install(apt_name) and can_import(import_name):
            return True, f"apt:{apt_name}"

    if local_path and Path(local_path).exists():
        print(f"    pip install {local_path} …", flush=True)
        if pip_install(local_path) and can_import(import_name):
            return True, f"local:{local_path}"

    return False, "none"


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> int:
    print("\n── check_deps: pre-flight dependency check ──────────────────────────", flush=True)

    ok_list:   list[str] = []
    warn_list: list[str] = []
    fail_list: list[str] = []
    installed: list[str] = []

    for import_name, pip_name, apt_name, local_path, critical in DEPS:
        if can_import(import_name):
            ok_list.append(import_name)
            print(f"  ✓ {import_name}", flush=True)
            continue

        print(f"  ✗ {import_name} missing — attempting install…", flush=True)
        success, method = resolve(import_name, pip_name, apt_name, local_path)

        if success:
            installed.append(f"{import_name} ({method})")
            ok_list.append(import_name)
            print(f"  ✓ {import_name} installed via {method}", flush=True)
        else:
            msg = import_name
            if critical:
                fail_list.append(msg)
                print(f"  ✗ {import_name} CRITICAL — could not install", flush=True)
            else:
                warn_list.append(msg)
                print(f"  ⚠ {import_name} optional — could not install (run continues)", flush=True)

    print("\n── check_deps: summary ──────────────────────────────────────────────", flush=True)
    print(f"  OK      : {ok_list}", flush=True)
    if installed:
        print(f"  Installed: {installed}", flush=True)
    if warn_list:
        print(f"  Missing (optional): {warn_list}", flush=True)
    if fail_list:
        print(f"  FAILED (critical): {fail_list}", flush=True)
        print("\n  Cannot start — fix the critical dependencies above.", flush=True)
        print("  For hailo_platform: reinstall the Hailo SDK.", flush=True)
        return 1

    print("  All critical dependencies satisfied.\n", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
