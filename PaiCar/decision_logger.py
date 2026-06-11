"""
Records every plan_move decision to disk.

For each call to log():
  - writes a JPEG copy of the input snapshot to <log_dir>/<timestamp>.jpg
  - appends one JSON line to <log_dir>/decisions.jsonl with the timestamp,
    image path, and all MoveAction fields (direction, distance, clearances,
    reasoning).

Usage in drive.py:
    import decision_logger
    action = planner.plan_move(image_b64, ...)
    decision_logger.log(image_b64, action, log_dir=pathlib.Path("logs"))
"""

from __future__ import annotations

import base64
import dataclasses
import datetime
import json
import pathlib
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from move_planner import MoveAction


def log(
    image_b64: str,
    action: "MoveAction",
    log_dir: pathlib.Path = pathlib.Path("logs"),
) -> pathlib.Path:
    """Save the snapshot and decision. Returns the path to the saved image."""
    log_dir = pathlib.Path(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    ts = datetime.datetime.now().strftime("%Y%m%dT%H%M%S_%f")
    img_path = log_dir / f"{ts}.jpg"
    img_path.write_bytes(base64.b64decode(image_b64))

    record = {
        "timestamp": ts,
        "image_path": str(img_path),
        **dataclasses.asdict(action),
    }
    with (log_dir / "decisions.jsonl").open("a") as fh:
        fh.write(json.dumps(record) + "\n")

    return img_path
