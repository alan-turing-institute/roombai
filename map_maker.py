#!/usr/bin/env python3
"""
map_maker.py — Annotated overhead map for a Roomba explore.py run.

Usage (two modes):

  1. Real-time (imported by explore.py):
       from map_maker import MapRecorder
       rec = MapRecorder()
       rec.record_position(x, y, heading)
       rec.record_bump(rx, ry, heading)
       rec.record_yolo(rx, ry, heading, "chair", "center", 90.0)
       rec.record_door(rx, ry, heading, "center", 120.0, door_open=True)
       rec.save_events()          # write /tmp/roomba_events.json
       path = rec.save_map()      # write timestamped PNG → /tmp/roomba_maps/

  2. Post-run standalone:
       python3 map_maker.py                        # uses /tmp/roomba_events.json
       python3 map_maker.py /tmp/roomba_events.json
"""

import json
import math
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Optional

EVENTS_FILE = Path("/tmp/roomba_events.json")
MAP_DIR     = Path(__file__).parent / "roommap"

# How close (cm) two events of the same label must be before we discard the
# duplicate.  Keeps the map readable without losing meaningful spread.
DEDUP_RADIUS_CM = 50.0
DOOR_DEDUP_CM   = 80.0
BUMP_DEDUP_CM   = 30.0


@dataclass
class MapEvent:
    world_x:   float
    world_y:   float
    label:     str    # human-readable object name
    method:    str    # "bumper" | "camera"
    timestamp: float = field(default_factory=time.time)


class MapRecorder:
    """Thread-safe recorder; call save_map() at end of run."""

    def __init__(self):
        self._lock   = threading.Lock()
        self._path:   list[tuple[float, float, float]] = [(0.0, 0.0, 0.0)]
        self._events: list[MapEvent] = []

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _project(
        self, robot_x: float, robot_y: float, heading: float,
        frame_pos: str, distance_cm: float,
    ) -> tuple[float, float]:
        offset_deg  = {"left": 25.0, "center": 0.0, "right": -25.0}.get(frame_pos, 0.0)
        bearing_rad = math.radians((heading + offset_deg) % 360)
        return (
            robot_x + distance_cm * math.cos(bearing_rad),
            robot_y + distance_cm * math.sin(bearing_rad),
        )

    def _near(self, label: str, wx: float, wy: float, radius: float) -> bool:
        for ev in self._events:
            if ev.label == label and math.hypot(ev.world_x - wx, ev.world_y - wy) < radius:
                return True
        return False

    # ── Recording API ─────────────────────────────────────────────────────────

    def record_position(self, x: float, y: float, heading: float):
        with self._lock:
            self._path.append((x, y, heading))

    def record_bump(self, robot_x: float, robot_y: float, heading: float):
        rad = math.radians(heading)
        bx  = robot_x + 25.0 * math.cos(rad)
        by  = robot_y + 25.0 * math.sin(rad)
        with self._lock:
            if not self._near("bump", bx, by, BUMP_DEDUP_CM):
                self._events.append(MapEvent(bx, by, "bump", "bumper"))

    def record_yolo(
        self,
        robot_x: float, robot_y: float, heading: float,
        class_name: str,
        frame_position: str,
        distance_cm: Optional[float],
    ):
        dist = distance_cm if distance_cm else 150.0
        wx, wy = self._project(robot_x, robot_y, heading, frame_position, dist)
        with self._lock:
            if not self._near(class_name, wx, wy, DEDUP_RADIUS_CM):
                self._events.append(MapEvent(wx, wy, class_name, "camera"))

    def record_door(
        self,
        robot_x: float, robot_y: float, heading: float,
        door_position: str,
        door_distance_cm: Optional[float],
        door_open: bool,
    ):
        dist  = door_distance_cm if door_distance_cm else 150.0
        wx, wy = self._project(robot_x, robot_y, heading, door_position or "center", dist)
        label = f"door ({'open' if door_open else 'closed'})"
        with self._lock:
            # Replace a "door (closed)" marker at the same spot if now open
            if door_open:
                self._events = [
                    e for e in self._events
                    if not (e.label == "door (closed)" and
                            math.hypot(e.world_x - wx, e.world_y - wy) < DOOR_DEDUP_CM)
                ]
            if not self._near(label, wx, wy, DOOR_DEDUP_CM):
                self._events.append(MapEvent(wx, wy, label, "camera"))

    # ── Persistence ───────────────────────────────────────────────────────────

    def save_events(self, path: Path = EVENTS_FILE):
        with self._lock:
            data = {
                "path":      self._path,
                "events":    [asdict(e) for e in self._events],
                "saved_at":  time.strftime("%Y-%m-%d %H:%M:%S"),
            }
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=2))

    @classmethod
    def load(cls, path: Path = EVENTS_FILE) -> "MapRecorder":
        data     = json.loads(path.read_text())
        rec      = cls()
        rec._path   = [tuple(p) for p in data["path"]]      # type: ignore[assignment]
        rec._events = [MapEvent(**e) for e in data["events"]]
        return rec

    # ── Rendering ─────────────────────────────────────────────────────────────

    def save_map(self, out_path: Optional[str] = None) -> str:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        MAP_DIR.mkdir(parents=True, exist_ok=True)
        if out_path is None:
            ts       = time.strftime("%Y-%m-%d_%H-%M-%S")
            out_path = str(MAP_DIR / f"map_{ts}.png")

        with self._lock:
            path_snap   = list(self._path)
            events_snap = list(self._events)

        fig, ax = plt.subplots(figsize=(14, 11))
        ax.set_aspect("equal")
        ax.set_title(
            f"Roomba Run Map  —  {time.strftime('%Y-%m-%d %H:%M:%S')}",
            fontsize=13, fontweight="bold",
        )
        ax.set_xlabel("x (cm) — forward from start")
        ax.set_ylabel("y (cm) — left from start")
        ax.grid(True, alpha=0.25, linestyle="--")

        # ── Robot path ────────────────────────────────────────────────────────
        if len(path_snap) >= 2:
            xs = [p[0] for p in path_snap]
            ys = [p[1] for p in path_snap]
            ax.plot(xs, ys, "-", color="#3498db", linewidth=1.5, alpha=0.6,
                    label="robot path", zorder=2)
            # Direction arrow at the final position
            fx, fy, fh = path_snap[-1]
            fh_rad = math.radians(fh)
            ax.annotate(
                "",
                xy=(fx + 18 * math.cos(fh_rad), fy + 18 * math.sin(fh_rad)),
                xytext=(fx, fy),
                arrowprops=dict(arrowstyle="->", color="#3498db", lw=2.0),
                zorder=3,
            )

        ax.plot(0, 0, "g^", markersize=14, label="start", zorder=6)
        if len(path_snap) >= 2:
            ax.plot(path_snap[-1][0], path_snap[-1][1],
                    "rs", markersize=10, label="end", zorder=6)

        # ── Events ────────────────────────────────────────────────────────────
        STYLES: dict[str, dict] = {
            "bump":           dict(marker="x",  color="#e74c3c", ms=11, mew=2.5),
            "door (open)":    dict(marker="D",  color="#2ecc71", ms=10, mew=1.5),
            "door (closed)":  dict(marker="D",  color="#f39c12", ms=10, mew=1.5),
            "person":         dict(marker="*",  color="#9b59b6", ms=13, mew=1.0),
            "chair":          dict(marker="s",  color="#e67e22", ms=9,  mew=1.0),
            "couch":          dict(marker="h",  color="#d35400", ms=9,  mew=1.0),
            "dining table":   dict(marker="p",  color="#1abc9c", ms=9,  mew=1.0),
            "bed":            dict(marker="P",  color="#16a085", ms=9,  mew=1.0),
            "potted plant":   dict(marker="^",  color="#27ae60", ms=8,  mew=1.0),
            "tv":             dict(marker="8",  color="#2980b9", ms=9,  mew=1.0),
        }
        DEFAULT_STYLE = dict(marker="o", color="#7f8c8d", ms=7, mew=1.0)

        legend_seen: set[str] = set()

        for ev in events_snap:
            s    = STYLES.get(ev.label, DEFAULT_STYLE)
            lkey = ev.label
            ax.plot(
                ev.world_x, ev.world_y,
                s["marker"], color=s["color"],
                markersize=s["ms"], markeredgewidth=s["mew"],
                markeredgecolor="white" if s["marker"] not in ("x",) else s["color"],
                label=(lkey if lkey not in legend_seen else "_"),
                zorder=5,
            )
            legend_seen.add(lkey)

            # Two-line annotation: what it is + how detected
            ax.annotate(
                f"{ev.label}\n({ev.method})",
                (ev.world_x, ev.world_y),
                textcoords="offset points",
                xytext=(6, 3),
                fontsize=6,
                color=s["color"],
                alpha=0.88,
                zorder=6,
            )

        ax.legend(loc="upper right", fontsize=8, framealpha=0.85,
                  markerscale=1.2, labelspacing=0.4)

        # ── Stats box ─────────────────────────────────────────────────────────
        n_bumps  = sum(1 for e in events_snap if e.method == "bumper")
        n_cam    = sum(1 for e in events_snap if e.method == "camera")
        n_doors  = sum(1 for e in events_snap if "door" in e.label)
        n_pts    = len(path_snap)
        stats    = (
            f"path pts: {n_pts}   bumps: {n_bumps}\n"
            f"camera events: {n_cam}   doors seen: {n_doors}"
        )
        ax.text(
            0.01, 0.01, stats, transform=ax.transAxes,
            fontsize=7.5, verticalalignment="bottom",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.7),
        )

        fig.tight_layout()
        fig.savefig(out_path, dpi=130, bbox_inches="tight")
        plt.close(fig)
        return out_path


# ── Standalone entry point ────────────────────────────────────────────────────

def main():
    import sys
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else EVENTS_FILE
    if not src.exists():
        print(f"No events file at {src}")
        print("Run explore.py first, or point to a saved JSON:  python3 map_maker.py <file>")
        raise SystemExit(1)
    rec = MapRecorder.load(src)
    out = rec.save_map()
    print(f"Map saved → {out}")


if __name__ == "__main__":
    main()
