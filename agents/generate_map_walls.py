"""
One-time generator: extracts wall segment data from the simulator source and
writes agents/map_walls.py so run_agent_real.py can ray-cast against walls
without the simulator running.

Run from agents/:
    uv run generate_map_walls.py
"""

import re
from pathlib import Path

MAP_DATA = Path("../simulator/src/map_data.rs")
PATTERN = re.compile(
    r"walls\.push\(Segment \{ p1: pdf_pt\(([^,]+),\s*([^)]+)\), p2: pdf_pt\(([^,]+),\s*([^)]+)\) \}\);"
)
PDF_TO_MM = 36.0

walls = []
for m in PATTERN.finditer(MAP_DATA.read_text()):
    x1, y1, x2, y2 = (float(v.strip()) for v in m.groups())
    walls.append(((x1 * PDF_TO_MM, y1 * PDF_TO_MM), (x2 * PDF_TO_MM, y2 * PDF_TO_MM)))

out = Path("map_walls.py")
out.write_text(
    "# Auto-generated from simulator/src/map_data.rs — do not edit manually.\n"
    "# Re-run generate_map_walls.py if the map changes.\n"
    f"WALLS: list[tuple[tuple[float, float], tuple[float, float]]] = {walls!r}\n"
)
print(f"Wrote {len(walls)} wall segments to {out}")
