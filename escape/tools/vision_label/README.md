# Vision labeling tools

Offline dev tools (not part of the runtime). Two scripts, run with two
different Pythons because of how Tk/PIL are split across the macOS interpreters
(no single one here has both):

| Task | Script | Interpreter (needs) |
|---|---|---|
| Draw the boundary labels | `label_ui.py` | `/usr/local/bin/python3` (tkinter, Tk 9) |
| Render overlays to review | `render_overlay.py` | `/opt/homebrew/bin/python3` (Pillow) |

`label_ui.py` deliberately avoids PIL/numpy: it displays PNGs (generated from
your JPEGs by `sips`) and writes JSON labels. `render_overlay.py` uses Pillow
just to burn the saved label back onto the image for a visual check.

## Label

Put the photos you want as fixtures in `photos/` (curate freely — the tool
labels every image in there; the camera will be positioned so the robot is not
in frame at test time, so pick frames accordingly). Then:

```bash
/usr/local/bin/python3 escape/tools/vision_label/label_ui.py
# defaults: --photos ../photos  --out crates/vision/tests/fixtures  --cols 24 --zoom 2
```

For each photo it auto-creates a 640×360 JPEG fixture (`fixtures/images/`) and a
640×360 display PNG (`tools/vision_label/work/`), then opens the canvas.

Controls:

```
left-drag        draw the green boundary (top edge of drivable floor)
b / f / o / r    mode: Boundary / Floor-point / Obstacle-point / ignore-Rect
left-click       f/o: drop a sample point;   r: press-drag a rectangle
u                undo last point in the active mode
c                clear the boundary (fully open)      x  clear all points + rect
h                toggle holdout (exclude from tuning)
[  /  ]          previous / next image (auto-saves)    s  save now
?                toggle help                           q / Esc  save and quit
```

Boundary semantics: the line marks where the drivable floor ends. Untouched
columns stay "fully open" (free to the top). Stored per column as
`free_frac` (fraction of image height that is floor, from the bottom) —
resolution-independent, which is exactly the `vision::FreeSpace` contract.

A headless logic check (no display): `python3 label_ui.py --selftest`.

## Review

```bash
/opt/homebrew/bin/python3 escape/tools/vision_label/render_overlay.py --all \
  escape/crates/vision/tests/fixtures/labels \
  escape/crates/vision/tests/fixtures/images \
  /tmp/vision_overlays
```

Then open the PNGs in `/tmp/vision_overlays/`.

## Validate

`cargo test -p vision` runs `fixtures_are_well_formed`, which checks every label
parses, matches its image dimensions, and has an in-range 24-column profile.
