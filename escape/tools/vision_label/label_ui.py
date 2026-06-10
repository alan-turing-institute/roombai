#!/usr/bin/env python3
"""label_ui.py — draw per-column free-space boundaries on the vision fixtures.

You draw the green line (the top edge of the drivable floor) directly on each
photo; the tool samples it into N columns and writes labels in the schema the
Rust segmenter harness consumes (vision::fixture::FloorLabel).

Environment: needs tkinter ONLY (no PIL / numpy). On macOS run it with the
Tk-9 Python:

    /usr/local/bin/python3 tools/vision_label/label_ui.py \
        --photos photos --out crates/vision/tests/fixtures

It labels every image in --photos. For each photo it auto-generates (via `sips`)
a 640x360 PNG for display and a 640x360 JPEG fixture; columns are stored
normalised (resolution-independent), points/rect in fixture pixels.

Controls:
    left-drag        draw the boundary under the cursor (Boundary mode)
    b / f / o / r    mode: Boundary / Floor-point / Obstacle-point / ignore-Rect
    left-click       f/o mode: drop a sample point;  r mode: press-drag a rect
    u                undo last point in the active mode
    c                clear the boundary (back to fully open)
    x                clear all points + ignore-rect
    h                toggle 'holdout' (exclude from algorithm tuning)
    [  /  ]          previous / next image (auto-saves current)
    s                save now
    ?                toggle the on-screen help
    q  /  Esc        save and quit

Run a headless logic check (no display needed) with: --selftest
"""
import argparse
import json
import os
import re
import subprocess
import sys

FIX_W, FIX_H = 640, 360  # fixture resolution (matches runtime processing size)
DEFAULT_COLS = 24
DEFAULT_TOL = 0.10  # tighter than the drafts: a human-drawn line is trusted more


# ----------------------------------------------------------------------------
# Pure logic (no tkinter) — covered by --selftest
# ----------------------------------------------------------------------------

def sanitize(stem):
    """photos filename stem -> safe fixture base name."""
    s = re.sub(r"[^A-Za-z0-9]+", "_", stem).strip("_").lower()
    return s or "img"


def clamp01(v):
    return 0.0 if v < 0.0 else 1.0 if v > 1.0 else v


def paint_segment(cols, prev, idx, frac):
    """Set column `idx` to `frac`; if dragging from a `prev` (pidx, pfrac),
    linearly interpolate across the columns in between so a fast drag still
    yields a continuous line. Returns the new (idx, frac) to use as `prev`."""
    n = len(cols)
    idx = max(0, min(n - 1, idx))
    frac = clamp01(frac)
    if prev is None:
        cols[idx] = frac
        return (idx, frac)
    pidx, pfrac = prev
    lo, hi = (pidx, idx) if pidx <= idx else (idx, pidx)
    for j in range(lo, hi + 1):
        if hi == lo:
            cols[j] = frac
        else:
            t = (j - pidx) / (idx - pidx) if idx != pidx else 1.0
            cols[j] = clamp01(pfrac + t * (frac - pfrac))
    return (idx, frac)


def build_label(base, cols, floor_pts, obstacle_pts, ignore_rect, holdout, notes,
                tol=DEFAULT_TOL):
    return {
        "image": base + ".jpg",
        "width": FIX_W,
        "height": FIX_H,
        "notes": notes,
        "columns": [round(float(c), 4) for c in cols],
        "floor_points": [[int(x), int(y)] for (x, y) in floor_pts],
        "obstacle_points": [[int(x), int(y)] for (x, y) in obstacle_pts],
        "ignore_rect": list(ignore_rect) if ignore_rect else None,
        "tolerance": tol,
        "holdout": bool(holdout),
    }


def selftest():
    cols = [1.0] * 8
    p = paint_segment(cols, None, 2, 0.5)
    assert cols[2] == 0.5 and p == (2, 0.5)
    paint_segment(cols, (2, 0.5), 6, 0.1)  # interpolate 2->6
    assert abs(cols[4] - 0.3) < 1e-6, cols  # midpoint
    assert abs(cols[6] - 0.1) < 1e-6
    assert sanitize("frame_0009 (1)") == "frame_0009_1"
    assert sanitize("1") == "1"
    lbl = build_label("img01", cols, [(10, 20)], [(30, 40)], [0, 0, 70, 30], True, "n")
    assert abs(lbl["columns"][6] - 0.1) < 1e-6 and lbl["holdout"] is True
    assert lbl["width"] == FIX_W and lbl["image"] == "img01.jpg"
    json.dumps(lbl)  # must be serialisable
    print("selftest ok")


# ----------------------------------------------------------------------------
# Asset prep via sips (macOS) — JPEG fixture + PNG for display
# ----------------------------------------------------------------------------

def _sips(args):
    subprocess.run(["sips", *args], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def ensure_assets(photo, fixture_jpg, work_png):
    """Generate the 640x360 fixture JPEG and display PNG if missing/stale."""
    src_mtime = os.path.getmtime(photo)
    if not os.path.exists(fixture_jpg) or os.path.getmtime(fixture_jpg) < src_mtime:
        _sips(["-z", str(FIX_H), str(FIX_W), photo, "--out", fixture_jpg])
    if not os.path.exists(work_png) or os.path.getmtime(work_png) < src_mtime:
        _sips(["-s", "format", "png", "-z", str(FIX_H), str(FIX_W), photo, "--out", work_png])


def discover(photos_dir, out_dir, work_dir):
    """Return [(base, photo, fixture_jpg, work_png)] for every photo, prepping
    assets. Sorted by base for stable ordering."""
    exts = {".jpg", ".jpeg", ".png"}
    items = []
    for fn in sorted(os.listdir(photos_dir)):
        stem, ext = os.path.splitext(fn)
        if ext.lower() not in exts:
            continue
        base = sanitize(stem)
        photo = os.path.join(photos_dir, fn)
        fixture_jpg = os.path.join(out_dir, "images", base + ".jpg")
        work_png = os.path.join(work_dir, base + ".png")
        ensure_assets(photo, fixture_jpg, work_png)
        items.append((base, photo, fixture_jpg, work_png))
    return items


# ----------------------------------------------------------------------------
# Tkinter UI
# ----------------------------------------------------------------------------

def launch(items, labels_dir, n_cols, zoom):
    import tkinter as tk

    os.makedirs(labels_dir, exist_ok=True)
    DW, DH = FIX_W * zoom, FIX_H * zoom

    root = tk.Tk()
    root.title("vision label")
    canvas = tk.Canvas(root, width=DW, height=DH, highlightthickness=0, bg="black")
    canvas.pack()

    state = {"i": 0, "mode": "b", "prev": None, "rect_start": None, "help": True,
             "photo": None}

    # per-image working data, keyed by base
    data = {}

    def label_path(base):
        return os.path.join(labels_dir, base + ".json")

    def load(base):
        if base in data:
            return data[base]
        d = {"cols": [1.0] * n_cols, "floor": [], "obst": [], "rect": None,
             "holdout": False, "notes": ""}
        p = label_path(base)
        if os.path.exists(p):
            j = json.load(open(p))
            c = j.get("columns", [])
            # resample if a different column count was stored
            if len(c) == n_cols:
                d["cols"] = [float(x) for x in c]
            elif c:
                d["cols"] = [float(c[min(len(c) - 1, int(k / n_cols * len(c)))])
                             for k in range(n_cols)]
            d["floor"] = [tuple(pt) for pt in j.get("floor_points", [])]
            d["obst"] = [tuple(pt) for pt in j.get("obstacle_points", [])]
            d["rect"] = j.get("ignore_rect")
            d["holdout"] = j.get("holdout", False)
            d["notes"] = j.get("notes", "")
        data[base] = d
        return d

    def save(base):
        d = load(base)
        lbl = build_label(base, d["cols"], d["floor"], d["obst"], d["rect"],
                          d["holdout"], d["notes"])
        json.dump(lbl, open(label_path(base), "w"), indent=2)

    def fx_from_canvas(cx, cy):
        """canvas px -> fixture px."""
        return cx / zoom, cy / zoom

    def col_frac(cx, cy):
        px, py = fx_from_canvas(cx, cy)
        idx = int(px / FIX_W * n_cols)
        frac = 1.0 - py / FIX_H
        return idx, frac

    def redraw():
        canvas.delete("ov")
        d = load(items[state["i"]][0])
        # column dividers
        for k in range(1, n_cols):
            x = k / n_cols * DW
            canvas.create_line(x, 0, x, DH, fill="#ffffff", width=1, tags="ov",
                               stipple="gray25")
        # boundary polyline + free-region tint
        pts = []
        for k, c in enumerate(d["cols"]):
            x = (k + 0.5) / n_cols * DW
            y = (1.0 - c) * DH
            pts += [x, y]
            canvas.create_line(k / n_cols * DW, y, (k + 1) / n_cols * DW, y,
                               fill="#00ff66", width=1, tags="ov")
        if len(pts) >= 4:
            canvas.create_line(*pts, fill="#00ff00", width=3, tags="ov")
        for (x, y) in d["floor"]:
            canvas.create_oval(x * zoom - 5, y * zoom - 5, x * zoom + 5, y * zoom + 5,
                               outline="#00ff00", width=2, tags="ov")
        for (x, y) in d["obst"]:
            canvas.create_oval(x * zoom - 5, y * zoom - 5, x * zoom + 5, y * zoom + 5,
                               outline="#ff3030", width=2, tags="ov")
        if d["rect"]:
            x, y, w, h = d["rect"]
            canvas.create_rectangle(x * zoom, y * zoom, (x + w) * zoom, (y + h) * zoom,
                                    outline="#ff00ff", width=2, tags="ov")
        base = items[state["i"]][0]
        status = (f"[{state['i']+1}/{len(items)}] {base}   mode={state['mode'].upper()}"
                  f"   holdout={'Y' if d['holdout'] else 'n'}"
                  f"   pts(f/o)={len(d['floor'])}/{len(d['obst'])}")
        canvas.create_text(8, 8, anchor="nw", text=status, fill="#ffff00",
                           font=("Menlo", 13), tags="ov")
        if state["help"]:
            help_txt = ("drag=draw  b/f/o/r=mode  click=point/rect  u=undo  "
                        "c=clear line  x=clear pts  h=holdout  [ ]=prev/next  "
                        "s=save  ?=help  q=quit")
            canvas.create_text(8, DH - 8, anchor="sw", text=help_txt, fill="#aaaaaa",
                               font=("Menlo", 11), tags="ov")

    def show():
        base, _photo, _fix, work_png = items[state["i"]]
        state["photo"] = tk.PhotoImage(file=work_png)
        if zoom != 1:
            state["photo"] = state["photo"].zoom(zoom, zoom)
        canvas.delete("img")
        canvas.create_image(0, 0, anchor="nw", image=state["photo"], tags="img")
        canvas.tag_lower("img")
        load(base)
        redraw()

    def goto(delta):
        save(items[state["i"]][0])
        state["i"] = (state["i"] + delta) % len(items)
        state["prev"] = None
        show()

    # ---- events ----
    def on_press(e):
        d = load(items[state["i"]][0])
        if state["mode"] == "b":
            state["prev"] = paint_segment(d["cols"], None, *col_frac(e.x, e.y))
        elif state["mode"] == "f":
            d["floor"].append(fx_from_canvas(e.x, e.y))
        elif state["mode"] == "o":
            d["obst"].append(fx_from_canvas(e.x, e.y))
        elif state["mode"] == "r":
            state["rect_start"] = fx_from_canvas(e.x, e.y)
        redraw()

    def on_drag(e):
        d = load(items[state["i"]][0])
        if state["mode"] == "b":
            idx, frac = col_frac(e.x, e.y)
            state["prev"] = paint_segment(d["cols"], state["prev"], idx, frac)
        elif state["mode"] == "r" and state["rect_start"]:
            x0, y0 = state["rect_start"]
            x1, y1 = fx_from_canvas(e.x, e.y)
            d["rect"] = [int(min(x0, x1)), int(min(y0, y1)),
                         int(abs(x1 - x0)), int(abs(y1 - y0))]
        redraw()

    def on_release(_e):
        state["prev"] = None

    def set_mode(m):
        state["mode"] = m
        redraw()

    def undo():
        d = load(items[state["i"]][0])
        if state["mode"] == "f" and d["floor"]:
            d["floor"].pop()
        elif state["mode"] == "o" and d["obst"]:
            d["obst"].pop()
        elif state["mode"] == "r":
            d["rect"] = None
        redraw()

    def clear_line():
        load(items[state["i"]][0])["cols"] = [1.0] * n_cols
        redraw()

    def clear_pts():
        d = load(items[state["i"]][0])
        d["floor"].clear(); d["obst"].clear(); d["rect"] = None
        redraw()

    def toggle_holdout():
        d = load(items[state["i"]][0]); d["holdout"] = not d["holdout"]; redraw()

    def toggle_help():
        state["help"] = not state["help"]; redraw()

    def quit_app():
        save(items[state["i"]][0]); root.destroy()

    canvas.bind("<ButtonPress-1>", on_press)
    canvas.bind("<B1-Motion>", on_drag)
    canvas.bind("<ButtonRelease-1>", on_release)
    root.bind("b", lambda e: set_mode("b"))
    root.bind("f", lambda e: set_mode("f"))
    root.bind("o", lambda e: set_mode("o"))
    root.bind("r", lambda e: set_mode("r"))
    root.bind("u", lambda e: undo())
    root.bind("c", lambda e: clear_line())
    root.bind("x", lambda e: clear_pts())
    root.bind("h", lambda e: toggle_holdout())
    root.bind("s", lambda e: save(items[state["i"]][0]))
    root.bind("<bracketleft>", lambda e: goto(-1))
    root.bind("<bracketright>", lambda e: goto(1))
    root.bind("question", lambda e: toggle_help())
    root.bind("q", lambda e: quit_app())
    root.bind("<Escape>", lambda e: quit_app())

    show()
    root.mainloop()


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, "..", ".."))  # escape/
    ap.add_argument("--photos", default=os.path.join(repo, "..", "photos"))
    ap.add_argument("--out", default=os.path.join(repo, "crates/vision/tests/fixtures"))
    ap.add_argument("--work", default=os.path.join(here, "work"))
    ap.add_argument("--cols", type=int, default=DEFAULT_COLS)
    ap.add_argument("--zoom", type=int, default=2)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return

    photos = os.path.abspath(args.photos)
    out = os.path.abspath(args.out)
    work = os.path.abspath(args.work)
    os.makedirs(os.path.join(out, "images"), exist_ok=True)
    os.makedirs(os.path.join(out, "labels"), exist_ok=True)
    os.makedirs(work, exist_ok=True)

    if not os.path.isdir(photos):
        sys.exit(f"photos dir not found: {photos}")
    items = discover(photos, out, work)
    if not items:
        sys.exit(f"no images in {photos}")
    print(f"labeling {len(items)} images from {photos}")
    print("  -> fixtures:", os.path.join(out, "images"))
    print("  -> labels:  ", os.path.join(out, "labels"))
    launch(items, os.path.join(out, "labels"), args.cols, args.zoom)


if __name__ == "__main__":
    main()
