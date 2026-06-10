#!/usr/bin/env python3
"""Render a per-column free-space label as an overlay on its fixture image, so a
human can confirm/correct the draft boundary at a glance.

Offline dev tool only (not part of the runtime). Pillow is the single dep.

Usage:
    python render_overlay.py LABEL.json [--out OUT.png]
    python render_overlay.py --all LABELS_DIR IMAGES_DIR OUT_DIR

The overlay draws:
  - vertical column dividers,
  - a green polyline at the labeled floor boundary (free_frac from the bottom),
  - green dots for floor sample points, red dots for obstacle points,
  - a hatched box over the ignore_rect (timestamp overlay).
"""
import argparse
import json
import os
import sys

try:
    from PIL import Image, ImageDraw
except ImportError:
    sys.exit("Pillow required: pip install pillow")


def render(label, images_dir):
    img_path = os.path.join(images_dir, label["image"])
    im = Image.open(img_path).convert("RGB")
    w, h = im.size
    d = ImageDraw.Draw(im, "RGBA")

    cols = label["columns"]
    n = len(cols)
    col_w = w / n

    # Boundary polyline: y where floor ends, at each column centre.
    pts = []
    for i, frac in enumerate(cols):
        cx = (i + 0.5) * col_w
        by = h * (1.0 - frac)
        pts.append((cx, by))
        d.line([(i * col_w, 0), (i * col_w, h)], fill=(255, 255, 255, 40), width=1)
    if len(pts) >= 2:
        d.line(pts, fill=(0, 255, 0, 220), width=3)
    for (cx, by) in pts:
        d.ellipse([cx - 3, by - 3, cx + 3, by + 3], fill=(0, 255, 0, 255))
        # shade the free (floor) region below the boundary
        ci = pts.index((cx, by))
        d.rectangle([ci * col_w, by, (ci + 1) * col_w, h], fill=(0, 255, 0, 30))

    for (px, py) in label.get("floor_points", []):
        d.ellipse([px - 5, py - 5, px + 5, py + 5], outline=(0, 255, 0, 255), width=2)
    for (px, py) in label.get("obstacle_points", []):
        d.ellipse([px - 5, py - 5, px + 5, py + 5], outline=(255, 0, 0, 255), width=2)

    rect = label.get("ignore_rect")
    if rect:
        x, y, rw, rh = rect
        d.rectangle([x, y, x + rw, y + rh], fill=(255, 0, 255, 70), outline=(255, 0, 255, 255))

    return im


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("label", nargs="?")
    ap.add_argument("--out")
    ap.add_argument("--all", nargs=3, metavar=("LABELS_DIR", "IMAGES_DIR", "OUT_DIR"))
    args = ap.parse_args()

    if args.all:
        labels_dir, images_dir, out_dir = args.all
        os.makedirs(out_dir, exist_ok=True)
        for fn in sorted(os.listdir(labels_dir)):
            if not fn.endswith(".json"):
                continue
            label = json.load(open(os.path.join(labels_dir, fn)))
            im = render(label, images_dir)
            out = os.path.join(out_dir, fn.replace(".json", "_overlay.png"))
            im.save(out)
            print(out)
    elif args.label:
        label = json.load(open(args.label))
        images_dir = os.path.dirname(args.label).replace("labels", "images")
        im = render(label, images_dir)
        out = args.out or args.label.replace(".json", "_overlay.png")
        im.save(out)
        print(out)
    else:
        ap.error("provide LABEL.json or --all")


if __name__ == "__main__":
    main()
