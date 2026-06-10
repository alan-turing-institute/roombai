#!/usr/bin/env python3
"""Overlay human label (green) vs Rust prediction (red) on each fixture.

  cargo run -p vision --example dump_predictions -- /tmp/pred
  /opt/homebrew/bin/python3 tools/vision_label/render_compare.py /tmp/pred /tmp/compare
"""
import glob
import json
import os
import sys

from PIL import Image, ImageDraw

FIXT = "crates/vision/tests/fixtures"


def line(d, cols, color, w, h, n):
    pts = [((k + 0.5) / n * w, (1 - cols[k]) * h) for k in range(len(cols))]
    d.line(pts, fill=color, width=3)


def main():
    pred_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/pred"
    out = sys.argv[2] if len(sys.argv) > 2 else "/tmp/compare"
    os.makedirs(out, exist_ok=True)
    for lp in sorted(glob.glob(f"{FIXT}/labels/*.json")):
        truth = json.load(open(lp))
        name = truth["image"]
        pp = os.path.join(pred_dir, name.replace(".jpg", ".json"))
        if not os.path.exists(pp):
            continue
        pred = json.load(open(pp))
        im = Image.open(f"{FIXT}/images/{name}").convert("RGB")
        d = ImageDraw.Draw(im)
        w, h = im.size
        line(d, truth["columns"], (0, 255, 0), w, h, len(truth["columns"]))   # human
        line(d, pred["columns"], (255, 40, 40), w, h, len(pred["columns"]))   # rust
        im.save(os.path.join(out, name.replace(".jpg", ".png")))
    print("green=human  red=rust ->", out)


if __name__ == "__main__":
    main()
