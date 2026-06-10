#!/usr/bin/env python3
"""Prototype the floor segmenter against the human labels to choose the colour
space + parameters BEFORE porting to Rust. Offline design tool only.

Approach (Ulrich & Nourbakhsh lineage, adapted):
  1. Seed a floor colour model from a bottom-centre patch (assumed floor).
  2. Coarse HS(V) histogram backprojection -> per-pixel floor likelihood.
  3. Per-column scan up from the bottom; boundary = first sustained non-floor
     run. free_frac = (H - boundary_row) / H.

  /opt/homebrew/bin/python3 tools/vision_label/prototype_seg.py
"""
import glob
import json
import os

import numpy as np
from PIL import Image

FIXT = "crates/vision/tests/fixtures"
NCOLS = 24


def rgb_to_hsv(arr):
    r, g, b = arr[..., 0] / 255.0, arr[..., 1] / 255.0, arr[..., 2] / 255.0
    mx = np.maximum(np.maximum(r, g), b)
    mn = np.minimum(np.minimum(r, g), b)
    df = mx - mn + 1e-9
    h = np.zeros_like(mx)
    mask = mx == r
    h[mask] = (60 * ((g - b) / df) % 360)[mask]
    mask = mx == g
    h[mask] = (60 * ((b - r) / df) + 120)[mask]
    mask = mx == b
    h[mask] = (60 * ((r - g) / df) + 240)[mask]
    s = df / (mx + 1e-9)
    v = mx
    return h, s, v


def floor_likelihood(img, hbins=18, sbins=6, vbins=4, seed_rows=0.15, seed_w=0.7):
    """HSV histogram backprojection from a bottom-centre seed."""
    H, W, _ = img.shape
    h, s, v = rgb_to_hsv(img)
    hi = np.clip((h / 360 * hbins).astype(int), 0, hbins - 1)
    si = np.clip((s * sbins).astype(int), 0, sbins - 1)
    vi = np.clip((v * vbins).astype(int), 0, vbins - 1)
    idx = (hi * sbins + si) * vbins + vi  # flat bin index

    r0 = int((1.0 - seed_rows) * H)
    c0, c1 = int((0.5 - seed_w / 2) * W), int((0.5 + seed_w / 2) * W)
    seed = idx[r0:H, c0:c1].ravel()
    hist = np.bincount(seed, minlength=hbins * sbins * vbins).astype(float)
    hist /= hist.max() + 1e-9
    return hist[idx]


def _smooth(a, k):
    if k <= 1:
        return a
    ker = np.ones(k) / k
    return np.convolve(a, ker, mode="same")


def column_local_floor(img, like_global, dens, band=0.12, lo=0.6):
    """Augment the global floor mask: a pixel also counts as floor if it matches
    its OWN column's bottom-strip colour distribution. Handles spatially-varying
    floor (e.g. a dark rubber mat that the central seed misses)."""
    H, W, _ = img.shape
    h, s, v = rgb_to_hsv(img)
    # coarse per-pixel colour key for local matching
    key = (np.clip((h / 360 * 12).astype(int), 0, 11) * 4
           + np.clip((v * 4).astype(int), 0, 3))
    r0 = int((1.0 - band) * H)
    out = like_global >= dens
    cw = W / NCOLS
    for c in range(NCOLS):
        x0, x1 = int(c * cw), int((c + 1) * cw)
        seed = key[r0:H, x0:x1].ravel()
        hist = np.bincount(seed, minlength=12 * 4).astype(float)
        hist /= hist.max() + 1e-9
        local = hist[key[:, x0:x1]] >= 0.05
        out[:, x0:x1] |= local
    return out.astype(float)


def segment(img, dens=0.02, row_thr=0.55, run=4, smooth=5, col_smooth=3, close=6,
            local=True):
    """Per-column scan up from the bottom; boundary = first sustained non-floor
    run of `run` rows below `row_thr` floor fraction. Output profile is
    median-smoothed across columns to match the smooth hand-drawn line."""
    H, W, _ = img.shape
    like = floor_likelihood(img)
    if local:
        floor = column_local_floor(img, like, dens)
    else:
        floor = (like >= dens).astype(float)
    cols = np.zeros(NCOLS)
    cw = W / NCOLS
    for c in range(NCOLS):
        x0, x1 = int(c * cw), int((c + 1) * cw)
        frac = _smooth(floor[:, x0:x1].mean(axis=1), smooth)
        isfloor = frac >= row_thr
        # fill short non-floor gaps (texture/shadow specks) up to `close` rows
        if close > 0:
            y = 0
            while y < H:
                if not isfloor[y]:
                    j = y
                    while j < H and not isfloor[j]:
                        j += 1
                    lo_ok = y > 0 and isfloor[y - 1]
                    hi_ok = j < H and isfloor[j]
                    if (j - y) <= close and lo_ok and hi_ok:
                        isfloor[y:j] = True
                    y = j
                else:
                    y += 1
        boundary, bad = 0, 0
        for y in range(H - 1, -1, -1):
            if not isfloor[y]:
                bad += 1
                if bad >= run:
                    boundary = y + run
                    break
            else:
                bad = 0
        cols[c] = (H - boundary) / H
    if col_smooth > 1:
        pad = col_smooth // 2
        padded = np.pad(cols, pad, mode="edge")
        cols = np.array([np.median(padded[i:i + col_smooth]) for i in range(NCOLS)])
    return cols


def main():
    labels = sorted(glob.glob(f"{FIXT}/labels/*.json"))
    errs, hold_errs = [], []
    for lp in labels:
        j = json.load(open(lp))
        img = np.asarray(Image.open(f"{FIXT}/images/{j['image']}").convert("RGB"))
        pred = segment(img)
        truth = np.array(j["columns"])
        e = np.abs(pred - truth).mean()
        tag = "HOLD" if j.get("holdout") else "    "
        flag = "  <-- over tol" if e > j.get("tolerance", 0.1) else ""
        print(f"{tag} {os.path.basename(lp):20s} mae={e:.3f}{flag}")
        (hold_errs if j.get("holdout") else errs).append(e)
    print(f"\ntrain mean mae={np.mean(errs):.3f}  max={np.max(errs):.3f}")
    if hold_errs:
        print(f"holdout mean mae={np.mean(hold_errs):.3f}  max={np.max(hold_errs):.3f}")


if __name__ == "__main__":
    main()
