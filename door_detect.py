#!/usr/bin/env python3
"""
door_detect.py — detect open doorway in an image.

Usage: python3 door_detect.py <image_path>
Exit code 0 = DOOR DETECTED
Exit code 1 = no door
"""

import sys
import cv2
import numpy as np

def detect_door(image_path):
    img = cv2.imread(image_path)
    if img is None:
        print(f"ERROR: Cannot load image: {image_path}")
        sys.exit(2)

    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # --- Method 1: Low-edge-density vertical region (doorway = absence of walls) ---
    edges = cv2.Canny(gray, 50, 150)

    # Scan vertical strips of width ~5% of frame
    strip_w = max(1, w // 20)
    strip_scores = []
    for x in range(0, w - strip_w, strip_w):
        strip = edges[:, x:x + strip_w]
        density = np.sum(strip > 0) / (strip.shape[0] * strip.shape[1])
        strip_scores.append((x, density))

    # Find the longest consecutive run of low-density strips (< 10% edge density)
    LOW_THRESH = 0.10
    best_start = 0
    best_len = 0
    cur_start = None
    cur_len = 0
    for i, (x, d) in enumerate(strip_scores):
        if d < LOW_THRESH:
            if cur_start is None:
                cur_start = i
                cur_len = 1
            else:
                cur_len += 1
            if cur_len > best_len:
                best_len = cur_len
                best_start = cur_start
        else:
            cur_start = None
            cur_len = 0

    low_density_width = best_len * strip_w
    low_density_frac = low_density_width / w

    # --- Method 2: Bright horizontal band in lower portion ---
    # A doorway to a lit corridor shows a bright band near the floor line
    lower_half = gray[h // 2:, :]
    bright_mask = lower_half > 180
    bright_frac = np.sum(bright_mask) / bright_mask.size

    # --- Method 3: Large uniform (low-variance) vertical region ---
    # Compute column-wise variance
    col_var = np.var(gray.astype(float), axis=0)
    # Smooth
    col_var_smooth = np.convolve(col_var, np.ones(strip_w) / strip_w, mode='same')
    low_var_cols = col_var_smooth < 200
    # Find longest run
    best_var_run = 0
    cur_run = 0
    for v in low_var_cols:
        if v:
            cur_run += 1
            best_var_run = max(best_var_run, cur_run)
        else:
            cur_run = 0
    low_var_frac = best_var_run / w

    # --- Decision ---
    # Primary: low edge density region wide enough and tall enough
    # We use full image height as proxy for "vertical" since the strip goes full height
    door_by_edges = (low_density_frac > 0.15)
    door_by_bright = (bright_frac > 0.25)
    door_by_variance = (low_var_frac > 0.15)

    # Compute centroid x of best low-density region for steering
    centroid_x = (best_start * strip_w + low_density_width / 2) if best_len > 0 else w // 2
    side = "LEFT" if centroid_x < w / 2 else "RIGHT"
    offset_pct = abs(centroid_x - w / 2) / (w / 2) * 100

    score = sum([door_by_edges, door_by_bright, door_by_variance])

    print(f"Image: {image_path}  ({w}x{h})")
    print(f"  Edge low-density region: {low_density_frac*100:.1f}% of width  -> door_by_edges={door_by_edges}")
    print(f"  Bright lower-half frac:  {bright_frac*100:.1f}%               -> door_by_bright={door_by_bright}")
    print(f"  Low-variance col run:    {low_var_frac*100:.1f}% of width     -> door_by_variance={door_by_variance}")
    print(f"  Score: {score}/3  |  Candidate centroid: x={centroid_x:.0f} ({side}, {offset_pct:.0f}% from center)")

    if score >= 2:
        print("DECISION: DOOR DETECTED")
        print(f"DOOR_SIDE={side}")
        print(f"DOOR_OFFSET_PCT={offset_pct:.0f}")
        sys.exit(0)
    else:
        print("DECISION: no door")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: door_detect.py <image_path>")
        sys.exit(2)
    detect_door(sys.argv[1])
