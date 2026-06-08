"""
vision.py — Shared vision, depth estimation, and odometry for RoombaI.

All inference is local: Hailo AI Hat+ (YOLOv8) for object detection,
OpenCV for door geometry and optical-flow depth estimation.

Depth from motion:
  Between two frames captured baseline_cm apart (from wheel odometry):
      scene_depth_cm = FOCAL_PX * baseline_cm / median_flow_px
  This uses the thin-lens / similar-triangles relation for a translating camera.

Door distance:
  From the apparent pixel width of the detected door opening and an assumed
  real-world width (standard interior door ≈ 80 cm):
      door_distance_cm = DOOR_WIDTH_CM * FOCAL_PX / door_pixel_width

Obstacle distance:
  From the apparent pixel height of a detected object and its real-world height:
      distance_cm = KNOWN_HEIGHTS_CM[class] * FOCAL_PX / pixel_height
  Falls back to scene_depth_cm from optical flow when class height is unknown.
"""

import math
import threading

import cv2
import numpy as np

# ── Camera intrinsics ────────────────────────────────────────────────────────
# Pi Camera v2/v3 at 640×480, horizontal FOV ≈ 66°
# focal_px = (width/2) / tan(FOV_h/2)  =  320 / tan(33°)  ≈  492
FOCAL_PX = 492.0

DOOR_WIDTH_CM = 80.0   # standard interior door

# Obstacle blocking threshold: only flag objects closer than this
OBSTACLE_BLOCK_DIST_CM = 200.0

# Stuck detection thresholds
STUCK_BASELINE_CM = 20.0
STUCK_FLOW_PX     = 3.0

# ── Known approximate heights for COCO classes (cm) ─────────────────────────
# Used to estimate distance from bounding-box pixel height:
#   distance_cm = known_height_cm * FOCAL_PX / pixel_height
KNOWN_HEIGHTS_CM: dict[str, float] = {
    "person":       170.0,
    "bicycle":      100.0,
    "car":          150.0,
    "motorcycle":   110.0,
    "bus":          300.0,
    "truck":        250.0,
    "bench":         80.0,
    "chair":         90.0,
    "couch":         85.0,
    "bed":           60.0,
    "dining table":  75.0,
    "toilet":        70.0,
    "tv":            60.0,
    "refrigerator": 170.0,
    "oven":          90.0,
    "sink":          90.0,
    "potted plant":  50.0,
    "vase":          30.0,
    "laptop":        25.0,
    "dog":           50.0,
    "cat":           25.0,
}

# ── COCO class list (YOLOv8-s, 80 classes) ──────────────────────────────────
COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]

# ── Hailo YOLO ───────────────────────────────────────────────────────────────
_hailo_model = None
_hailo_lock  = threading.Lock()


def init_hailo(hef_path: str = "/usr/share/hailo-models/yolov8s_h8l.hef") -> bool:
    """Load YOLOv8 onto the Hailo AI Hat+. Safe to call multiple times."""
    global _hailo_model
    with _hailo_lock:
        if _hailo_model is not None:
            return True
        try:
            from hailo_platform import (
                VDevice, HEF, ConfigureParams,
                InputVStreamParams, OutputVStreamParams,
                FormatType, HailoStreamInterface,
            )
            hef    = HEF(hef_path)
            target = VDevice()
            cfg    = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
            ngs    = target.configure(hef, cfg)
            ng     = ngs[0]
            in_p   = InputVStreamParams.make(ng, format_type=FormatType.UINT8)
            out_p  = OutputVStreamParams.make(ng, format_type=FormatType.FLOAT32)
            _hailo_model = dict(ng=ng, ng_params=ng.create_params(), in_p=in_p, out_p=out_p)
            return True
        except Exception as e:
            print(f"[hailo] init failed: {e}")
            return False


def run_yolo(img_bgr: np.ndarray) -> list[dict]:
    """
    Run YOLOv8 on img_bgr via the Hailo AI Hat+.

    Returns a list of detection dicts, each with:
      class_name   str    — COCO class label
      confidence   float  — detection confidence (0–1)
      bbox_px      tuple  — (x1, y1, x2, y2) in original image pixels
      distance_cm  float | None  — estimated distance from known object height
      position     str   — "left" | "center" | "right" (horizontal frame third)

    Hailo yolov8s_h8l.hef outputs post-NMS detections in rows of
    [y_min, x_min, y_max, x_max, score, class_id] normalised to [0, 1].
    Coordinates are scaled back to (orig_h, orig_w) before returning.
    """
    with _hailo_lock:
        m = _hailo_model
    if m is None:
        return []

    orig_h, orig_w = img_bgr.shape[:2]

    try:
        from hailo_platform import InferVStreams
        resized = cv2.resize(img_bgr, (640, 640))
        rgb     = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        with m["ng"].activate(m["ng_params"]):
            with InferVStreams(m["ng"], m["in_p"], m["out_p"]) as pipe:
                in_name = pipe.get_input_vstream_infos()[0].name
                with pipe.async_infer(
                    {in_name: np.expand_dims(rgb, 0).astype(np.uint8)}
                ) as job:
                    output = job.get()

        detections: list[dict] = []
        for arr in output.values():
            a = arr[0] if arr.ndim > 2 else arr
            if a.ndim != 2:
                continue
            for det in a:
                if len(det) < 6:
                    continue
                score = float(det[4])
                if score < 0.35:
                    continue
                cls_idx = int(det[5])
                if not (0 <= cls_idx < len(COCO_CLASSES)):
                    continue

                class_name = COCO_CLASSES[cls_idx]

                # Hailo yolov8 output: [y1, x1, y2, x2, score, class] normalised
                raw = det[0:4].tolist()
                # Handle both normalised (≤1) and already-pixel (>1) coordinates
                if max(raw) <= 1.0:
                    y1n, x1n, y2n, x2n = raw
                    x1 = int(x1n * orig_w)
                    y1 = int(y1n * orig_h)
                    x2 = int(x2n * orig_w)
                    y2 = int(y2n * orig_h)
                else:
                    # Pixel coords from 640×640 inference space → scale back
                    x1 = int(raw[1] / 640 * orig_w)
                    y1 = int(raw[0] / 640 * orig_h)
                    x2 = int(raw[3] / 640 * orig_w)
                    y2 = int(raw[2] / 640 * orig_h)

                # Clamp
                x1, x2 = max(0, x1), min(orig_w, x2)
                y1, y2 = max(0, y1), min(orig_h, y2)
                if x2 <= x1 or y2 <= y1:
                    continue

                pixel_h = y2 - y1
                real_h  = KNOWN_HEIGHTS_CM.get(class_name)
                dist_cm = round(real_h * FOCAL_PX / pixel_h, 1) if (real_h and pixel_h > 5) else None

                cx = (x1 + x2) / 2
                pos = (
                    "left"   if cx < orig_w / 3
                    else "right" if cx > 2 * orig_w / 3
                    else "center"
                )

                detections.append({
                    "class_name":  class_name,
                    "confidence":  round(score, 3),
                    "bbox_px":     (x1, y1, x2, y2),
                    "distance_cm": dist_cm,
                    "position":    pos,
                })

        return detections

    except Exception as e:
        print(f"[yolo] inference error: {e}")
        return []


def yolo_labels(detections: list[dict]) -> list[str]:
    """Extract unique class names from a run_yolo() result."""
    return list({d["class_name"] for d in detections})


# ── Obstacle analysis ─────────────────────────────────────────────────────────
def analyze_obstacles(
    detections: list[dict],
    frame_w: int,
    frame_h: int,
    scene_depth_cm: float | None = None,
) -> dict:
    """
    Map YOLO detections to a spatial obstacle picture.

    An obstacle blocks a region if:
      - Its horizontal centre falls in that third (left / center / right)
      - Its bottom edge is in the lower 60 % of the frame (at floor level)
      - Its estimated distance is < OBSTACLE_BLOCK_DIST_CM

    When a detection has no known-height distance, falls back to scene_depth_cm
    from optical flow (which represents the nearest forward surface).

    Returns dict:
      obstacles      list[dict]  — enriched detections (adds 'blocking' bool)
      blocked        dict        — {"left": bool, "center": bool, "right": bool}
      clear_path     str | None  — first unblocked third (prefer center > right > left)
      nearest_cm     float | None — distance to nearest blocking obstacle in any region
    """
    floor_y_threshold = frame_h * 0.4   # bottom 60 % of frame = floor level

    enriched: list[dict] = []
    blocked = {"left": False, "center": False, "right": False}
    nearest_cm: float | None = None

    for det in detections:
        x1, y1, x2, y2 = det["bbox_px"]
        bbox_bottom = y2

        dist = det.get("distance_cm")
        if dist is None:
            dist = scene_depth_cm   # fall back to flow-based scene depth

        at_floor   = bbox_bottom > floor_y_threshold
        within_range = dist is None or dist < OBSTACLE_BLOCK_DIST_CM
        blocking   = at_floor and within_range

        enriched.append({**det, "blocking": blocking, "distance_cm": dist})

        if blocking:
            region = det["position"]
            blocked[region] = True
            if dist is not None:
                if nearest_cm is None or dist < nearest_cm:
                    nearest_cm = dist

    # Choose clearest path: unblocked thirds in preference order
    clear_path: str | None = None
    for candidate in ("center", "right", "left"):
        if not blocked[candidate]:
            clear_path = candidate
            break

    return {
        "obstacles":  enriched,
        "blocked":    blocked,
        "clear_path": clear_path,
        "nearest_cm": nearest_cm,
    }


# ── Door detection (OpenCV geometry) ─────────────────────────────────────────
def detect_door_cv(img_bgr: np.ndarray) -> dict:
    """
    Detect a doorway by finding two near-vertical line clusters (door pillars)
    with a wide, low-edge-density, bright gap between them.

    Returns a dict with:
      door_visible          bool
      door_open             bool   (gap brightness ≥ 82 % of surroundings)
      door_position         "left" | "center" | "right" | null
      in_doorway            bool   (pillars span ≥ 82 % of frame width)
      door_pixel_width      int    (width of gap in pixels, 0 if not visible)
      door_distance_cm      float  (estimated distance using DOOR_WIDTH_CM, None if unknown)
      rounded_column_visible bool
      confidence            float  (0–1)
      notes                 str
    """
    null: dict = {
        "door_visible": False, "door_open": False, "door_position": None,
        "in_doorway": False, "door_pixel_width": 0, "door_distance_cm": None,
        "rounded_column_visible": False, "confidence": 0.0, "notes": "no door",
    }
    try:
        h, w  = img_bgr.shape[:2]
        gray  = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 30, 100, apertureSize=3)

        lines = cv2.HoughLinesP(
            edges, rho=1, theta=np.pi / 180, threshold=50,
            minLineLength=int(h * 0.25), maxLineGap=25,
        )
        vert_xs: list[int] = []
        if lines is not None:
            for (x1, y1, x2, y2), in lines:
                dx, dy = abs(x2 - x1), abs(y2 - y1)
                if dy > 0 and dx / dy < 0.25:
                    vert_xs.append((x1 + x2) // 2)

        if len(vert_xs) < 2:
            return null

        vert_xs.sort()
        clusters: list[int] = []
        group = [vert_xs[0]]
        for x in vert_xs[1:]:
            if x - group[-1] < 40:
                group.append(x)
            else:
                clusters.append(int(np.mean(group)))
                group = [x]
        clusters.append(int(np.mean(group)))

        if len(clusters) < 2:
            return null

        best: tuple | None = None
        best_score = 0.0
        for i in range(len(clusters)):
            for j in range(i + 1, len(clusters)):
                lx, rx = clusters[i], clusters[j]
                gap    = rx - lx
                if not (0.15 * w < gap < 0.70 * w):
                    continue

                edge_dens  = float(edges[:, lx:rx].mean()) / 255.0
                gap_bright = float(gray[:, lx:rx].mean())
                sl = gray[:, max(0, lx - gap // 3):lx]
                sr = gray[:, rx:min(w, rx + gap // 3)]
                sur_bright = float(
                    np.concatenate([sl, sr], axis=1).mean()
                ) if sl.size + sr.size else gap_bright
                bright_r = gap_bright / max(sur_bright, 1.0)

                score = gap * (1.0 - edge_dens * 4.0) * max(bright_r, 0.4)
                if score > best_score:
                    best_score = score
                    best = (lx, rx, gap_bright, sur_bright, edge_dens)

        if best is None or best_score < 15.0:
            return null

        lx, rx, gap_bright, sur_bright, edge_dens = best
        gap      = rx - lx
        center_x = (lx + rx) / 2.0
        bright_r = gap_bright / max(sur_bright, 1.0)

        pos = (
            "left"   if center_x < w / 3
            else "right" if center_x > 2 * w / 3
            else "center"
        )
        in_doorway = lx < w * 0.18 and rx > w * 0.82
        door_open  = bright_r >= 0.82

        door_dist = round(DOOR_WIDTH_CM * FOCAL_PX / gap, 1) if gap > 0 else None

        circles = cv2.HoughCircles(
            blur, cv2.HOUGH_GRADIENT, dp=1.2,
            minDist=int(h * 0.25), param1=80, param2=30,
            minRadius=int(h * 0.06), maxRadius=int(h * 0.35),
        )
        rounded_col = circles is not None and len(circles[0]) > 0

        confidence = min(1.0, best_score / 500.0)
        notes = (
            f"gap={gap}px dist≈{door_dist}cm pos={pos} "
            f"bright={bright_r:.2f} edge_dens={edge_dens:.2f}"
        )
        return {
            "door_visible":           True,
            "door_open":              door_open,
            "door_position":          pos,
            "in_doorway":             in_doorway,
            "door_pixel_width":       gap,
            "door_distance_cm":       door_dist,
            "rounded_column_visible": rounded_col,
            "confidence":             confidence,
            "notes":                  notes,
        }
    except Exception as e:
        return {**null, "notes": f"cv error: {e}"}


# ── Optical flow depth & stuck detection ─────────────────────────────────────
def analyze_motion(
    img1_bgr: np.ndarray,
    img2_bgr: np.ndarray,
    baseline_cm: float,
) -> dict:
    """
    Estimate scene depth and detect whether the robot is stuck, using optical
    flow between two consecutive frames and the wheel-odometry baseline.

    Physics:
      A scene point at depth D (cm) from the camera produces a pixel
      displacement of magnitude:
          flow_px ≈ FOCAL_PX × baseline_cm / D
      Rearranged:
          D ≈ FOCAL_PX × baseline_cm / flow_px

    Returns dict:
      scene_depth_cm  float | None  — estimated depth to nearest forward surface
      flow_px         float         — median flow magnitude in central ROI
      stuck           bool          — commanded motion with near-zero flow
    """
    result: dict = {"scene_depth_cm": None, "flow_px": 0.0, "stuck": False}
    if baseline_cm < 1.0:
        return result
    try:
        g1 = cv2.cvtColor(img1_bgr, cv2.COLOR_BGR2GRAY)
        g2 = cv2.cvtColor(img2_bgr, cv2.COLOR_BGR2GRAY)

        flow = cv2.calcOpticalFlowFarneback(
            g1, g2, None,
            pyr_scale=0.5, levels=3, winsize=15,
            iterations=3, poly_n=5, poly_sigma=1.2, flags=0,
        )

        h, w = g1.shape
        m = min(h, w) // 6
        cy, cx = h // 2, w // 2
        roi = flow[cy - m:cy + m, cx - m:cx + m]
        mag = np.sqrt(roi[..., 0] ** 2 + roi[..., 1] ** 2)

        valid = mag[mag > 1.5]
        if valid.size < 20:
            result["stuck"] = baseline_cm > STUCK_BASELINE_CM
            return result

        median_flow = float(np.median(valid))
        result["flow_px"] = round(median_flow, 2)
        if median_flow > 0.5:
            result["scene_depth_cm"] = round(FOCAL_PX * baseline_cm / median_flow, 1)
        result["stuck"] = baseline_cm > STUCK_BASELINE_CM and median_flow < STUCK_FLOW_PX
        return result
    except Exception as e:
        result["notes"] = f"flow error: {e}"
        return result


# ── Odometry tracker ──────────────────────────────────────────────────────────
class OdometryTracker:
    """
    Dead-reckoning position estimate from commanded wheel moves.

    Coordinate frame:
      x — forward at start  (cm)
      y — left at start     (cm)
      heading — CCW positive (degrees), 0 = initial forward direction

    Thread-safe.
    """

    def __init__(self):
        self._lock    = threading.Lock()
        self._x       = 0.0
        self._y       = 0.0
        self._heading = 0.0

    def forward(self, speed_cm_s: float, duration_s: float):
        dist = speed_cm_s * duration_s
        with self._lock:
            rad        = math.radians(self._heading)
            self._x   += dist * math.cos(rad)
            self._y   += dist * math.sin(rad)

    def turn(self, deg: float):
        with self._lock:
            self._heading = (self._heading + deg) % 360

    @property
    def x(self) -> float:
        with self._lock:
            return self._x

    @property
    def y(self) -> float:
        with self._lock:
            return self._y

    @property
    def heading(self) -> float:
        with self._lock:
            return self._heading

    def distance_from_origin(self) -> float:
        with self._lock:
            return math.hypot(self._x, self._y)

    def snapshot(self) -> tuple[float, float, float]:
        with self._lock:
            return (self._x, self._y, self._heading)


# ── Calibration helper ────────────────────────────────────────────────────────
def calibrate_focal(
    known_object_width_cm: float,
    measured_pixel_width: int,
    known_distance_cm: float,
) -> float:
    """
    Compute focal length from a single measurement of a known object.

        f = calibrate_focal(30, measured_pixels, 100)
        # Set FOCAL_PX = f in this file
    """
    return (measured_pixel_width * known_distance_cm) / known_object_width_cm
