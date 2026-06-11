"""
vision.py — Multi-model vision pipeline for RoombaI.

Models running on this Pi:
  yolo_det   YOLOv8s detection  (Hailo AI Hat+ H8L)  COCO 80-class boxes + distance
  fast_depth fast-depth ONNX    (CPU / onnxruntime)   per-pixel metric depth
  opencv     OpenCV geometry    (CPU)                  door pillar-gap detector
  flow       Farneback flow     (CPU)                  between-frame depth + stuck

Main entry point: analyze_scene(img_bgr, prev_img, baseline_cm) → scene dict.
"""

import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np

# ── fast_depth ONNX path (CPU inference via onnxruntime) ─────────────────────
_FAST_DEPTH_ONNX = Path.home() / ".cache" / "fast_depth" / "fastdepth.onnx"
_fast_depth_session = None
_fast_depth_input_name: str = "input.1"   # queried at load time

# ── door_yolo ONNX path (CPU inference via onnxruntime) ──────────────────────
# YOLOv8s fine-tuned for door detection, single class "door".
# Input: 640×640 normalised RGB; output: [1, 5, 8400] (cx,cy,w,h,score).
_DOOR_YOLO_ONNX  = Path.home() / ".cache" / "door_yolo" / "doors.onnx"
_door_yolo_session = None
_DOOR_YOLO_SIZE  = 640

# ── Camera intrinsics ─────────────────────────────────────────────────────────
# IMX708 full-sensor 2304×1296, downscaled to 640×480 for inference.
# hFOV = 66° → FOCAL_PX = (640/2) / tan(33°) = 492 px  (horizontal axis, correct)
# vFOV = 40.1° (from 16:9 sensor geometry, ±20.1° from horizontal)
# Camera tilt: 0° (horizontal).  Blind spot in front of robot ≈ 43 cm.
FOCAL_PX          = 492.0
FOCAL_PX_V        = 655.0    # vertical focal length in 640×480 (stretched from 16:9→4:3)
CAMERA_TILT_DEG   = 0.0      # degrees above horizontal (positive = tilted up)
DOOR_WIDTH_CM     = 80.0
CAMERA_HEIGHT_CM  = 20.0     # camera is ~20 cm off the floor
ROBOT_WIDTH_CM    = 40.0     # Roomba diameter ≈ 2 × camera height
OBSTACLE_BLOCK_DIST_CM = 200.0
# Chair leg detection (thin near-vertical structures in the floor region)
# Chairs are only treated as blocking when closer than this — allows the robot
# to navigate a 60-80 cm corridor with chairs on one side (robot ⌀ 40 cm).
CHAIR_BLOCK_DIST_CM = 55.0  # cm — robot radius 20 + 35 cm buffer
CHAIR_BBOX_EXPAND = 0.05    # small margin for localisation uncertainty only
                             # (was 0.25 — that made one chair block the full frame)
# Classes whose ground footprint must be respected — but only when close enough.
FLOOR_BLOCKER_CLASSES = {"chair", "couch", "dining table", "bench"}
LEG_FLOOR_FRAC    = 0.55    # analyse bottom LEG_FLOOR_FRAC of frame for legs
LEG_MIN_LENGTH    = 0.22    # min leg segment as fraction of floor-region height
LEG_MAX_SLOPE     = 0.15    # max |dx/dy| to count as near-vertical
LEG_MIN_CLUSTER   = 2       # min Hough segments per cluster
CHAIR_PAIR_MAX_CM = 80.0    # real-world gap below which two legs are treated as the same
                             # chair; the invisible floor bar means the gap is also blocked
LEG_MAX_DIST_CM   = 200.0   # don't block navigation for legs further than this
LEG_NO_DIST_BLOCK_CM = 80.0 # assumed distance when no depth estimate available
STUCK_BASELINE_CM = 20.0
STUCK_FLOW_PX     = 3.0
TEXTURE_MIN_VAR   = 50.0   # Laplacian variance below which a region is treated as textureless
                            # (blank wall / whiteboard). open_space is scaled by var/TEXTURE_MIN_VAR
                            # so a wall with var=10 gives confidence≈0.20, suppressing false "open".
                            # Lowered from 80→50: var≈40 (corridor floor) now gets confidence≈0.80
                            # instead of 0.50, reducing over-suppression of real open space.

# ── Blind-spot / obstacle memory constants ────────────────────────────────────
BLIND_SPOT_CM     = 43.0   # closest floor point visible from camera (horizontal, 20cm height)
MEMORY_TIMEOUT_S  = 25.0   # forget obstacles not refreshed within this time
MEMORY_SAFETY_CM  = 20.0   # extra buffer beyond blind spot edge before expiring forward
ROOMBA_RADIUS_CM  = 20.0   # half the Roomba diameter — obstacle is "passed" once behind this


class ObstacleMemory:
    """
    Tracks recently detected obstacles in robot-local 2D coordinates so they
    remain "visible" even after they enter the camera blind spot (~43 cm in
    front of the robot where the floor is no longer in frame).

    Coordinate system (robot frame):
      d_fwd  — distance forward from robot centre (cm); positive = in front
      d_lat  — lateral offset from robot centre (cm); positive = right of robot

    After each movement the caller must call update_forward() / update_turn()
    so positions stay accurate.  Obstacles expire once clearly passed or stale.
    """

    def __init__(self) -> None:
        self._obs: list[dict] = []
        self._lock = threading.Lock()
        self._blind_spot_cm: float = BLIND_SPOT_CM  # updated each frame from y_horizon

    def set_blind_spot(self, cm: float) -> None:
        """Update the blind-spot distance (called by the camera thread each frame)."""
        with self._lock:
            self._blind_spot_cm = max(5.0, min(400.0, cm))

    def add(self, class_name: str, distance_cm: float, lateral_cm: float,
            conf: float = 1.0) -> None:
        """Record a detected obstacle.  Merges with a nearby existing entry of
        the same class rather than duplicating (within 30 cm in both axes)."""
        now = time.time()
        with self._lock:
            for obs in self._obs:
                if (obs["class_name"] == class_name
                        and abs(obs["d_fwd"] - distance_cm) < 30
                        and abs(obs["d_lat"] - lateral_cm) < 30):
                    obs["d_fwd"] = distance_cm
                    obs["d_lat"] = lateral_cm
                    obs["conf"]  = conf
                    obs["ts"]    = now
                    return
            self._obs.append({
                "class_name": class_name,
                "d_fwd":      distance_cm,
                "d_lat":      lateral_cm,
                "conf":       conf,
                "ts":         now,
            })

    def update_forward(self, cm: float) -> None:
        """Robot moved forward cm — reduce all forward distances accordingly."""
        with self._lock:
            for obs in self._obs:
                obs["d_fwd"] -= cm

    def update_turn(self, deg: float) -> None:
        """Robot turned deg (+CCW/left, -CW/right) — rotate all positions.

        When the robot turns CCW (left) by θ, every obstacle appears to shift
        clockwise in the robot frame:
            new_d_fwd = d_fwd·cos θ  −  d_lat·sin θ
            new_d_lat = d_fwd·sin θ  +  d_lat·cos θ
        """
        theta = math.radians(deg)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        with self._lock:
            for obs in self._obs:
                d, x = obs["d_fwd"], obs["d_lat"]
                obs["d_fwd"] = d * cos_t - x * sin_t
                obs["d_lat"] = d * sin_t + x * cos_t

    def expire(self) -> None:
        """Remove obstacles that have been passed or not refreshed recently."""
        now = time.time()
        with self._lock:
            self._obs = [
                o for o in self._obs
                if o["d_fwd"] > -ROOMBA_RADIUS_CM          # not yet passed
                and now - o["ts"] < MEMORY_TIMEOUT_S       # not stale
            ]

    def clear(self) -> None:
        """Discard all stored obstacles (e.g. after a bump clears the path)."""
        with self._lock:
            self._obs.clear()

    def get_blocking(self) -> dict:
        """Return blocking info for obstacles within or near the blind spot.

        Only reports obstacles whose d_fwd is within BLIND_SPOT_CM + MEMORY_SAFETY_CM
        (i.e. either in the blind spot already, or close enough that they will enter
        it before the next camera frame can re-detect them).

        Returns a dict compatible with analyze_obstacles output:
            blocked     — {left, center, right}
            nearest_cm  — closest obstacle d_fwd, or None
            obstacles   — list of raw memory entries (for logging)
        """
        blocked: dict[str, bool] = {"left": False, "center": False, "right": False}
        nearest_cm: float | None = None
        in_blind: list[dict] = []

        concern_dist = self._blind_spot_cm + MEMORY_SAFETY_CM
        robot_half   = ROBOT_WIDTH_CM / 2.0

        with self._lock:
            for obs in self._obs:
                d = obs["d_fwd"]
                if d <= 0 or d > concern_dist:
                    continue  # behind robot, or still clearly visible to camera
                x = obs["d_lat"]
                if abs(x) > ROBOT_WIDTH_CM * 1.5:
                    continue  # well to the side — not in travel path

                side = ("right" if x >  robot_half / 2
                        else "left"  if x < -robot_half / 2
                        else "center")
                blocked[side] = True
                if nearest_cm is None or d < nearest_cm:
                    nearest_cm = d
                in_blind.append(dict(obs))

        return {"blocked": blocked, "nearest_cm": nearest_cm, "obstacles": in_blind}

    def __len__(self) -> int:
        with self._lock:
            return len(self._obs)


# ── Hailo model specs ─────────────────────────────────────────────────────────
_MDIR = Path("/usr/share/hailo-models")

MODEL_SPECS: dict[str, dict] = {
    "yolo_det": {
        "hef":      _MDIR / "yolov8s_h8l.hef",
        "input_wh": (640, 640),
        "kind":     "yolo_det",
        "dataset":  "",
    },
}

# ── COCO class list (YOLOv8, 80 classes) ─────────────────────────────────────
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

KNOWN_HEIGHTS_CM: dict[str, float] = {
    "person": 170.0, "bicycle": 100.0, "car": 150.0, "motorcycle": 110.0,
    "bus": 300.0, "truck": 250.0, "bench": 80.0, "chair": 90.0,
    "couch": 85.0, "bed": 60.0, "dining table": 75.0, "toilet": 70.0,
    "tv": 60.0, "refrigerator": 170.0, "oven": 90.0, "sink": 90.0,
    "potted plant": 50.0, "vase": 30.0, "laptop": 25.0, "dog": 50.0,
    "cat": 25.0,
}

# Typical object widths (cm) — complement to heights; used when width is
# the more reliably visible dimension (e.g. wide objects, partial occlusion).
KNOWN_WIDTHS_CM: dict[str, float] = {
    "person": 50.0, "bicycle": 60.0, "chair": 55.0, "couch": 190.0,
    "dining table": 120.0, "bench": 120.0, "car": 185.0, "tv": 80.0,
    "refrigerator": 70.0, "bed": 140.0, "toilet": 40.0, "laptop": 35.0,
}

# Smoothed horizon y-coordinate (pixels), updated per frame via EMA.
# Represents the vanishing line of the floor plane.
_y_horizon_ema: float | None = None
_HORIZON_EMA_ALPHA = 0.10   # slow update — horizon is stable across frames

# ── Hailo multi-model registry ────────────────────────────────────────────────
_hailo_lock = threading.Lock()
_hailo_vdev = None                       # one VDevice shared across all models
_hailo_reg: dict[str, dict] = {}        # name → loaded model state
MODELS_AVAILABLE: dict[str, bool] = {k: False for k in MODEL_SPECS} | {"fast_depth": False, "door_yolo": False}


def _get_vdevice():
    global _hailo_vdev
    if _hailo_vdev is None:
        from hailo_platform import VDevice
        _hailo_vdev = VDevice()
    return _hailo_vdev


def _load_model(name: str) -> bool:
    spec = MODEL_SPECS[name]
    hef_path = Path(spec["hef"])
    if not hef_path.exists():
        print(f"[vision] {name}: HEF not found — {hef_path}")
        return False
    try:
        from hailo_platform import (
            HEF, ConfigureParams, InputVStreamParams,
            OutputVStreamParams, FormatType, HailoStreamInterface,
        )
        vdev  = _get_vdevice()
        hef   = HEF(str(hef_path))
        cfg   = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
        ngs   = vdev.configure(hef, cfg)
        ng    = ngs[0]
        in_p  = InputVStreamParams.make(ng, format_type=FormatType.UINT8)
        out_p = OutputVStreamParams.make(ng, format_type=FormatType.FLOAT32)

        # Resolve the input stream name once at load time so _hailo_infer never
        # calls get_input_vstream_infos() on the InferVStreams pipe — that method
        # was removed in some SDK versions (the ng object always has it).
        try:
            in_name = ng.get_input_vstream_infos()[0].name
        except Exception:
            # Older SDK: name may be accessible directly from in_p
            if isinstance(in_p, dict):
                in_name = next(iter(in_p))
            elif isinstance(in_p, list):
                in_name = in_p[0].name
            else:
                in_name = getattr(in_p, "name", None)

        _hailo_reg[name] = {
            "ng":       ng,
            "params":   ng.create_params(),
            "in_p":     in_p,
            "out_p":    out_p,
            "in_name":  in_name,
            "input_wh": spec["input_wh"],
            "kind":     spec["kind"],
            "dataset":  spec.get("dataset", ""),
        }
        MODELS_AVAILABLE[name] = True
        print(f"[vision] {name}: loaded  ({hef_path.name})")
        return True
    except Exception as e:
        print(f"[vision] {name}: init failed — {e}")
        return False


def init_all_models() -> dict[str, bool]:
    """Load every model in MODEL_SPECS plus fast_depth and door_yolo ONNX. Call once at startup."""
    with _hailo_lock:
        for name in MODEL_SPECS:
            if not MODELS_AVAILABLE[name]:
                _load_model(name)
        _init_fast_depth()
        _init_door_yolo()
    return dict(MODELS_AVAILABLE)


def init_hailo(hef_path: str = "") -> bool:
    """Legacy alias — loads all models, returns True if yolo_det succeeded."""
    return init_all_models().get("yolo_det", False)


def _init_fast_depth() -> bool:
    """Load fast_depth ONNX session. Returns True on success."""
    global _fast_depth_session, _fast_depth_input_name
    if not _FAST_DEPTH_ONNX.exists():
        print(f"[vision] fast_depth: ONNX not found at {_FAST_DEPTH_ONNX} — run ensure_models() first")
        return False
    try:
        import onnxruntime as ort
        sess = ort.InferenceSession(str(_FAST_DEPTH_ONNX), providers=["CPUExecutionProvider"])
        _fast_depth_input_name = sess.get_inputs()[0].name
        _fast_depth_session = sess
        MODELS_AVAILABLE["fast_depth"] = True
        print(f"[vision] fast_depth: loaded (ONNX/CPU, input={_fast_depth_input_name})")
        return True
    except Exception as e:
        print(f"[vision] fast_depth: init failed — {e}")
        return False


def _infer_fast_depth(img_bgr: np.ndarray) -> np.ndarray | None:
    """
    Run fast_depth ONNX on CPU.
    Returns H×W float32 metric depth map in metres, or None if unavailable.
    Input is BGR uint8; model expects RGB float32 / 255 in NCHW format.
    """
    if _fast_depth_session is None:
        return None
    try:
        resized = cv2.resize(img_bgr, (224, 224))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        inp = np.transpose(rgb, (2, 0, 1))[np.newaxis]          # (1, 3, 224, 224)
        result = _fast_depth_session.run(None, {_fast_depth_input_name: inp})
        return result[0][0, 0]                                    # (H, W) in metres
    except Exception as e:
        print(f"[vision] fast_depth infer error: {e}")
        return None


# ── door_yolo ONNX (door panel detector) ─────────────────────────────────────

def _init_door_yolo() -> bool:
    """Load the door_yolo ONNX session. Returns True on success."""
    global _door_yolo_session
    if not _DOOR_YOLO_ONNX.exists():
        print(f"[vision] door_yolo: ONNX not found at {_DOOR_YOLO_ONNX} — run ensure_models() first")
        return False
    try:
        import onnxruntime as ort
        sess = ort.InferenceSession(str(_DOOR_YOLO_ONNX), providers=["CPUExecutionProvider"])
        _door_yolo_session = sess
        MODELS_AVAILABLE["door_yolo"] = True
        print(f"[vision] door_yolo: loaded (ONNX/CPU, input={_DOOR_YOLO_SIZE}×{_DOOR_YOLO_SIZE})")
        return True
    except Exception as e:
        print(f"[vision] door_yolo: init failed — {e}")
        return False


def _infer_door_yolo(img_bgr: np.ndarray, conf_thresh: float = 0.30) -> list[dict]:
    """
    Run the door_yolo ONNX model on CPU.

    Returns a list of door-panel detections, each:
      {"bbox_px": (x1,y1,x2,y2), "confidence": float, "position": "left"|"center"|"right"}

    YOLOv8 ONNX output shape: [1, 5, 8400]  (cx, cy, w, h, score) for single class.
    Coords may be in model pixel space [0, 640] or normalised [0, 1] depending on
    the export version — both are handled.
    """
    if _door_yolo_session is None:
        return []

    orig_h, orig_w = img_bgr.shape[:2]
    sz = _DOOR_YOLO_SIZE

    # Preprocess: BGR → RGB, resize, normalise, NCHW float32
    rgb     = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (sz, sz))
    inp     = np.transpose(resized.astype(np.float32) / 255.0, (2, 0, 1))[np.newaxis]

    try:
        input_name = _door_yolo_session.get_inputs()[0].name
        raw_out    = _door_yolo_session.run(None, {input_name: inp})
    except Exception as e:
        print(f"[vision] door_yolo infer error: {e}")
        return []

    # raw_out[0]: [1, 5, 8400] → squeeze → [5, 8400] → transpose → [8400, 5]
    preds = raw_out[0][0].T      # [8400, 5]: cx, cy, w, h, score

    mask  = preds[:, 4] >= conf_thresh
    preds = preds[mask]
    if len(preds) == 0:
        return []

    cx, cy, w, h = preds[:, 0], preds[:, 1], preds[:, 2], preds[:, 3]

    # Normalise coords to model pixel space if they came out as fractions
    if cx.max() <= 1.01:
        cx, cy, w, h = cx * sz, cy * sz, w * sz, h * sz

    x1 = np.clip(cx - w / 2, 0, sz).astype(int)
    y1 = np.clip(cy - h / 2, 0, sz).astype(int)
    x2 = np.clip(cx + w / 2, 0, sz).astype(int)
    y2 = np.clip(cy + h / 2, 0, sz).astype(int)

    # NMS in model-pixel space
    boxes_xywh = np.stack([x1, y1, x2 - x1, y2 - y1], axis=1).tolist()
    scores     = preds[:, 4].tolist()
    indices    = cv2.dnn.NMSBoxes(boxes_xywh, scores, conf_thresh, nms_threshold=0.45)
    if len(indices) == 0:
        return []
    indices = indices.flatten() if hasattr(indices, "flatten") else list(indices)

    # Scale bboxes back to original image size
    sx, sy = orig_w / sz, orig_h / sz
    detections: list[dict] = []
    for i in indices:
        px1, py1 = int(x1[i] * sx), int(y1[i] * sy)
        px2, py2 = int(x2[i] * sx), int(y2[i] * sy)
        cx_orig  = (px1 + px2) / 2.0
        pos      = ("left"   if cx_orig < orig_w / 3
                    else "right" if cx_orig > 2 * orig_w / 3
                    else "center")
        detections.append({
            "bbox_px":    (px1, py1, px2, py2),
            "confidence": round(float(preds[i, 4]), 3),
            "position":   pos,
        })

    return detections


def _fuse_door_detections(cv_door: dict, yolo_doors: list[dict], orig_w: int) -> dict:
    """
    Fuse OpenCV geometric door detection with YOLO door-panel detections.

    Roles:
      door_yolo  detects the *wooden door panel* (solid surface; works when
                 the door is closed or only partially open).
      OpenCV     detects the *gap / frame* in the glass wall (works best when
                 the door is open and the opening is the bright region).

    Decision table:
      Both agree  → boost confidence; keep OpenCV's open/position assessment
                    (it has better spatial precision for navigation).
      YOLO only   → door panel visible but no navigable gap yet; mark closed.
      OpenCV only → gap detected, panel swung out of frame; door is open.
      Neither     → no door.
    """
    if not cv_door["door_visible"] and not yolo_doors:
        return cv_door

    if yolo_doors and not cv_door["door_visible"]:
        best = max(yolo_doors, key=lambda d: d["confidence"])
        return {
            **cv_door,
            "door_visible":  True,
            "door_open":     False,
            "door_position": best["position"],
            "confidence":    round(best["confidence"] * 0.7, 3),
            "notes": (
                f"YOLO door panel at {best['position']} "
                f"(conf={best['confidence']:.2f}); no geometric gap yet"
            ),
        }

    if cv_door["door_visible"] and yolo_doors:
        best     = max(yolo_doors, key=lambda d: d["confidence"])
        boosted  = round(min(1.0, cv_door["confidence"] + best["confidence"] * 0.25), 3)
        return {
            **cv_door,
            "confidence": boosted,
            "notes": cv_door["notes"] + f" | YOLO panel {best['position']} ({best['confidence']:.2f})",
        }

    # OpenCV only (door fully open, panel swung clear of frame) — return as-is
    return cv_door


# ── Raw Hailo inference ───────────────────────────────────────────────────────
def _normalise_hailo_output(v) -> np.ndarray:
    """
    Convert a single Hailo output value to a numpy array.

    pipe.infer() returns Python lists.  Two formats are observed:

    Homogeneous  — v is a nested list that np.array() can convert directly
                   (e.g. shape (1, N, 6) flat detection output).

    Class-grouped NMS — v has outer shape (1, 80) where each of the 80 entries
                   is a variable-length list of detections for that class.
                   np.array(v) fails with "inhomogeneous shape".
                   We flatten to (1, N, 6) by appending class_idx as column 6.
    """
    if isinstance(v, np.ndarray):
        return v
    try:
        return np.array(v, dtype=np.float32)
    except ValueError:
        # Class-grouped NMS: v[batch][class_idx] = list of detections
        flat: list[list[float]] = []
        try:
            batch0 = v[0]   # drop batch dimension
            for cls_idx, class_dets in enumerate(batch0):
                for det in (class_dets or []):
                    row = [float(x) for x in det]
                    if len(row) >= 5:
                        flat.append(row[:5] + [float(cls_idx)])
        except Exception:
            pass
        arr = np.array(flat, dtype=np.float32) if flat else np.zeros((0, 6), dtype=np.float32)
        return arr[np.newaxis]   # (1, N, 6)


def _hailo_infer(name: str, img_bgr: np.ndarray) -> dict[str, np.ndarray] | None:
    """Resize, preprocess, infer on named model. Returns raw output stream dict."""
    m = _hailo_reg.get(name)
    if m is None:
        return None
    iw, ih = m["input_wh"]
    try:
        from hailo_platform import InferVStreams
        resized = cv2.resize(img_bgr, (iw, ih))
        rgb     = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        inp     = np.expand_dims(rgb, 0).astype(np.uint8)
        with m["ng"].activate(m["params"]):
            with InferVStreams(m["ng"], m["in_p"], m["out_p"]) as pipe:
                raw = pipe.infer({m["in_name"]: inp})
                return {k: _normalise_hailo_output(v) for k, v in raw.items()}
    except Exception as e:
        print(f"[vision] {name} infer error: {e}")
        return None


# ── Optical-flow depth ────────────────────────────────────────────────────────
def object_depth_from_flow(
    flow_field: np.ndarray,
    bbox_px: tuple[int, int, int, int],
    baseline_cm: float,
) -> float | None:
    """Per-object depth from the optical-flow field within its bounding box."""
    if baseline_cm < 1.0 or flow_field is None:
        return None
    fh, fw = flow_field.shape[:2]
    x1, y1, x2, y2 = bbox_px
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(fw, x2), min(fh, y2)
    if x2 <= x1 or y2 <= y1:
        return None
    mag   = np.sqrt(flow_field[y1:y2, x1:x2, 0] ** 2 + flow_field[y1:y2, x1:x2, 1] ** 2)
    valid = mag[mag > 1.5]
    if valid.size < 10:
        return None
    med = float(np.median(valid))
    return round(FOCAL_PX * baseline_cm / med, 1) if med > 0.5 else None


def analyze_motion(
    img1_bgr: np.ndarray,
    img2_bgr: np.ndarray,
    baseline_cm: float,
) -> dict:
    """
    Farneback optical flow between consecutive frames.

    Returns scene_depth_cm (scene-wide median), flow_px, stuck flag,
    and flow_field (H×W×2) for downstream per-object depth queries.
    """
    result: dict = {"scene_depth_cm": None, "flow_px": 0.0, "stuck": False, "flow_field": None}
    if baseline_cm < 1.0:
        return result
    try:
        g1   = cv2.cvtColor(img1_bgr, cv2.COLOR_BGR2GRAY)
        g2   = cv2.cvtColor(img2_bgr, cv2.COLOR_BGR2GRAY)
        flow = cv2.calcOpticalFlowFarneback(
            g1, g2, None, pyr_scale=0.5, levels=3, winsize=15,
            iterations=3, poly_n=5, poly_sigma=1.2, flags=0,
        )
        result["flow_field"] = flow
        h, w = g1.shape
        m    = min(h, w) // 6
        cy, cx = h // 2, w // 2
        roi  = flow[cy - m:cy + m, cx - m:cx + m]
        mag  = np.sqrt(roi[..., 0] ** 2 + roi[..., 1] ** 2)
        valid = mag[mag > 1.5]
        if valid.size < 20:
            result["stuck"] = baseline_cm > STUCK_BASELINE_CM
            return result
        med = float(np.median(valid))
        result["flow_px"] = round(med, 2)
        if med > 0.5:
            result["scene_depth_cm"] = round(FOCAL_PX * baseline_cm / med, 1)
        result["stuck"] = baseline_cm > STUCK_BASELINE_CM and med < STUCK_FLOW_PX
        return result
    except Exception as e:
        result["notes"] = f"flow error: {e}"
        return result


# ── Floor-plane geometry ──────────────────────────────────────────────────────
def floor_plane_depth(y_base: float, y_horizon: float) -> float | None:
    """
    Metric distance to a floor-contact point via camera height + pixel row.

    Uses FOCAL_PX_V (vertical focal length in the 640×480 image, accounting for
    the 16:9→4:3 stretch) rather than the horizontal FOCAL_PX.

    Returns None when y_base is at or above the horizon (object too far),
    or when the implied distance exceeds 800 cm (formula unreliable).
    """
    dy = y_base - y_horizon
    if dy < 2:
        return None
    d = CAMERA_HEIGHT_CM * FOCAL_PX_V / dy
    return round(d, 1) if d <= 800 else None


def _estimate_horizon(img_bgr: np.ndarray) -> float | None:
    """
    Estimate the horizon y-coordinate from near-horizontal Hough lines.

    Long horizontal edges (door frames, skirting boards, wall junctions)
    converge near the horizon.  The median y-intercept of lines that are
    within ~8° of horizontal and land in the middle 55 % of the frame
    gives a robust vanishing-line estimate.
    """
    h, w = img_bgr.shape[:2]
    gray  = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 30, 90)
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180, threshold=40,
        minLineLength=int(w * 0.20), maxLineGap=15,
    )
    if lines is None:
        return None
    ys = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        if dx < 1 or dy / dx > 0.15:   # reject lines steeper than ~8°
            continue
        y_mid = (y1 + y2) / 2.0
        if h * 0.20 < y_mid < h * 0.75:
            ys.append(y_mid)
    return float(np.median(ys)) if len(ys) >= 3 else None


def _update_horizon(img_bgr: np.ndarray) -> float:
    """
    Update the EMA-smoothed horizon estimate and return the current value.

    Falls back to frame_h / 2 (perfectly horizontal camera) when no lines
    have been detected yet.  Clamped to [25 %, 75 %] of frame height to
    prevent runaway estimates after an extreme bump.
    """
    global _y_horizon_ema
    h = img_bgr.shape[0]
    est = _estimate_horizon(img_bgr)
    if est is not None:
        if _y_horizon_ema is None:
            _y_horizon_ema = est
        else:
            _y_horizon_ema = (
                (1 - _HORIZON_EMA_ALPHA) * _y_horizon_ema
                + _HORIZON_EMA_ALPHA * est
            )
    base = _y_horizon_ema if _y_horizon_ema is not None else h / 2.0
    # Wide clamp [10 %, 90 %] supports camera tilts up to ~20° in either direction.
    return float(np.clip(base, h * 0.10, h * 0.90))


def _floor_texture_depth(img_bgr: np.ndarray, y_horizon: float) -> float | None:
    """
    Estimate scene depth from the texture-frequency gradient on the floor.

    Perspective causes a uniform floor texture to appear at higher spatial
    frequency further away: E ∝ d² where E is Laplacian variance and d is
    distance.  We calibrate the constant k = E/d² using floor-plane geometry
    on 8 horizontal bands, then derive a scene-level depth from the median
    texture energy.

    Returns None on smooth / textureless floors (variance too uniform) or
    when y_horizon leaves fewer than 3 usable bands.
    Weight guideline for callers: 0.25 (weaker than fast_depth, stronger
    than known-height — metric but assumes uniform texture).
    """
    h, w = img_bgr.shape[:2]
    floor_top = max(0, int(y_horizon))
    if floor_top >= h - 20:
        return None

    gray   = cv2.cvtColor(img_bgr[floor_top:, :], cv2.COLOR_BGR2GRAY)
    roi_h  = gray.shape[0]
    n_bands = 8
    bh     = max(1, roi_h // n_bands)

    pairs: list[tuple[float, float]] = []   # (d_cm, laplacian_variance)
    for i in range(n_bands):
        y0  = i * bh
        y1b = min(roi_h, y0 + bh)
        band = gray[y0:y1b, :]
        energy = float(np.var(cv2.Laplacian(band.astype(np.float32), cv2.CV_32F)))
        y_abs  = floor_top + (y0 + y1b) / 2.0
        d      = floor_plane_depth(y_abs, y_horizon)
        if d is not None and energy > 0:
            pairs.append((d, energy))

    if len(pairs) < 3:
        return None

    energies = [e for _, e in pairs]
    if np.std(energies) < 5.0:     # floor too uniform — texture gradient unreliable
        return None

    # k = E / d²  (median across bands)
    k = float(np.median([e / (d ** 2) for d, e in pairs]))
    if k <= 0:
        return None

    med_energy = float(np.median(energies))
    d_est = round((med_energy / k) ** 0.5, 1)
    return d_est if 20 < d_est < 600 else None


# ── Distance fusion ───────────────────────────────────────────────────────────
def _fuse_distances(estimates: list[tuple[float, float]]) -> float | None:
    """
    Confidence-weighted average of distance estimates with outlier rejection.

    estimates: list of (distance_cm, weight) pairs.

    Any estimate that diverges more than 50 % from the median is treated as
    an outlier and its weight is reduced by 5×. This prevents a single bad
    signal (e.g. a wrongly-matched flow vector or an atypical object height)
    from dominating when two other methods agree.

    Weight guidelines (callers should pass these):
      fast_depth      — 0.7     (metric, primary sensor, highest trust)
      floor-plane     — 0.8     (metric, exact for floor-contact objects)
      optical flow    — 0.2–0.7 (scales with baseline_cm / 25, capped at 0.7)
      texture depth   — 0.25    (metric but assumes uniform floor texture)
      known height/w  — 0.3     (assumes typical object size, lowest trust)
    """
    if not estimates:
        return None
    if len(estimates) == 1:
        return estimates[0][0]

    dists    = sorted(d for d, _ in estimates)
    median_d = dists[len(dists) // 2]

    adjusted: list[tuple[float, float]] = []
    for d, w in estimates:
        if median_d > 0 and abs(d - median_d) / median_d > 0.50:
            w /= 5.0          # outlier: keep in average but heavily discounted
        adjusted.append((d, w))

    total_w = sum(w for _, w in adjusted)
    if total_w == 0:
        return None
    return round(sum(d * w for d, w in adjusted) / total_w, 1)


# ── YOLO box parser (shared by yolo_det and yolo_seg) ────────────────────────
def _parse_yolo_boxes(
    raw: dict[str, np.ndarray],
    orig_w: int,
    orig_h: int,
    depth_map: np.ndarray | None = None,
    scene_depth_cm: float | None = None,
    flow_field: np.ndarray | None = None,
    baseline_cm: float = 0.0,
    y_horizon: float = 0.0,
) -> list[dict]:
    """
    Parse raw Hailo YOLO output into detection dicts.

    All available depth signals are combined via _fuse_distances():
      fast_depth    w = 0.7      (metric depth, primary sensor — best single-frame signal)
      optical flow  w = 0.2–0.7  (scales with baseline; 20 cm scan rock → 0.7)
      known height  w = 0.3      (assumed typical object size, lowest trust)

    Estimates that diverge >50 % from the median are down-weighted 5× before
    averaging, so agreement between methods raises confidence and disagreement
    preserves the majority signal rather than blindly averaging.
    real_height_cm is back-computed from the fused distance.
    """
    detections: list[dict] = []
    inf_w, inf_h = 640, 640

    for arr in raw.values():
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

            raw4 = det[0:4].tolist()
            if max(raw4) <= 1.0:
                y1n, x1n, y2n, x2n = raw4
                x1 = int(x1n * orig_w); y1 = int(y1n * orig_h)
                x2 = int(x2n * orig_w); y2 = int(y2n * orig_h)
            else:
                x1 = int(det[1] / inf_w * orig_w); y1 = int(det[0] / inf_h * orig_h)
                x2 = int(det[3] / inf_w * orig_w); y2 = int(det[2] / inf_h * orig_h)

            x1, x2 = max(0, x1), min(orig_w, x2)
            y1, y2 = max(0, y1), min(orig_h, y2)
            if x2 <= x1 or y2 <= y1:
                continue

            pixel_h = y2 - y1
            cx_px   = (x1 + x2) / 2
            cy_px   = (y1 + y2) / 2
            pos     = ("left"   if cx_px < orig_w / 3
                        else "right" if cx_px > 2 * orig_w / 3
                        else "center")

            estimates: list[tuple[float, float]] = []

            # Optical flow — confidence scales with baseline (20 cm scan rock → 0.7)
            if flow_field is not None and baseline_cm >= 1.0:
                fd = object_depth_from_flow(flow_field, (x1, y1, x2, y2), baseline_cm)
                if fd is not None:
                    flow_w = min(0.7, 0.2 + baseline_cm / 25.0)
                    estimates.append((fd, flow_w))

            # fast_depth: scene_depth_cm is the metric median; depth_map encodes
            # normalised inverse depth so ref_inv/obj_inv gives the relative scale.
            if depth_map is not None and scene_depth_cm is not None:
                dh, dw = depth_map.shape[:2]
                cy_s = max(0, min(dh - 1, int(cy_px * dh / orig_h)))
                cx_s = max(0, min(dw - 1, int(cx_px * dw / orig_w)))
                obj_inv = float(depth_map[cy_s, cx_s])
                ref_inv = float(depth_map[dh // 2, dw // 2])
                if obj_inv > 0.05 and ref_inv > 0.05:
                    md = round(scene_depth_cm * ref_inv / obj_inv, 1)
                    estimates.append((md, 0.7))   # primary: metric depth

            # Known-height formula
            rh = KNOWN_HEIGHTS_CM.get(class_name)
            if rh and pixel_h > 5:
                estimates.append((round(rh * FOCAL_PX / pixel_h, 1), 0.3))

            # Known-width formula — complement to height; useful when the
            # object is wide but partially occluded vertically.
            rw = KNOWN_WIDTHS_CM.get(class_name)
            pixel_w = x2 - x1
            if rw and pixel_w > 5:
                estimates.append((round(rw * FOCAL_PX / pixel_w, 1), 0.3))

            # Floor-plane geometry — bbox bottom is the floor contact point.
            # Metric and reliable; weight 0.8 for floor-touching objects.
            if y_horizon > 0:
                fp = floor_plane_depth(y2, y_horizon)
                if fp is not None:
                    estimates.append((fp, 0.8))

            dist_cm   = _fuse_distances(estimates)
            real_h_cm = round(pixel_h * dist_cm / FOCAL_PX, 1) if (dist_cm and pixel_h > 5) else None

            detections.append({
                "class_name":     class_name,
                "confidence":     round(score, 3),
                "bbox_px":        (x1, y1, x2, y2),
                "distance_cm":    dist_cm,
                "real_height_cm": real_h_cm,
                "position":       pos,
                "dist_sources":   len(estimates),   # how many methods contributed
            })
    return detections


def _person_soft_detected(raw: dict[str, np.ndarray] | None, threshold: float = 0.15) -> bool:
    """
    Return True if the raw Hailo output contains any person detection at or
    above `threshold` (default 0.15).

    Used only for greeting — obstacle avoidance uses the main 0.35 threshold.
    From floor level, YOLO confidence for partially-visible seated/standing
    people rarely exceeds 0.35, so the greeting never fired at standard threshold.
    """
    if raw is None:
        return False
    person_idx = COCO_CLASSES.index("person")
    for arr in raw.values():
        a = arr[0] if arr.ndim > 2 else arr
        if a.ndim != 2:
            continue
        for det in a:
            if len(det) < 6:
                continue
            if float(det[4]) < threshold:
                continue
            if int(det[5]) == person_idx:
                return True
    return False


def run_yolo(img_bgr: np.ndarray) -> list[dict]:
    """Legacy alias — runs yolo_det, returns detection list."""
    orig_h, orig_w = img_bgr.shape[:2]
    raw = _hailo_infer("yolo_det", img_bgr)
    return _parse_yolo_boxes(raw, orig_w, orig_h) if raw else []


def yolo_labels(detections: list[dict]) -> list[str]:
    return list({d["class_name"] for d in detections})


# ── fast_depth depth parser ───────────────────────────────────────────────────
def _parse_fast_depth(depth_m: np.ndarray, orig_w: int, orig_h: int) -> np.ndarray:
    """
    Convert fast_depth metric output (H×W float32, metres) to a normalised
    inverse-depth map [0, 1]: 1 = close, 0 = far.

    The inversion matches the convention used by _open_space_from_depth and
    detect_door_cv (low value = further away = potential open space).
    """
    inv = 1.0 / (depth_m + 0.01)    # +0.01 avoids div/0 at zero-depth pixels
    lo, hi = float(inv.min()), float(inv.max())
    if hi > lo:
        inv = (inv - lo) / (hi - lo)
    return cv2.resize(inv.astype(np.float32), (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)


# ── Open-space analysis from depth map ───────────────────────────────────────
def _open_space_from_depth(depth_map: np.ndarray,
                           img_bgr: np.ndarray | None = None) -> dict:
    """
    Fraction of far pixels (potential open space / doorway) in each frame third,
    soft-scaled by per-third texture confidence.

    A blank, textureless region (low Laplacian variance) is likely a wall that
    fast_depth misreads as far.  Texture confidence = min(1, var / TEXTURE_MIN_VAR)
    suppresses the open_space score without hard-blocking it.

    img_bgr is optional; when absent, texture gating is skipped.
    """
    threshold = float(np.percentile(depth_map, 40))   # low inv-depth = far
    far   = depth_map <= threshold
    h, w  = depth_map.shape[:2]
    third = w // 3
    slices = {
        "left":   (slice(None), slice(0, third)),
        "center": (slice(None), slice(third, 2 * third)),
        "right":  (slice(None), slice(2 * third, w)),
    }

    result: dict[str, float] = {}
    for side, (rs, cs) in slices.items():
        raw = float(far[rs, cs].mean())

        if img_bgr is not None:
            # Compute Laplacian variance on the grayscale region to measure texture.
            region = img_bgr[rs, cs]
            gray   = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
            lap_var = float(cv2.Laplacian(gray.astype(np.float32), cv2.CV_32F).var())
            confidence = min(1.0, lap_var / TEXTURE_MIN_VAR)
            result[side] = raw * confidence
        else:
            result[side] = raw

    return result


# ── Obstacle analysis ─────────────────────────────────────────────────────────
def analyze_obstacles(
    detections: list[dict],
    frame_w: int,
    frame_h: int,
    scene_depth_cm: float | None = None,
    flow_field: np.ndarray | None = None,
    baseline_cm: float = 0.0,
) -> dict:
    """
    Map detections to blocked left/center/right regions.

    Uses the fused distance_cm already computed by _parse_yolo_boxes.
    Falls back to scene_depth_cm only when a detection has no distance at all
    (e.g. raw detections passed without going through _parse_yolo_boxes).
    """
    floor_threshold = frame_h * 0.4
    enriched: list[dict] = []
    blocked    = {"left": False, "center": False, "right": False}
    nearest_cm: float | None = None

    for det in detections:
        x1, y1, x2, y2 = det["bbox_px"]

        # Trust the fused estimate; fall back to scene depth if completely absent
        dist   = det.get("distance_cm") or scene_depth_cm
        real_h = det.get("real_height_cm")

        class_name   = det.get("class_name", "")
        is_floor_obj = class_name in FLOOR_BLOCKER_CLASSES

        # Floor objects (chairs, couches, tables): expand x-extent to cover the
        # full ground footprint, and always treat as floor-level.  The camera
        # looks upward so the seat appears high in frame — the area beneath it
        # is still off-limits.
        if is_floor_obj:
            margin = int(frame_w * CHAIR_BBOX_EXPAND)
            x1 = max(0, x1 - margin)
            x2 = min(frame_w, x2 + margin)

        # Persons always occupy floor space regardless of where they appear in
        # the frame (upward camera may not capture their feet).
        is_person    = class_name == "person"
        at_floor     = is_floor_obj or is_person or (y2 > floor_threshold)
        # Floor objects (chairs etc.) block only when within CHAIR_BLOCK_DIST_CM.
        # This allows the robot to navigate a narrow corridor alongside chairs
        # without steering away prematurely.  Persons always block (can't predict
        # movement).  Leg detection handles under-chair protection at close range.
        if is_floor_obj:
            within_range = dist is None or dist < CHAIR_BLOCK_DIST_CM
        elif is_person:
            within_range = True
        else:
            within_range = dist is None or dist < OBSTACLE_BLOCK_DIST_CM
        blocking     = at_floor and within_range

        enriched.append({**det, "blocking": blocking, "distance_cm": dist, "real_height_cm": real_h})
        if blocking:
            if dist is not None and (nearest_cm is None or dist < nearest_cm):
                nearest_cm = dist

            if is_floor_obj:
                # Block every frame-third that the expanded bbox overlaps —
                # the ground footprint under the chair/couch may span all three.
                third = frame_w / 3
                for side, (lo, hi) in [("left",   (0,       third)),
                                        ("center", (third,   2 * third)),
                                        ("right",  (2 * third, frame_w))]:
                    if x1 < hi and x2 > lo:
                        blocked[side] = True
            else:
                blocked[det["position"]] = True

            # Lateral clearance: if the gap between this obstacle and the frame
            # edge is narrower than the robot, that side is also impassable.
            if dist and dist > 0:
                robot_px = ROBOT_WIDTH_CM * FOCAL_PX / dist
                if x1 < robot_px:           # not enough room on the left
                    blocked["left"] = True
                if (frame_w - x2) < robot_px:  # not enough room on the right
                    blocked["right"] = True

    clear_path: str | None = None
    for c in ("center", "right", "left"):
        if not blocked[c]:
            clear_path = c
            break

    return {"obstacles": enriched, "blocked": blocked, "clear_path": clear_path, "nearest_cm": nearest_cm}


# ── Door detection (OpenCV + depth + segmentation fusion) ─────────────────────
def detect_door_cv(
    img_bgr: np.ndarray,
    scene_depth_cm: float | None = None,
    depth_map: np.ndarray | None = None,
) -> dict:
    """
    Detect a doorway via near-vertical pillar pairs with a wide bright gap.

    Three additional signals fused on top of the geometric test:
      scene_depth_cm  validates real-world gap width (50–150 cm)
      depth_map       gap should be further than its surroundings
      door_seg_mask   segmentation 'door' pixels boost confidence

    The horizontal-bar false-positive filter only rejects bars in the LOWER
    40 % of frame (bench/shelf level). Bars higher up are the door frame
    crossbar at 2 m+ and must not cause rejection.
    """
    null: dict = {
        "door_visible": False, "door_open": False, "door_position": None,
        "in_doorway": False, "door_pixel_width": 0, "door_distance_cm": None,
        "gap_real_width_cm": None, "confidence": 0.0, "notes": "no door",
    }
    try:
        h, w  = img_bgr.shape[:2]
        gray  = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 30, 100, apertureSize=3)

        lines = cv2.HoughLinesP(
            edges, rho=1, theta=np.pi / 180, threshold=50,
            minLineLength=int(h * 0.40), maxLineGap=25,
        )
        vert_segs: list[tuple[int, int]] = []
        if lines is not None:
            for (x1, y1, x2, y2), in lines:
                dx, dy = abs(x2 - x1), abs(y2 - y1)
                if dy > 0 and dx / dy < 0.25:
                    vert_segs.append(((x1 + x2) // 2, abs(y2 - y1)))
        if len(vert_segs) < 2:
            return null

        vert_segs.sort(key=lambda s: s[0])
        clusters: list[tuple[int, int]] = []
        gx, gs = [vert_segs[0][0]], [vert_segs[0][1]]
        for xc, span in vert_segs[1:]:
            if xc - gx[-1] < 40:
                gx.append(xc); gs.append(span)
            else:
                clusters.append((int(np.mean(gx)), max(gs)))
                gx, gs = [xc], [span]
        clusters.append((int(np.mean(gx)), max(gs)))
        if len(clusters) < 2:
            return null

        best: tuple | None = None
        best_score = 0.0
        for i in range(len(clusters)):
            for j in range(i + 1, len(clusters)):
                (lx, lspan), (rx, rspan) = clusters[i], clusters[j]
                gap = rx - lx
                if not (0.20 * w < gap < 0.70 * w):
                    continue
                if max(lspan, rspan) < h * 0.40:
                    continue
                edge_dens  = float(edges[:, lx:rx].mean()) / 255.0
                gap_bright = float(gray[:, lx:rx].mean())
                sl = gray[:, max(0, lx - gap // 3):lx]
                sr = gray[:, rx:min(w, rx + gap // 3)]
                sur = float(np.concatenate([sl, sr], axis=1).mean()) if sl.size + sr.size else gap_bright
                score = gap * (1.0 - edge_dens * 4.0) * max(gap_bright / max(sur, 1.0), 0.4)
                if score > best_score:
                    best_score = score
                    best = (lx, rx, gap_bright, sur, edge_dens)

        if best is None or best_score < 50.0:
            return null

        lx, rx, gap_bright, sur_bright, edge_dens = best
        gap      = rx - lx
        center_x = (lx + rx) / 2.0
        bright_r = gap_bright / max(sur_bright, 1.0)

        # Filter 1: warm-hued gap → curtain
        gap_hsv   = cv2.cvtColor(img_bgr[:, lx:rx], cv2.COLOR_BGR2HSV)
        warm      = (gap_hsv[:, :, 1] > 50) & ((gap_hsv[:, :, 0] < 20) | (gap_hsv[:, :, 0] > 160))
        if float(warm.mean()) > 0.15:
            return {**null, "notes": "rejected: warm-hued gap (curtain)"}

        # Filter 2: horizontal bar only in lower 40 % of frame → bench/shelf
        horiz = cv2.HoughLinesP(
            edges[:, lx:rx], rho=1, theta=np.pi / 180, threshold=15,
            minLineLength=int(gap * 0.40), maxLineGap=8,
        )
        if horiz is not None:
            for (x1h, y1h, x2h, y2h), in horiz:
                if abs(y2h - y1h) < 20 and (y1h + y2h) / 2 > h * 0.60:
                    return {**null, "notes": "rejected: horizontal bar in lower gap (bench/shelf)"}

        # Filter 3: real-world gap width via scene depth
        gap_real_cm: float | None = None
        if scene_depth_cm is not None and scene_depth_cm > 0:
            gap_real_cm = round(gap * scene_depth_cm / FOCAL_PX, 1)
            if not (ROBOT_WIDTH_CM < gap_real_cm < 150):
                return {**null, "notes": f"rejected: gap real width {gap_real_cm:.0f} cm (not door-sized)"}

        # Filter 4: fast_depth — gap should be further (lower inv-depth) than surroundings
        if depth_map is not None:
            dh, dw = depth_map.shape[:2]
            sx, sy = dw / w, dh / h
            gslice  = depth_map[:, int(lx * sx):int(rx * sx)]
            lslice  = depth_map[:, max(0, int((lx - gap // 3) * sx)):int(lx * sx)]
            rslice  = depth_map[:, int(rx * sx):min(dw, int((rx + gap // 3) * sx))]
            if gslice.size > 0 and lslice.size + rslice.size > 0:
                gap_inv = float(gslice.mean())
                sur_inv = float(np.concatenate([lslice.ravel(), rslice.ravel()]).mean())
                # gap_inv should be LOWER (further away) than surroundings
                if gap_inv > sur_inv * 1.10:
                    return {**null, "notes": "rejected: depth map shows gap not deeper than surroundings"}

        pos        = ("left"   if center_x < w / 3
                       else "right" if center_x > 2 * w / 3
                       else "center")
        in_doorway = lx < w * 0.18 and rx > w * 0.82
        door_open  = bright_r >= 0.82
        door_dist  = round(scene_depth_cm, 1) if gap_real_cm is not None and scene_depth_cm else \
                     round(DOOR_WIDTH_CM * FOCAL_PX / gap, 1) if gap > 0 else None

        confidence = min(1.0, best_score / 500.0)
        notes = (
            f"gap={gap}px dist≈{door_dist}cm real_w≈{gap_real_cm}cm pos={pos} "
            f"bright={bright_r:.2f} edge={edge_dens:.2f}"
        )
        return {
            "door_visible":     True,
            "door_open":        door_open,
            "door_position":    pos,
            "in_doorway":       in_doorway,
            "door_pixel_width": gap,
            "door_distance_cm": door_dist,
            "gap_real_width_cm": gap_real_cm,
            "confidence":       confidence,
            "notes":            notes,
        }
    except Exception as e:
        return {**null, "notes": f"cv error: {e}"}


# ── Thin chair-leg detector ───────────────────────────────────────────────────
def detect_thin_legs(
    img_bgr: np.ndarray,
    scene_depth_cm: float | None = None,
    y_horizon: float = 0.0,
) -> dict:
    """
    Detect thin near-vertical structures (chair legs) in the floor region.

    Looks at the bottom LEG_FLOOR_FRAC of the frame using Canny + HoughLinesP
    tuned for thin (~1 cm diameter) near-vertical lines.  Returns a dict with
    the same shape as build_obstacle_map's output so it can be merged in.

    blocked  dict  left/center/right — True when a leg cluster occupies that third
    legs     list  x-centre fractions of detected leg clusters (for logging)
    """
    h, w = img_bgr.shape[:2]
    # With a tilted camera the floor occupies less of the bottom of the frame.
    # Cap the search zone to (floor_height + 8 % margin), so we don't look for
    # legs in wall/ceiling area when the camera is pointed upward.
    if y_horizon > 0:
        floor_frac = min(LEG_FLOOR_FRAC, (h - y_horizon) / h + 0.08)
        floor_frac = max(0.12, floor_frac)
    else:
        floor_frac = LEG_FLOOR_FRAC
    floor_top = int(h * (1.0 - floor_frac))
    roi = img_bgr[floor_top:, :]
    roi_h = roi.shape[0]

    gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur  = cv2.GaussianBlur(gray, (3, 3), 0)
    edges = cv2.Canny(blur, 35, 100)   # moderate thresholds: catch legs without picking up floor/wall noise
    # Connect broken edge chains on thin/reflective legs before Hough transform
    vkernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 7))
    edges   = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, vkernel)

    min_len = int(roi_h * LEG_MIN_LENGTH)
    lines = cv2.HoughLinesP(
        edges,
        rho=1, theta=np.pi / 180,
        threshold=22,
        minLineLength=min_len,
        maxLineGap=18,
    )

    blocked = {"left": False, "center": False, "right": False}
    leg_xs: list[float] = []

    if lines is None:
        return {"blocked": blocked, "legs": leg_xs}

    # Keep only near-vertical segments and cluster by x-position.
    # Also track the max y (lowest pixel) per segment for floor-plane depth.
    segs: list[tuple[float, float]] = []   # (x_centre, y_bottom_in_full_frame)
    for line in lines:
        x1l, y1l, x2l, y2l = line[0]
        dy = abs(y2l - y1l)
        dx = abs(x2l - x1l)
        if dy == 0:
            continue
        if dx / dy > LEG_MAX_SLOPE:   # too diagonal — not a leg
            continue
        x_mid  = (x1l + x2l) / 2.0
        y_bot  = floor_top + max(y1l, y2l)   # convert ROI coords to full frame
        segs.append((x_mid, y_bot))

    if not segs:
        return {"blocked": blocked, "legs": leg_xs}

    # Simple 1-D clustering by x-position (merge within 30 px)
    segs.sort(key=lambda s: s[0])
    clusters: list[list[tuple[float, float]]] = [[segs[0]]]
    for seg in segs[1:]:
        if seg[0] - clusters[-1][-1][0] < 30:
            clusters[-1].append(seg)
        else:
            clusters.append([seg])

    for cl in clusters:
        if len(cl) < LEG_MIN_CLUSTER:   # single stray segment — ignore
            continue
        cx     = float(np.mean([s[0] for s in cl]))
        y_bot  = float(max(s[1] for s in cl))
        cx_frac = cx / w
        leg_xs.append(round(cx_frac, 3))

        # Distance to this leg: prefer floor-plane formula (metric), fall
        # back to scene_depth_cm.  Skip when no estimate is available —
        # wall edges and floor texture without depth produce false positives.
        fp_dist  = floor_plane_depth(y_bot, y_horizon) if y_horizon > 0 else None
        leg_dist = fp_dist if fp_dist is not None else scene_depth_cm

        if leg_dist is None or leg_dist > LEG_MAX_DIST_CM:
            continue

        # Estimate real-world clearance: use ROBOT_WIDTH_CM at leg distance.
        if leg_dist and leg_dist > 0:
            robot_px = ROBOT_WIDTH_CM * FOCAL_PX / leg_dist
            left_clear  = cx > robot_px / 2
            right_clear = (w - cx) > robot_px / 2
        else:
            left_clear  = cx > w * 0.15
            right_clear = (w - cx) > w * 0.15

        # Mark the third the leg sits in, and any third where the robot can't
        # fit between the leg and the frame edge.
        third = w / 3
        if cx < third:
            blocked["left"] = True
            if not right_clear:
                blocked["center"] = True
        elif cx > 2 * third:
            blocked["right"] = True
            if not left_clear:
                blocked["center"] = True
        else:
            blocked["center"] = True
            if not left_clear:
                blocked["left"] = True
            if not right_clear:
                blocked["right"] = True

    # ── Chair pair analysis ───────────────────────────────────────────────────
    # Chairs often have an invisible horizontal bar at floor level between their
    # legs.  For every pair of validated clusters, compute the real-world gap; if
    # it is less than CHAIR_PAIR_MAX_CM they are likely the same chair and the
    # entire corridor between (and including) the legs is blocked.
    valid_clusters = [
        (float(np.mean([s[0] for s in cl])),           # cx
         float(max(s[1] for s in cl)))                  # y_bot (lowest pixel)
        for cl in clusters if len(cl) >= LEG_MIN_CLUSTER
    ]
    third = w / 3
    for i, (cx_a, y_bot_a) in enumerate(valid_clusters):
        for cx_b, y_bot_b in valid_clusters[i + 1:]:
            # Use the nearer leg's y_bot for a more accurate distance estimate
            y_near = max(y_bot_a, y_bot_b)   # larger y = closer to camera
            dist = (floor_plane_depth(y_near, y_horizon) if y_horizon > 0
                    else scene_depth_cm)
            if not dist or dist <= 0:
                continue
            gap_px = abs(cx_b - cx_a)
            gap_cm = gap_px * dist / FOCAL_PX
            if gap_cm > CHAIR_PAIR_MAX_CM:
                continue   # too far apart — different chairs or not a chair
            if dist is None or dist > LEG_MAX_DIST_CM:
                continue   # unknown distance or too far — skip

            # Block every frame-third that overlaps the region [x_left, x_right]
            x_left  = min(cx_a, cx_b)
            x_right = max(cx_a, cx_b)
            for side, (lo, hi) in [("left",   (0,       third)),
                                    ("center", (third,   2 * third)),
                                    ("right",  (2 * third, w))]:
                if x_left < hi and x_right > lo:
                    blocked[side] = True
            print(
                f"[vision] chair pair: x=[{x_left/w:.2f},{x_right/w:.2f}] "
                f"gap≈{gap_cm:.0f}cm @ {dist:.0f}cm — gap blocked",
                flush=True,
            )

    legs_blocking = any(blocked.values())
    return {"blocked": blocked, "legs": leg_xs, "legs_blocking": legs_blocking}


# ── Main scene analysis ───────────────────────────────────────────────────────
def analyze_scene(
    img_bgr: np.ndarray,
    prev_img: np.ndarray | None = None,
    baseline_cm: float = 0.0,
) -> dict:
    """
    Run all available models on img_bgr and return a fused scene dict.

    Keys:
      detections      list[dict]        YOLO objects: class, bbox, distance_cm,
                                        real_height_cm, position
      depth_map       ndarray|None      H×W float32, 0=far / 1=close  (fast_depth)
      door            dict              fused door detection (same fields as detect_door_cv)
      obstacles       dict              blocked / clear_path / nearest_cm / obstacles[]
      open_space      dict              left/center/right far-pixel fractions from depth
      scene_depth_cm  float|None        scene-wide depth from optical flow
      flow_field      ndarray|None      H×W×2 flow for ad-hoc per-region depth queries
      stuck           bool
    """
    orig_h, orig_w = img_bgr.shape[:2]

    # 0. Horizon calibration — estimate vanishing line of the floor plane.
    # Updated every frame via EMA so it tracks camera tilt after bumps.
    y_horizon = _update_horizon(img_bgr)

    # 1. fast_depth — primary depth sensor (metric, every frame, no motion needed)
    depth_map: np.ndarray | None = None
    scene_depth_cm: float | None = None
    depth_m = _infer_fast_depth(img_bgr)
    if depth_m is not None:
        scene_depth_cm = round(float(np.median(depth_m)) * 100.0, 1)  # metres → cm
        depth_map = _parse_fast_depth(depth_m, orig_w, orig_h)

    # 2. Optical flow — stuck detection + per-object flow depth vectors
    flow_field: np.ndarray | None = None
    stuck = False
    if prev_img is not None and baseline_cm >= 1.0:
        motion         = analyze_motion(prev_img, img_bgr, baseline_cm)
        flow_field     = motion.get("flow_field")
        stuck          = motion.get("stuck", False)
        if scene_depth_cm is None:
            scene_depth_cm = motion.get("scene_depth_cm")    # fallback when fast_depth unavailable

    # 2b. Texture-gradient depth — fallback when both fast_depth and flow unavailable.
    if scene_depth_cm is None:
        scene_depth_cm = _floor_texture_depth(img_bgr, y_horizon)

    # 3. YOLO detection
    detections: list[dict] = []
    raw = _hailo_infer("yolo_det", img_bgr)
    person_soft = _person_soft_detected(raw)   # low-conf check for greeting only
    if raw is not None:
        detections = _parse_yolo_boxes(raw, orig_w, orig_h, depth_map, scene_depth_cm,
                                       flow_field, baseline_cm, y_horizon)

    # 4. Obstacle map (YOLO detections + chair-leg geometry)
    obstacles = analyze_obstacles(
        detections, orig_w, orig_h, scene_depth_cm, flow_field, baseline_cm
    )
    leg_result = detect_thin_legs(img_bgr, scene_depth_cm=scene_depth_cm,
                                  y_horizon=y_horizon)
    legs_blocking = leg_result.get("legs_blocking", False)
    if legs_blocking:
        for side, val in leg_result["blocked"].items():
            if val:
                obstacles["blocked"][side] = True
        # Recompute clear_path after merging leg blocks
        obstacles["clear_path"] = next(
            (c for c in ("center", "right", "left") if not obstacles["blocked"][c]),
            None,
        )
        if leg_result["legs"]:
            print(
                f"[vision] legs detected at x-fracs={leg_result['legs']} "
                f"blocking={[s for s, v in leg_result['blocked'].items() if v]}",
                flush=True,
            )

    # 5. Open-space map from depth, texture-gated to suppress blank walls
    open_space = (
        _open_space_from_depth(depth_map, img_bgr)
        if depth_map is not None
        else {"left": 0.0, "center": 0.0, "right": 0.0}
    )

    # 6. Camera tilt — derived from y_horizon every frame.
    # Formula: tan(tilt) = (y_horizon - orig_h/2) / FOCAL_PX_V
    # Positive tilt = camera pointed above horizontal.
    tilt_deg = round(math.degrees(math.atan2(y_horizon - orig_h / 2.0, FOCAL_PX_V)), 1)
    # Closest floor point visible at the bottom pixel — grows with upward tilt.
    dy_bottom = max(1.0, orig_h - y_horizon)
    blind_spot_cm = round(CAMERA_HEIGHT_CM * FOCAL_PX_V / dy_bottom, 1)

    # 7. Door detection — OpenCV geometry + YOLO panel, fused
    cv_door    = detect_door_cv(img_bgr, scene_depth_cm=scene_depth_cm, depth_map=depth_map)
    yolo_doors = _infer_door_yolo(img_bgr)
    door       = _fuse_door_detections(cv_door, yolo_doors, orig_w)

    return {
        "detections":     detections,
        "depth_map":      depth_map,
        "door":           door,
        "yolo_doors":     yolo_doors,
        "obstacles":      obstacles,
        "open_space":     open_space,
        "scene_depth_cm": scene_depth_cm,
        "flow_field":     flow_field,
        "stuck":          stuck,
        "y_horizon":      y_horizon,
        "legs_blocking":  legs_blocking,
        "tilt_deg":       tilt_deg,
        "blind_spot_cm":  blind_spot_cm,
        "person_soft":    person_soft,   # person at ≥0.15 conf — for greeting only
    }


# ── Odometry tracker ──────────────────────────────────────────────────────────
class OdometryTracker:
    """
    Dead-reckoning position from commanded wheel moves.
    x — forward (cm), y — left (cm), heading — CCW+ degrees (0 = initial forward).
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
            rad       = math.radians(self._heading)
            self._x  += dist * math.cos(rad)
            self._y  += dist * math.sin(rad)

    def turn(self, deg: float):
        with self._lock:
            self._heading = (self._heading + deg) % 360

    @property
    def x(self) -> float:
        with self._lock: return self._x

    @property
    def y(self) -> float:
        with self._lock: return self._y

    @property
    def heading(self) -> float:
        with self._lock: return self._heading

    def distance_from_origin(self) -> float:
        with self._lock: return math.hypot(self._x, self._y)

    def snapshot(self) -> tuple[float, float, float]:
        with self._lock: return (self._x, self._y, self._heading)


# ── Calibration helper ────────────────────────────────────────────────────────
def calibrate_focal(known_w_cm: float, pixel_w: int, dist_cm: float) -> float:
    return (pixel_w * dist_cm) / known_w_cm
