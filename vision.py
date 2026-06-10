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
# Pi Camera v2/v3, 640×480, horizontal FOV ≈ 66°
FOCAL_PX          = 492.0
DOOR_WIDTH_CM     = 80.0
CAMERA_HEIGHT_CM  = 20.0     # camera is ~20 cm off the floor
ROBOT_WIDTH_CM    = 40.0     # Roomba diameter ≈ 2 × camera height
OBSTACLE_BLOCK_DIST_CM = 200.0
STUCK_BASELINE_CM = 20.0
STUCK_FLOW_PX     = 3.0

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
                # pipe.infer() may return lists instead of ndarray; normalise here
                return {k: np.array(v) if not isinstance(v, np.ndarray) else v
                        for k, v in raw.items()}
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
      fast_depth    — 0.7     (metric depth, primary sensor, highest trust)
      optical flow  — 0.2–0.7 (scales with baseline_cm / 25, capped at 0.7)
      known height  — 0.3     (assumes typical object size, lowest trust)
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
def _open_space_from_depth(depth_map: np.ndarray) -> dict:
    """
    Fraction of far pixels (potential open space / doorway) in each frame third.
    Returns {"left": float, "center": float, "right": float} in [0, 1].
    """
    threshold = float(np.percentile(depth_map, 40))   # low inv-depth = far
    far   = depth_map <= threshold
    w     = depth_map.shape[1]
    third = w // 3
    return {
        "left":   float(far[:, :third].mean()),
        "center": float(far[:, third:2 * third].mean()),
        "right":  float(far[:, 2 * third:].mean()),
    }


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

        at_floor     = y2 > floor_threshold
        within_range = dist is None or dist < OBSTACLE_BLOCK_DIST_CM
        blocking     = at_floor and within_range

        enriched.append({**det, "blocking": blocking, "distance_cm": dist, "real_height_cm": real_h})
        if blocking:
            blocked[det["position"]] = True
            if dist is not None and (nearest_cm is None or dist < nearest_cm):
                nearest_cm = dist
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

    # 3. YOLO detection
    detections: list[dict] = []
    raw = _hailo_infer("yolo_det", img_bgr)
    if raw is not None:
        detections = _parse_yolo_boxes(raw, orig_w, orig_h, depth_map, scene_depth_cm,
                                       flow_field, baseline_cm)

    # 4. Obstacle map
    obstacles = analyze_obstacles(
        detections, orig_w, orig_h, scene_depth_cm, flow_field, baseline_cm
    )

    # 5. Open-space map from depth
    open_space = (
        _open_space_from_depth(depth_map)
        if depth_map is not None
        else {"left": 0.0, "center": 0.0, "right": 0.0}
    )

    # 6. Door detection — OpenCV geometry + YOLO panel, fused
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
