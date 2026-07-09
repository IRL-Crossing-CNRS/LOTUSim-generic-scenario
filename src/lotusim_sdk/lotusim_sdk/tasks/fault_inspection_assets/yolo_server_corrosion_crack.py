"""
Detection module for corrosion and crack analysis.

Provides:
  - HSV colour-based corrosion detection  (detect_corrosion_by_color)
  - YOLO crack detection                  (yolo_model)
  - Debug-window annotation               (draw_detections)

Previously a standalone Flask HTTP server; now imported directly by
FaultInspectionTask (lotusim_sdk.tasks.fault_inspection) as the detection
back-end for a ROS 2 pub/sub pipeline.
"""

from ultralytics import YOLO
import numpy as np
import cv2
import torch
import os

# ─── PyTorch 2.6 fix ──────────────────────────────────────────────────────────
_original_torch_load = torch.load
def _patched_torch_load(f, *args, **kwargs):
    kwargs.setdefault("weights_only", False)
    return _original_torch_load(f, *args, **kwargs)
torch.load = _patched_torch_load
# ──────────────────────────────────────────────────────────────────────────────

# ─── Configuration ────────────────────────────────────────────────────────────

# --- YOLO crack detection (optional, set to None to disable) ---
YOLO_MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "crack.pt")
YOLO_CONFIDENCE = 0.05

# --- Color-based corrosion detection ---
COLOR_DETECTION_ENABLED = True

# Tuned for dark red-brown rust on yellow wind turbine base, underwater blue tint
# Range 1: dark red-brown (main rust color, low brightness)
HSV_LOWER_1 = np.array([0,   100,  15])
HSV_UPPER_1 = np.array([15,  255, 140])
# Range 2: red wraps in HSV (165-180)
HSV_LOWER_2 = np.array([165, 100,  15])
HSV_UPPER_2 = np.array([180, 255, 140])
# Range 3: brownish-orange darker tones
HSV_LOWER_3 = np.array([5,   50,  15])
HSV_UPPER_3 = np.array([25,  220, 130])

# Minimum area in pixels — increase if too many false positives
MIN_CONTOUR_AREA = 400

SHOW_WINDOW = False
# ──────────────────────────────────────────────────────────────────────────────

# Load YOLO if enabled — runs once at import time; Python's module cache prevents
# reloading across multiple FaultInspectionTask instances or on_enter re-activations.
yolo_model = None
if YOLO_MODEL_PATH:
    print(f"[Detection] Loading YOLO model: {YOLO_MODEL_PATH}")
    yolo_model = YOLO(YOLO_MODEL_PATH)
    print(f"[Detection] YOLO ready. Classes: {list(yolo_model.names.values())}")

print("[Detection] Color-based corrosion detection: ENABLED" if COLOR_DETECTION_ENABLED else "")


def detect_corrosion_by_color(pil_image):
    """
    Detect rust/corrosion patches by HSV color range.
    Returns list of detections with bounding boxes.
    """
    img_bgr = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # Combine all 3 rust color ranges
    mask1 = cv2.inRange(img_hsv, HSV_LOWER_1, HSV_UPPER_1)
    mask2 = cv2.inRange(img_hsv, HSV_LOWER_2, HSV_UPPER_2)
    mask3 = cv2.inRange(img_hsv, HSV_LOWER_3, HSV_UPPER_3)
    mask = cv2.bitwise_or(mask1, cv2.bitwise_or(mask2, mask3))

    # Clean up noise with morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # remove small dots
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # fill small holes
    mask = cv2.dilate(mask, kernel, iterations=1)

    # Find contours of corrosion patches
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detections = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_CONTOUR_AREA:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        # Confidence proxy: larger area = higher "confidence"
        conf = min(1.0, area / 5000.0)

        detections.append({
            "label": "corrosion",
            "confidence": round(conf, 3),
            "x": x, "y": y, "w": w, "h": h,
            "cx": x + w // 2, "cy": y + h // 2,
            "source": "color"
        })

    return detections, mask


def draw_detections(img_np, detections, mask=None):
    """Draw bounding boxes and labels on a numpy BGR image."""

    # Overlay the color mask in translucent red so you can see what it's picking up
    if mask is not None:
        overlay = img_np.copy()
        overlay[mask > 0] = (0, 0, 180)
        img_np = cv2.addWeighted(img_np, 0.7, overlay, 0.3, 0)

    for det in detections:
        x, y, w, h = det["x"], det["y"], det["w"], det["h"]
        label = det["label"]
        conf = det["confidence"]
        source = det.get("source", "yolo")

        if source == "color":
            color = (0, 200, 255)   # yellow-orange for corrosion
        elif conf > 0.7:
            color = (0, 255, 0)
        elif conf > 0.5:
            color = (0, 165, 255)
        else:
            color = (0, 0, 255)

        cv2.rectangle(img_np, (x, y), (x + w, y + h), color, 2)

        #text = f"{label} [{source}] {conf:.0%}"
        #text = f"{label} {conf:.0%}"
        text = f"{label}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img_np, (x, y - th - 6), (x + tw + 4, y), color, -1)
        cv2.putText(img_np, text, (x + 2, y - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    color_count = sum(1 for d in detections if d.get("source") == "color")
    yolo_count  = sum(1 for d in detections if d.get("source") != "color")
    cv2.putText(img_np, f"Corrosion: {color_count} | Cracks: {yolo_count}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
    return img_np
