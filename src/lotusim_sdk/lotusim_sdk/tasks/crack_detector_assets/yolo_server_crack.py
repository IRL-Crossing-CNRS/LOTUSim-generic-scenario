from ultralytics import YOLO
import numpy as np
import cv2
import torch
import os

_original_torch_load = torch.load


def _patched_torch_load(f, *args, **kwargs):
    kwargs.setdefault("weights_only", False)
    return _original_torch_load(f, *args, **kwargs)


torch.load = _patched_torch_load

MODEL_FILES = {
    "simulation": "finetuned.pt",
    "real": "best.pt",
}


def load_yolo_model(model="simulation"):
    """Load the checkpoint selected by the inspection environment."""
    model = str(model).strip().lower()
    try:
        filename = MODEL_FILES[model]
    except KeyError as exc:
        raise ValueError(f"Unknown crack detector model {model!r}; choose 'simulation' or 'real'.") from exc

    model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
    print(f"[DETECTION] Loading YOLO model ({model}): {model_path}")
    loaded = YOLO(model_path)
    print(f"[DETECTION] YOLO ready ({model}).")
    return loaded


# Populated by CrackDetector's background loader after it has selected a model.
yolo_model = None


def applyClahe(image, clipLimit=2.0, tileGridSize=(8, 8)):
    clahe = cv2.createCLAHE(clipLimit=clipLimit, tileGridSize=tileGridSize)
    return clahe.apply(image)


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
            color = (0, 200, 255)  # yellow-orange for corrosion
        elif conf > 0.7:
            color = (0, 255, 0)
        elif conf > 0.5:
            color = (0, 165, 255)
        else:
            color = (0, 0, 255)

        color = (0, 0, 255)

        cv2.rectangle(img_np, (x, y), (x + w, y + h), color, 2)

        text = f"{label} {conf:.0%}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img_np, (x, y - th - 6), (x + tw + 4, y), color, -1)
        cv2.putText(img_np, text, (x + 2, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    color_count = sum(1 for d in detections if d.get("source") == "color")
    yolo_count = sum(1 for d in detections if d.get("source") != "color")
    cv2.putText(
        img_np,
        f"Corrosion: {color_count} | Cracks: {yolo_count}",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0),
        2,
    )
    return img_np
