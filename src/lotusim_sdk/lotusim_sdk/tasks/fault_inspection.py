from __future__ import annotations

import json
import threading

import cv2
import numpy as np
from PIL import Image

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent

# Populated on first on_enter; Python's import cache ensures YOLO loads only once
# even when multiple FaultInspectionTask instances exist (e.g. one per agent).
_det = None

# run_agent uses MultiThreadedExecutor, so multiple agents' _image_callbacks can
# fire concurrently.  YOLO inference is not thread-safe — serialize with this lock.
# Module-level so it is shared across all instances (one lock → one YOLO model).
_yolo_lock = threading.Lock()

# Sensor-data QoS for incoming camera frames: drop stale frames, keep only latest.
_IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ── Debug-window display manager ──────────────────────────────────────────────
# cv2 HighGUI (Qt) is NOT thread-safe and its event loop must be pumped from ONE
# dedicated thread.  Our _image_callback runs inside MultiThreadedExecutor worker
# threads (run_agent.py) — a different worker can service each frame — so calling
# imshow/waitKey straight from the callback drives Qt from shifting threads, its
# event queue never drains, memory grows unbounded and the OOM killer fires
# ("Killed").  Instead callbacks hand annotated frames to this single global
# display thread, which owns every window and pumps waitKey for the whole process
# (also handles N agents → N windows correctly, one GUI thread for all of them).
_display_lock = threading.Lock()
_display_frames: dict = {}      # window title -> latest annotated BGR frame
_display_closing: set = set()   # window titles pending destruction
_display_started = False


def _display_loop() -> None:
    while True:
        with _display_lock:
            frames = dict(_display_frames)
            closing = list(_display_closing)
            _display_closing.clear()
        for title in closing:
            try:
                cv2.destroyWindow(title)
            except Exception:
                pass
        for title, frame in frames.items():
            try:
                cv2.imshow(title, frame)
            except Exception:
                pass
        # Pump the Qt event loop every ~30 ms even when no new frame arrived.
        cv2.waitKey(30)


def _ensure_display_thread() -> None:
    global _display_started
    with _display_lock:
        if _display_started:
            return
        _display_started = True
    threading.Thread(
        target=_display_loop, name="fault-inspection-display", daemon=True
    ).start()


def _show_frame(title: str, frame) -> None:
    with _display_lock:
        _display_frames[title] = frame


def _close_window(title: str) -> None:
    with _display_lock:
        _display_frames.pop(title, None)
        _display_closing.add(title)


class FaultInspectionTask(TaskAgent):
    """Run YOLO corrosion/crack detection over ROS 2 for the duration of a mission.

    Folded into the BT lifecycle — no separate rclpy node.  ``self.host`` is
    already the spinning rclpy.Node, so ``create_subscription`` /
    ``create_publisher`` on it are dispatched by the same executor that ticks
    missions (identical to the ``CheckBatteryStateTask`` pattern).

    - ``on_enter``       loads the detection module (once), creates subscriber +
                         publisher.
    - ``_image_callback`` decodes each CompressedImage JPEG, runs HSV corrosion
                         detection and YOLO crack inference, publishes JSON.
    - ``on_exit``        destroys subscriber + publisher.
    - ``update``         always returns RUNNING — the leaf is event-driven and
                         stays alive until the mission halts it.

    Topics (per agent):
        SUB  /{world}/{agent}/inspection/image       sensor_msgs/CompressedImage
        PUB  /{world}/{agent}/inspection/detections  std_msgs/String  (JSON)

    JSON shape published (Unity keys overlay colours and labels off these fields):
        {"detections": [
            {"label": "corrosion", "confidence": 0.83,
             "x": 10, "y": 20, "w": 30, "h": 40, "cx": 25, "cy": 40,
             "source": "color"},
            {"label": "crack", "confidence": 0.61,
             "x": ..., "source": "yolo"}
          ], "count": 2}
    """

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        # show_window=true opens a local cv2 debug window; off by default so the
        # task works safely on headless and remote machines.
        self._show_window: bool = bool(self.params.get("show_window", False))
        self._window_title: str = ""
        self._image_sub = None
        self._detections_pub = None

    # -- lifecycle ------------------------------------------------------------

    def on_enter(self) -> None:
        global _det
        if _det is None:
            try:
                from lotusim_sdk.tasks.fault_inspection_assets import (
                    yolo_server_corrosion_crack as _loaded,
                )
                _det = _loaded
            except Exception as e:
                self.host.get_logger().error(
                    f"FaultInspectionTask: failed to load detection module: {e}"
                )
                return

        world = self.host.world_name
        # Use agent_name (the entity/topic name), NOT get_name() (the rclpy node
        # name, frozen at construction). When the host deconflicts a spawn name
        # collision it returns a different name that the client adopts into
        # agent_name; the node name can no longer be changed. Building topics from
        # agent_name keeps them routed to the real entity. on_enter() runs only
        # after missions start, which is gated on the pose arriving for agent_name,
        # so by here agent_name already holds the host-assigned name.
        agent = self.host.agent_name

        self._image_sub = self.host.create_subscription(
            CompressedImage,
            f"/{world}/{agent}/inspection/image",
            self._image_callback,
            _IMAGE_QOS,
        )
        # TRANSIENT_LOCAL so Unity receives the last detection burst even if it
        # subscribes slightly after the first frames arrive (matches the QoS Unity
        # declares on this topic — VOLATILE publisher would be rejected).
        self._detections_pub = self.host.create_publisher(
            String,
            f"/{world}/{agent}/inspection/detections",
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        # Debug window: drive it from a single dedicated GUI thread, never from
        # the executor worker threads that run this callback (see display manager).
        if self._show_window:
            self._window_title = f"{agent} - Corrosion + Crack Detection"
            _ensure_display_thread()

        self.host.get_logger().info(
            f"FaultInspectionTask: listening on /{world}/{agent}/inspection/image"
        )

    def update(self) -> Status:
        return Status.RUNNING

    def on_exit(self, _status: Status) -> None:
        if self._image_sub is not None:
            self.host.destroy_subscription(self._image_sub)
            self._image_sub = None
        if self._detections_pub is not None:
            self.host.destroy_publisher(self._detections_pub)
            self._detections_pub = None
        if self._window_title:
            _close_window(self._window_title)

    # -- detection callback ---------------------------------------------------

    def _image_callback(self, msg: CompressedImage) -> None:
        if _det is None or self._detections_pub is None:
            return

        # Decode JPEG → BGR
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img_bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if img_bgr is None:
            return

        # Convert BGR → PIL RGB — detection functions keep their original signature
        pil_image = Image.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))

        detections = []
        mask = None

        # 1) Color-based corrosion detection (OpenCV-only, thread-safe)
        if _det.COLOR_DETECTION_ENABLED:
            color_detections, mask = _det.detect_corrosion_by_color(pil_image)
            detections.extend(color_detections)

        # 2) YOLO crack detection — serialize across concurrent agent callbacks
        if _det.yolo_model is not None:
            with _yolo_lock:
                results = _det.yolo_model.predict(
                    source=pil_image,
                    conf=_det.YOLO_CONFIDENCE,
                    iou=0.4,
                    verbose=False,
                )
            for result in results:
                if result.boxes is None:
                    continue
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    label = _det.yolo_model.names[cls_id]
                    detections.append({
                        "label": label,
                        "confidence": round(conf, 3),
                        "x": int(x1), "y": int(y1),
                        "w": int(x2 - x1), "h": int(y2 - y1),
                        "cx": int((x1 + x2) / 2), "cy": int((y1 + y2) / 2),
                        "source": "yolo",
                    })

        self.host.get_logger().debug(
            f"FaultInspectionTask: {len(detections)} detection(s): "
            f"{[(d['label'], d['confidence'], d['source']) for d in detections]}"
        )

        out = String()
        out.data = json.dumps({"detections": detections, "count": len(detections)})
        self._detections_pub.publish(out)

        # Optional debug window — enable via params {"show_window": true} in the
        # mission JSON.  Hand the annotated frame to the global display thread; we
        # must NOT call imshow/waitKey here (this runs in an executor worker thread).
        if self._show_window:
            annotated = _det.draw_detections(img_bgr.copy(), detections, mask)
            _show_frame(self._window_title, annotated)
