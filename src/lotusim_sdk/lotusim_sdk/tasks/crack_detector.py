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


_det = None
_det_model = None
_det_load_lock = threading.Lock()
_det_loading = False


def _ensure_detection_module_loading(logger=None,model="simulation") -> None:
    """Start loading the YOLO detection module on a background thread (once)."""
    global _det_loading, _det_model
    model = str(model).strip().lower()
    if model not in {"simulation", "real"}:
        if logger is not None:
            logger.warning(
                f"CrackDetector: unknown model={model!r}; using 'simulation'."
            )
        model = "simulation"
    with _det_load_lock:
        if _det is not None and _det_model == model:
            return
        if _det_loading:
            return
        _det_loading = True

    def _load() -> None:
        global _det, _det_loading, _det_model
        try:
            from lotusim_sdk.tasks.crack_detector_assets import (
                yolo_server_crack as _loaded,
            )
            _loaded.yolo_model = _loaded.load_yolo_model(model)
            _det = _loaded
            _det_model = model
            if logger is not None:
                logger.info(f"CrackDetector: loaded {model!r} YOLO model.")
        except Exception as e:  # noqa: BLE001 - surface any load failure, keep serving
            if logger is not None:
                logger.error(
                    f"CrackDetector: failed to load detection module: {e}"
                )
        finally:
            with _det_load_lock:
                _det_loading = False

    threading.Thread(target=_load, name="yolo-model-load", daemon=True).start()

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
        target=_display_loop, name="crack-detector-display", daemon=True
    ).start()


def _show_frame(title: str, frame) -> None:
    with _display_lock:
        _display_frames[title] = frame


def _close_window(title: str) -> None:
    with _display_lock:
        _display_frames.pop(title, None)
        _display_closing.add(title)



class CrackDetector(TaskAgent):

    
    def __init__(self,host,params=None,blackboard=None,id: str="") -> None:
        super().__init__(host,params,blackboard,id)








        self._show_window: bool = bool(self.params.get("show_window", False))

        # ``model`` is the public mission parameter; keep ``using_model`` as
        # a compatibility alias for existing scenarios.
        self.using_model = self.params.get(
            "model", self.params.get("using_model", "simulation")
        )

        self._window_title: str = ""

        

        self._image_sub = None
        self._detections_pub = None


    def on_enter(self) -> None:

        #on enter we want to ensure that the module is loading
        # The detection module is loaded on the first frame, not here.

        #set the world name
        world = self.host.world_name
        #set the agent name
        agent = self.host.agent_name

        #create a subscription using self.host to /inspection/image
        self._image_sub = self.host.create_subscription(
            CompressedImage,
            f"/{world}/{agent}/inspection/image",
            self._image_callback,
            _IMAGE_QOS,
        )

        #we want to publish our detections, so create a publisher to /inspection/detections
        self._detections_pub = self.host.create_publisher(
            String,
            f"{world}/{agent}/inspection/detections",
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        #if we want to show window show window
        if self._show_window:
            self._window_title = f"{agent} - Crack Detector - YOLO + CLAHE"
            _ensure_display_thread()

        self.host.get_logger().info(
            f"Crack Detector (YOLO + CLAHE): listening on/{world}/{agent}/inspection/image"
        )


    def update(self) -> Status:
        return Status.RUNNING

    def on_exit(self, _status: Status) -> None:

        if self._image_sub is not None:
            #destroy sub
            self.host.destroy_subscription(self._image_sub)
            self._image_sub = None
        if self._detections_pub is not None:
            #destroy publisher
            self.host.destroy_publisher(self._detections_pub)
            self._detections_pub = None
        if self._window_title:
            #destroy window
            _close_window(self._window_title)



    def _image_callback(self, msg:CompressedImage) -> None:
        if _det is None or _det_model != str(self.using_model).strip().lower():
            _ensure_detection_module_loading(self.host.get_logger(), self.using_model)
        if _det is None or self._detections_pub is None:
            return


        detections = []
        compressed_bytes = np.frombuffer(msg.data,dtype=np.uint8)

        color_image = cv2.imdecode(compressed_bytes,cv2.IMREAD_COLOR)
        image = cv2.imdecode(compressed_bytes,cv2.IMREAD_GRAYSCALE)
        if image is None:
            self.host.get_logger().error("Failed to decode compressed image")
            return


        
        #apply CLAHE to enchance the contrast of the image 
        #FOR UNDERWATER USE ONLY. HARDCODED FOR NOW. 
        # TODO: APPLY THIS ONLY IF THE IMAGE IS UNDERWATER
        clahe = cv2.createCLAHE(clipLimit=2, tileGridSize=(8,8))
        clahe_image = clahe.apply(image)
        blurred_image = cv2.GaussianBlur(clahe_image, (13,13),0)

        final_image = cv2.addWeighted(clahe_image,2,blurred_image,-1,0)

        #final_image = image


        if _det.yolo_model is not None:
            with _yolo_lock:
                # NOTE: device=0 pins inference to the first CUDA GPU, so this
                # task REQUIRES a CUDA-capable machine — it raises on a CPU-only
                # host rather than falling back. imgsz/conf are the values the
                # checkpoints were tuned against; they are not scenario params.
                results = _det.yolo_model.predict(
                    source=final_image,
                    imgsz = 832,
                    conf = 0.42,
                    device = 0
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
            f"CrackDetector: {len(detections)} detection(s): "
            f"{[(d['label'], d['confidence'], d['source']) for d in detections]}"
        )

        out = String()
        out.data = json.dumps({"detections": detections, "count": len(detections)})
        self._detections_pub.publish(out)


        if self._show_window:
            annotated = _det.draw_detections(color_image.copy(),detections,None)
            _show_frame(self._window_title,annotated)

        


        
