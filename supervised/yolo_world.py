"""Open-vocabulary object detector using YOLO-World with MPS hardware acceleration."""
import logging
from pathlib import Path
from typing import List, Dict, Any, Optional
import numpy as np


DEFAULT_VOCABULARY = [
    "chair", "armchair", "doorway", "door", "table", "couch", "person"
]


class YoloWorldDetector:
    def __init__(
        self,
        model_path: str = "models/yolov8s-worldv2.pt",
        device: str = "mps",
        classes: Optional[List[str]] = None,
        conf_threshold: float = 0.25
    ):
        self.model_path = Path(model_path)
        self.device = device
        self.conf_threshold = conf_threshold
        self.classes = list(classes) if classes else list(DEFAULT_VOCABULARY)
        self.model = None
        self._load_model()

    def _load_model(self):
        try:
            import torch
            from ultralytics import YOLOWorld

            # Check if MPS is available
            target_device = self.device
            if target_device == "mps" and not torch.backends.mps.is_available():
                logging.warning("MPS device requested but not available; falling back to CPU")
                target_device = "cpu"

            self.model = YOLOWorld(str(self.model_path))
            self.model.set_classes(self.classes)
            self.device = target_device
            logging.info("YOLO-World loaded from %s on device=%s with %d classes",
                         self.model_path, self.device, len(self.classes))
        except Exception as error:
            logging.error("Failed to load YOLO-World model: %s", error)
            self.model = None

    def set_classes(self, classes: List[str]):
        """Dynamically update the text vocabulary for zero-shot detection."""
        self.classes = list(classes)
        if self.model is not None:
            self.model.set_classes(self.classes)

    def detect(self, rgb_image: np.ndarray, conf: Optional[float] = None) -> List[Dict[str, Any]]:
        """Run zero-shot open-vocabulary detection on an RGB image array."""
        if self.model is None:
            return []

        threshold = conf if conf is not None else self.conf_threshold
        # Convert RGB to BGR for OpenCV / Ultralytics
        bgr = rgb_image[:, :, ::-1] if rgb_image.ndim == 3 and rgb_image.shape[2] == 3 else rgb_image

        results = self.model.predict(
            bgr,
            device=self.device,
            imgsz=640,
            conf=threshold,
            verbose=False
        )

        detections = []
        if not results or len(results) == 0:
            return detections

        boxes_obj = results[0].boxes
        if boxes_obj is None or len(boxes_obj) == 0:
            return detections

        xyxy = boxes_obj.xyxy.cpu().numpy()
        confs = boxes_obj.conf.cpu().numpy()
        cls_ids = boxes_obj.cls.cpu().numpy().astype(int)

        for i in range(len(xyxy)):
            box = xyxy[i].tolist()
            confidence = float(confs[i])
            cls_id = int(cls_ids[i])
            label = self.classes[cls_id] if 0 <= cls_id < len(self.classes) else "unknown"
            detections.append({
                "label": label,
                "confidence": round(confidence, 3),
                "box": [round(c, 1) for c in box]
            })

        return detections

    def detect_with_geometry(
        self,
        rgb_image: np.ndarray,
        contact_geometry,
        floor_mask: Optional[np.ndarray] = None,
        pitch_delta_rad: float = 0.0,
        conf: Optional[float] = None
    ) -> List[Dict[str, Any]]:
        """Run detection and compute metric 3D ground coordinates for each object."""
        detections = self.detect(rgb_image, conf=conf)
        for det in detections:
            spatial = contact_geometry.evaluate_object(
                det["box"],
                floor_mask=floor_mask,
                pitch_delta_rad=pitch_delta_rad
            )
            det["spatial"] = spatial
        return detections
