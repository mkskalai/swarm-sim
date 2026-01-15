"""
YOLOv11 object detector with GPU/CPU fallback.

Uses Ultralytics YOLO for inference on drone camera images.
Targets: person (0), car (2), motorcycle (3), bus (5), truck (7)
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)

# Target COCO classes for surveillance
TARGET_CLASSES: dict[int, str] = {
    0: "person",
    2: "car",
    3: "motorcycle",
    5: "bus",
    7: "truck",
}


@dataclass
class DetectorConfig:
    """Configuration for YOLO detector."""

    model_name: str = "yolo11n.pt"  # nano model for speed
    confidence_threshold: float = 0.4
    iou_threshold: float = 0.5
    target_classes: dict[int, str] = field(default_factory=lambda: TARGET_CLASSES.copy())
    use_gpu: bool = True  # Will fallback to CPU if GPU unavailable
    image_size: int = 640  # Input size for YOLO
    half_precision: bool = True  # FP16 on GPU for speed


@dataclass
class Detection:
    """Single detection result."""

    class_id: int
    class_name: str
    confidence: float
    bbox: tuple[int, int, int, int]  # x, y, w, h (top-left corner + size)
    track_id: int = 0  # Set by tracker, 0 = untracked


class YOLODetector:
    """
    YOLOv11 detector with automatic GPU/CPU selection.

    Usage:
        detector = YOLODetector()
        if detector.initialize():
            detections, inference_ms = detector.detect(image)
    """

    def __init__(self, config: Optional[DetectorConfig] = None):
        self.config = config or DetectorConfig()
        self._model = None
        self._device: Optional[str] = None
        self._initialized = False

    def initialize(self) -> bool:
        """
        Load model and detect device.

        Returns:
            True if successful, False otherwise.
        """
        try:
            from ultralytics import YOLO
        except ImportError:
            logger.error(
                "ultralytics not installed. Run: pip install ultralytics"
            )
            return False

        try:
            import torch

            # Detect device
            if self.config.use_gpu and torch.cuda.is_available():
                self._device = "cuda"
                gpu_name = torch.cuda.get_device_name(0)
                logger.info(f"Using GPU: {gpu_name}")
            else:
                self._device = "cpu"
                if self.config.use_gpu:
                    logger.info("GPU requested but not available, using CPU")
                else:
                    logger.info("Using CPU for inference")

            # Load model
            logger.info(f"Loading model: {self.config.model_name}")
            self._model = YOLO(self.config.model_name)
            self._model.to(self._device)

            # Warmup inference with dummy image
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            self._model.predict(dummy, verbose=False)

            self._initialized = True
            logger.info(
                f"YOLOv11 initialized on {self._device} "
                f"(half={self.config.half_precision and self._device == 'cuda'})"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to initialize YOLO: {e}")
            return False

    def detect(self, image: np.ndarray) -> tuple[list[Detection], float]:
        """
        Run detection on image.

        Args:
            image: BGR image (H, W, 3) numpy array from OpenCV

        Returns:
            Tuple of (list of Detection objects, inference_time_ms)

        Raises:
            RuntimeError: If detector not initialized
        """
        if not self._initialized or self._model is None:
            raise RuntimeError("Detector not initialized. Call initialize() first.")

        import time

        start = time.perf_counter()

        # Run inference
        results = self._model.predict(
            image,
            conf=self.config.confidence_threshold,
            iou=self.config.iou_threshold,
            classes=list(self.config.target_classes.keys()),
            verbose=False,
            half=self.config.half_precision and self._device == "cuda",
        )

        inference_ms = (time.perf_counter() - start) * 1000

        # Parse results
        detections: list[Detection] = []
        if len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes
            for i in range(len(boxes)):
                cls_id = int(boxes.cls[i].item())
                conf = float(boxes.conf[i].item())

                # Convert from xyxy to xywh
                x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy()
                bbox = (int(x1), int(y1), int(x2 - x1), int(y2 - y1))

                detections.append(
                    Detection(
                        class_id=cls_id,
                        class_name=self.config.target_classes.get(
                            cls_id, f"class_{cls_id}"
                        ),
                        confidence=conf,
                        bbox=bbox,
                    )
                )

        return detections, inference_ms

    @property
    def device(self) -> str:
        """Get current device (cuda/cpu)."""
        return self._device or "uninitialized"

    @property
    def is_initialized(self) -> bool:
        """Check if detector is ready."""
        return self._initialized

    def get_stats(self) -> dict:
        """Get detector statistics."""
        return {
            "device": self._device,
            "initialized": self._initialized,
            "model": self.config.model_name,
            "target_classes": list(self.config.target_classes.values()),
            "confidence_threshold": self.config.confidence_threshold,
        }
