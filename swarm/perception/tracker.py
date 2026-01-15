"""
Simple IoU-based tracker for maintaining track IDs across frames.

This tracker uses Intersection over Union (IoU) to match detections
across consecutive frames and maintain consistent track IDs.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .detector import Detection


@dataclass
class TrackerConfig:
    """Configuration for the tracker."""

    max_age: int = 30  # Frames before track is deleted
    min_hits: int = 3  # Minimum detections before track is confirmed
    iou_threshold: float = 0.3  # Minimum IoU for matching


@dataclass
class Track:
    """Single object track."""

    track_id: int
    class_id: int
    class_name: str
    bbox: tuple[int, int, int, int]  # x, y, w, h
    confidence: float
    hits: int = 1
    age: int = 0
    is_confirmed: bool = False


class SimpleTracker:
    """
    IoU-based multi-object tracker.

    Maintains track IDs across frames by matching detections using
    Intersection over Union (IoU) metric.

    Usage:
        tracker = SimpleTracker()

        # Each frame:
        detections = detector.detect(image)
        tracks = tracker.update(detections)

        for track in tracks:
            print(f"Track {track.track_id}: {track.class_name}")
    """

    def __init__(self, config: Optional[TrackerConfig] = None):
        self.config = config or TrackerConfig()
        self._tracks: dict[int, Track] = {}
        self._next_id = 1

    def update(self, detections: list[Detection]) -> list[Track]:
        """
        Update tracks with new detections.

        Args:
            detections: List of Detection objects from detector

        Returns:
            List of confirmed Track objects with assigned track_ids
        """
        # Age all existing tracks
        for track in self._tracks.values():
            track.age += 1

        if not detections:
            self._remove_stale_tracks()
            return [t for t in self._tracks.values() if t.is_confirmed]

        # Build cost matrix (IoU)
        det_bboxes = [d.bbox for d in detections]
        track_ids = list(self._tracks.keys())
        track_bboxes = [self._tracks[tid].bbox for tid in track_ids]

        if track_bboxes:
            iou_matrix = self._compute_iou_matrix(det_bboxes, track_bboxes)
            matches, unmatched_dets, unmatched_tracks = self._match(
                iou_matrix, track_ids, len(detections)
            )
        else:
            matches = []
            unmatched_dets = list(range(len(detections)))
            unmatched_tracks = []

        # Update matched tracks
        for det_idx, track_id in matches:
            det = detections[det_idx]
            track = self._tracks[track_id]
            track.bbox = det.bbox
            track.confidence = det.confidence
            track.hits += 1
            track.age = 0
            if track.hits >= self.config.min_hits:
                track.is_confirmed = True

        # Create new tracks for unmatched detections
        for det_idx in unmatched_dets:
            det = detections[det_idx]
            track = Track(
                track_id=self._next_id,
                class_id=det.class_id,
                class_name=det.class_name,
                bbox=det.bbox,
                confidence=det.confidence,
            )
            self._tracks[self._next_id] = track
            self._next_id += 1

        self._remove_stale_tracks()

        # Return only confirmed tracks
        return [t for t in self._tracks.values() if t.is_confirmed]

    def _compute_iou_matrix(
        self,
        det_bboxes: list[tuple[int, int, int, int]],
        track_bboxes: list[tuple[int, int, int, int]],
    ) -> np.ndarray:
        """Compute IoU between all detection-track pairs."""
        matrix = np.zeros((len(det_bboxes), len(track_bboxes)))
        for i, db in enumerate(det_bboxes):
            for j, tb in enumerate(track_bboxes):
                matrix[i, j] = self._iou(db, tb)
        return matrix

    def _iou(
        self,
        box1: tuple[int, int, int, int],
        box2: tuple[int, int, int, int],
    ) -> float:
        """
        Calculate IoU between two boxes.

        Args:
            box1, box2: Boxes in (x, y, w, h) format

        Returns:
            IoU value between 0 and 1
        """
        x1, y1, w1, h1 = box1
        x2, y2, w2, h2 = box2

        # Calculate intersection
        xi = max(x1, x2)
        yi = max(y1, y2)
        xf = min(x1 + w1, x2 + w2)
        yf = min(y1 + h1, y2 + h2)

        if xf <= xi or yf <= yi:
            return 0.0

        inter = (xf - xi) * (yf - yi)
        union = w1 * h1 + w2 * h2 - inter

        return inter / union if union > 0 else 0.0

    def _match(
        self,
        iou_matrix: np.ndarray,
        track_ids: list[int],
        num_detections: int,
    ) -> tuple[list[tuple[int, int]], list[int], list[int]]:
        """
        Greedy matching based on IoU.

        Returns:
            Tuple of (matches, unmatched_detections, unmatched_tracks)
            where matches is list of (detection_idx, track_id) pairs
        """
        matches: list[tuple[int, int]] = []
        unmatched_dets = list(range(num_detections))
        unmatched_track_indices = list(range(len(track_ids)))

        # Create working copy
        matrix = iou_matrix.copy()

        # Greedy matching by highest IoU
        while matrix.size > 0:
            # Find maximum IoU
            max_idx = np.unravel_index(np.argmax(matrix), matrix.shape)
            max_iou = matrix[max_idx]

            if max_iou < self.config.iou_threshold:
                break

            det_idx, track_idx = max_idx

            # Map back to original indices
            original_det_idx = unmatched_dets[det_idx]
            original_track_id = track_ids[unmatched_track_indices[track_idx]]

            matches.append((original_det_idx, original_track_id))

            # Remove matched from consideration
            unmatched_dets.pop(det_idx)
            unmatched_track_indices.pop(track_idx)
            matrix = np.delete(matrix, det_idx, axis=0)
            matrix = np.delete(matrix, track_idx, axis=1)

        # Convert remaining track indices to track IDs
        unmatched_tracks = [track_ids[i] for i in unmatched_track_indices]

        return matches, unmatched_dets, unmatched_tracks

    def _remove_stale_tracks(self) -> None:
        """Remove tracks that haven't been updated recently."""
        to_remove = [
            tid
            for tid, track in self._tracks.items()
            if track.age > self.config.max_age
        ]
        for tid in to_remove:
            del self._tracks[tid]

    def reset(self) -> None:
        """Clear all tracks and reset ID counter."""
        self._tracks.clear()
        self._next_id = 1

    def get_all_tracks(self) -> list[Track]:
        """Get all tracks including unconfirmed ones."""
        return list(self._tracks.values())

    def get_track(self, track_id: int) -> Optional[Track]:
        """Get a specific track by ID."""
        return self._tracks.get(track_id)

    def get_stats(self) -> dict:
        """Get tracker statistics."""
        confirmed = sum(1 for t in self._tracks.values() if t.is_confirmed)
        return {
            "total_tracks": len(self._tracks),
            "confirmed_tracks": confirmed,
            "next_id": self._next_id,
        }
