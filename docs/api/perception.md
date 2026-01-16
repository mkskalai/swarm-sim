# Perception API Reference

The `swarm.perception` module provides object detection and tracking.

## YOLODetector

YOLOv11 object detection wrapper with GPU/CPU fallback.

**Location:** `swarm/perception/detector.py`

```python
from swarm.perception import YOLODetector, DetectorConfig
import cv2

config = DetectorConfig(
    model_name="yolo11n.pt",
    confidence_threshold=0.4,
    use_gpu=True,
)
detector = YOLODetector(config)
detector.initialize()

# Detect objects in image
image = cv2.imread("test.jpg")
detections, inference_ms = detector.detect(image)

for det in detections:
    print(f"{det.class_name}: {det.confidence:.2f} at {det.bbox}")
```

### Methods

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `initialize()` | - | `bool` | Load model and warm up |
| `detect(image)` | `np.ndarray (BGR)` | `tuple[list[Detection], float]` | Run detection, return detections and inference time |
| `detect_batch(images)` | `list[np.ndarray]` | `list[list[Detection]]` | Batch detection |

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `is_initialized` | `bool` | Model loaded |
| `device` | `str` | "cuda" or "cpu" |
| `model_name` | `str` | Model filename |

---

## DetectorConfig

Configuration for YOLODetector.

**Location:** `swarm/perception/detector.py`

```python
from swarm.perception import DetectorConfig

config = DetectorConfig(
    model_name="yolo11n.pt",
    confidence_threshold=0.4,
    nms_threshold=0.5,
    use_gpu=True,
    target_classes=[0, 2, 3, 5, 7],  # person, car, motorcycle, bus, truck
)
```

### Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `model_name` | `str` | `"yolo11n.pt"` | YOLO model to load |
| `confidence_threshold` | `float` | `0.4` | Minimum confidence |
| `nms_threshold` | `float` | `0.5` | NMS IoU threshold |
| `use_gpu` | `bool` | `True` | Use GPU if available |
| `target_classes` | `list[int]` | `[0,2,3,5,7]` | COCO classes to detect |
| `image_size` | `int` | `640` | Input image size |

### Available Models

| Model | Size | Speed | Accuracy |
|-------|------|-------|----------|
| `yolo11n.pt` | 6 MB | Fastest | Good |
| `yolo11s.pt` | 22 MB | Fast | Better |
| `yolo11m.pt` | 51 MB | Medium | Best |

---

## Detection

Single detection result.

**Location:** `swarm/perception/detector.py`

```python
detection = Detection(
    class_id=0,
    class_name="person",
    confidence=0.85,
    bbox=(100, 50, 200, 300),  # x, y, width, height
)
```

### Fields

| Field | Type | Description |
|-------|------|-------------|
| `class_id` | `int` | COCO class ID |
| `class_name` | `str` | Human-readable class name |
| `confidence` | `float` | Detection confidence (0-1) |
| `bbox` | `tuple[int, int, int, int]` | Bounding box (x, y, width, height) |
| `track_id` | `int` | Tracker-assigned ID (0 if untracked) |

### COCO Class IDs (Relevant)

| ID | Class |
|----|-------|
| 0 | person |
| 2 | car |
| 3 | motorcycle |
| 5 | bus |
| 7 | truck |

---

## SimpleTracker

IoU-based multi-object tracker.

**Location:** `swarm/perception/tracker.py`

```python
from swarm.perception import SimpleTracker, TrackerConfig

tracker = SimpleTracker(TrackerConfig(
    iou_threshold=0.3,
    max_age=30,
    min_hits=3,
))

# Update with new detections each frame
tracks = tracker.update(detections)

for track in tracks:
    print(f"Track {track.track_id}: {track.class_name} at {track.bbox}")
```

### Methods

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `update(detections)` | `list[Detection]` | `list[Track]` | Update tracker with new detections |
| `reset()` | - | `None` | Clear all tracks |
| `get_active_tracks()` | - | `list[Track]` | Get confirmed tracks |

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `num_tracks` | `int` | Number of active tracks |

---

## TrackerConfig

Configuration for SimpleTracker.

**Location:** `swarm/perception/tracker.py`

```python
from swarm.perception import TrackerConfig

config = TrackerConfig(
    iou_threshold=0.3,
    max_age=30,
    min_hits=3,
)
```

### Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `iou_threshold` | `float` | `0.3` | Minimum IoU for matching |
| `max_age` | `int` | `30` | Frames before track is deleted |
| `min_hits` | `int` | `3` | Detections needed to confirm track |

---

## Track

Tracked object with persistent ID.

**Location:** `swarm/perception/tracker.py`

### Fields

| Field | Type | Description |
|-------|------|-------------|
| `track_id` | `int` | Unique track identifier |
| `class_id` | `int` | COCO class ID |
| `class_name` | `str` | Human-readable class name |
| `bbox` | `tuple` | Current bounding box |
| `confidence` | `float` | Last detection confidence |
| `age` | `int` | Frames since track started |
| `hits` | `int` | Total detections matched |
| `time_since_update` | `int` | Frames since last detection |

---

## GazeboCamera

Utility class for Gazebo camera topics.

**Location:** `swarm/perception/camera.py`

```python
from swarm.perception import GazeboCamera

# List available camera topics
topics = GazeboCamera.list_available_topics()

# Check if camera is publishing
is_active = GazeboCamera.is_publishing("/drone_0/camera")

# Get ROS2 bridge command
cmd = GazeboCamera.get_ros2_bridge_command(
    gz_topic="/drone_0/camera",
    ros_topic="/drone_0/camera/image",
)
```

### Class Methods

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `list_available_topics()` | - | `list[str]` | List Gazebo camera topics |
| `is_publishing(topic)` | `str` | `bool` | Check if topic has publishers |
| `get_ros2_bridge_command(gz_topic, ros_topic)` | `str`, `str` | `str` | Generate bridge command |

---

## Usage Example: Full Pipeline

```python
import cv2
from swarm.perception import YOLODetector, DetectorConfig, SimpleTracker

# Initialize detector
detector = YOLODetector(DetectorConfig(use_gpu=True))
detector.initialize()

# Initialize tracker
tracker = SimpleTracker()

# Process video stream
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detect objects
    detections, inference_ms = detector.detect(frame)

    # Update tracker
    tracks = tracker.update(detections)

    # Draw results
    for track in tracks:
        x, y, w, h = track.bbox
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(frame, f"{track.class_name} #{track.track_id}",
                    (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("Detections", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
```
