# DetectionArray.msg - Batch of detections from a single camera frame
# Published by each drone's PerceptionNode

# Source drone (0-indexed)
uint8 drone_id

# Frame sequence number (for tracking/deduplication)
uint32 frame_seq

# Drone position when frame was captured (for world position estimation)
float32[3] drone_position_ned
float32 drone_yaw_deg

# Inference timing (for performance monitoring)
float32 inference_time_ms

# All detections in this frame
Detection[] detections

# Timestamp of the original image
builtin_interfaces/Time image_stamp

# Timestamp when this message was created
builtin_interfaces/Time stamp
