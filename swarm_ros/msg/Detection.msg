# Detection.msg - Single object detection from drone camera
# Published as part of DetectionArray

# Drone that made this detection (0-indexed)
uint8 drone_id

# Unique track ID maintained across frames by tracker (0 = untracked)
uint32 track_id

# Detection ID within this frame
uint16 detection_id

# Object class from COCO dataset
# Common values: 0=person, 2=car, 3=motorcycle, 5=bus, 7=truck
uint8 class_id

# Human-readable class name (e.g., "person", "car")
string class_name

# Detection confidence (0.0 - 1.0)
float32 confidence

# Bounding box in image coordinates (pixels)
uint16 bbox_x        # Top-left x
uint16 bbox_y        # Top-left y
uint16 bbox_width
uint16 bbox_height

# Image dimensions for normalization
uint16 image_width
uint16 image_height

# Estimated position in world NED frame (meters)
# Calculated from camera pose + depth estimation (if available)
# Set to NaN values if position cannot be estimated
float32[3] estimated_position_ned

# Position estimation confidence (0.0 = no estimate, 1.0 = high confidence)
float32 position_confidence

# Timestamp when detection was made
builtin_interfaces/Time stamp
