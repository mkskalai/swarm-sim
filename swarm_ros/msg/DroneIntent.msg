# DroneIntent.msg - What the drone intends to do next
# Published by each drone at 4Hz for coordination

# Drone identifier
uint8 drone_id

# Target position in NED frame (where drone is heading)
float32[3] target_position_ned

# Target yaw in degrees
float32 target_yaw_deg

# Action type enumeration
uint8 ACTION_HOVER = 0
uint8 ACTION_GOTO = 1
uint8 ACTION_ORBIT = 2
uint8 ACTION_TRACK = 3
uint8 ACTION_SEARCH = 4
uint8 ACTION_RTL = 5
uint8 ACTION_LAND = 6
uint8 action_type

# Unique action ID for deduplication and tracking
uint16 action_id

# Estimated time to reach target (seconds)
float32 eta_seconds

# Timestamp
builtin_interfaces/Time stamp
