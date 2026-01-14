# DroneState.msg - Position, status, and health of a single drone
# Published by each drone at 10Hz for P2P awareness

# Drone identifier (0-indexed)
uint8 drone_id

# Position in local NED frame (meters)
# north: positive = north, east: positive = east, down: positive = down (negative = altitude)
float32[3] position_ned

# Heading in degrees (0-360, 0 = north, 90 = east)
float32 yaw_deg

# Velocity in NED frame (m/s)
float32[3] velocity_ned

# Flight state enumeration
uint8 STATE_DISCONNECTED = 0
uint8 STATE_CONNECTED = 1
uint8 STATE_ARMED = 2
uint8 STATE_TAKING_OFF = 3
uint8 STATE_IN_FLIGHT = 4
uint8 STATE_LANDING = 5
uint8 STATE_LANDED = 6
uint8 STATE_ERROR = 7
uint8 state

# Battery level (0-100 percent)
uint8 battery_percent

# Health flags
bool is_healthy
bool has_gps_lock
bool ekf_ok

# Timestamp when this state was captured
builtin_interfaces/Time stamp
