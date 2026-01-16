# VIOState.msg - Visual-Inertial Odometry state estimate
# Published by VIONode at variable rate (typically 10-15 Hz)

# Drone identifier
uint8 drone_id

# Position in NED frame (meters)
# north, east, down
float32[3] position_ned

# Velocity in NED frame (m/s)
# vn, ve, vd
float32[3] velocity_ned

# Orientation as quaternion [w, x, y, z]
float32[4] orientation

# Position uncertainty (1-sigma, meters)
float32[3] position_std

# Velocity uncertainty (1-sigma, m/s)
float32[3] velocity_std

# Navigation mode constants
uint8 MODE_GPS = 0
uint8 MODE_VIO = 1
uint8 MODE_HYBRID = 2
uint8 MODE_DEAD_RECKONING = 3

# Current navigation mode
uint8 mode

# Visual odometry quality (0.0-1.0)
# Based on successful feature matches
float32 vo_quality

# Scale estimation confidence (0.0-1.0)
# Higher means more confident in metric scale
float32 scale_confidence

# Number of IMU samples integrated since last VO update
uint32 imu_sample_count

# Current metric scale estimate
float32 scale

# IMU sample rate (Hz)
float32 imu_rate

# Timestamps
builtin_interfaces/Time stamp
builtin_interfaces/Time last_vo_time
builtin_interfaces/Time last_gps_time
