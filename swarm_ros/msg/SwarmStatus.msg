# SwarmStatus.msg - Aggregated status of entire swarm
# Published by MissionCoordinator at 1Hz

# Swarm size and connectivity
uint8 num_drones_configured
uint8 num_drones_connected
uint8 num_drones_armed
uint8 num_drones_in_flight
uint8 num_drones_failed

# Current mission info
string current_mission_name
float32 mission_progress  # 0.0 to 1.0

# Current formation
uint8 current_formation_type  # See FormationRole.msg for enum values
uint16 current_formation_id

# Swarm state enumeration
uint8 SWARM_IDLE = 0
uint8 SWARM_CONNECTING = 1
uint8 SWARM_READY = 2
uint8 SWARM_ARMING = 3
uint8 SWARM_TAKING_OFF = 4
uint8 SWARM_IN_FORMATION = 5
uint8 SWARM_EXECUTING_MISSION = 6
uint8 SWARM_LANDING = 7
uint8 SWARM_ERROR = 8
uint8 swarm_state

# Individual drone states (array)
DroneState[] drone_states

# Timestamp
builtin_interfaces/Time stamp
