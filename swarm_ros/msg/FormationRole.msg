# FormationRole.msg - Formation membership and role
# Published by each drone at 4Hz for distributed formation flying

# Drone identifier
uint8 drone_id

# Formation identifier (allows multiple formations in swarm)
uint16 formation_id

# Role in formation
uint8 ROLE_UNASSIGNED = 0
uint8 ROLE_LEADER = 1
uint8 ROLE_FOLLOWER = 2
uint8 role

# Leader's drone_id (if role is FOLLOWER)
uint8 leader_id

# Offset from leader in NED frame (if role is FOLLOWER)
# This is the desired relative position from leader
float32[3] offset_from_leader

# Formation type enumeration (matches FormationType in formations.py)
uint8 FORMATION_LINE = 0
uint8 FORMATION_V = 1
uint8 FORMATION_TRIANGLE = 2
uint8 FORMATION_GRID = 3
uint8 FORMATION_CIRCLE = 4
uint8 FORMATION_DIAMOND = 5
uint8 formation_type

# Position index within formation (0 = leader position)
uint8 position_index

# Timestamp
builtin_interfaces/Time stamp
