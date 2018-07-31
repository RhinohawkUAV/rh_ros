#
# Rhinohawk Constants
#

# Is the vehicle a quadplane? If not, copter is assumed.
isQuadPlane = False

# Mavlink modes
MODE_AUTO = 'AUTO'
MODE_GUIDED = 'GUIDED'
MODE_LOITER = 'QLOITER' if isQuadPlane else 'LOITER'

# Mavlink frames
MAV_FRAME_GLOBAL = 0
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

# Mavlink commands
MAV_CMD_WAYPOINT = 16
MAV_CMD_RTL = 20
MAV_CMD_TAKEOFF = 84 if isQuadPlane else 22
MAV_CMD_LAND = 85 if isQuadPlane else 21
MAV_CMD_DO_SET_HOME = 179

# Our magic number used to stash goal ids in the Mavlink WaypointList
GOAL_ID_START = 3333

# Mission cruise altitude in meters
CRUISE_ALTITUDE = 30

# Should we search for a target, or just land?
PERFORM_SEARCH = True

# Search altitude in meters
SEARCH_ALTITUDE = 20

# Search radius in meters
SEARCH_RADIUS = 50

# Overlap of search strips in meters
SEARCH_STRIP_OVERLAP = 10

# Distance from waypoint in meters at which the waypoint is considered "completed"
WAYPOINT_ACCEPTANCE_RADIUS = 10

# Distance which must be kept from NFZ's during a mission
NOFLYZONE_BUFFER_SIZE = 10

