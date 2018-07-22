#
# Constants, mainly for talking with Mavlink
#

isQuadPlane = False

MODE_AUTO = 'AUTO'
MODE_GUIDED = 'GUIDED'
MODE_LOITER = 'QLOITER' if isQuadPlane else 'LOITER'

MAV_FRAME_GLOBAL = 0
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

MAV_CMD_WAYPOINT = 16
MAV_CMD_RTL = 20
MAV_CMD_TAKEOFF = 84 if isQuadPlane else 22
MAV_CMD_LAND = 85 if isQuadPlane else 21
MAV_CMD_DO_SET_HOME = 179

GOAL_ID_START = 3333


