# see enum MAV_FRAME
uint8 frame
uint8 FRAME_GLOBAL = 0
uint8 FRAME_LOCAL_NED = 1
uint8 FRAME_MISSION = 2
uint8 FRAME_GLOBAL_REL_ALT = 3
uint8 FRAME_LOCAL_ENU = 4

# The mission definition
rh_msgs/Mission mission

# List of last known dynamic no fly zones
rh_msgs/GPSCoordList[] dynamic_nfzs

# Last known vehicle state
rh_msgs/VehicleState vehicle_state

# List of waypoints currently uploaded to the autopilot
mavros_msgs/Waypoint[] apm_wps

# Index of the target mission transit waypoint
uint8 target_mission_wp

# Identified landing location
rh_msgs/GPSCoord landing_location

# See rh_autonomy.state.MissionStatus
uint8 mission_status
uint8 MISSION_STATUS_NOT_READY = 1
uint8 MISSION_STATUS_READY = 2
uint8 MISSION_STATUS_RUNNING = 3
uint8 MISSION_STATUS_ABORTING = 4
uint8 MISSION_STATUS_COMPLETE = 5

