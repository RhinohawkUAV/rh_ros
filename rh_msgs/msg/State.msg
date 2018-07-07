# The mission definition
rh_msgs/Mission mission

# List of last known dynamic no fly zones
rh_msgs/GPSCoordList[] dynamic_nfzs

# Last known GPS position
sensor_msgs/NavSatFix gps_position

# List of waypoints currently uploaded to the autopilot
mavros_msgs/Waypoint[] apm_wps

# Index of the target mission transit waypoint
uint8 target_mission_wp

# Identified landing location
rh_msgs/GPSCoord landing_location

