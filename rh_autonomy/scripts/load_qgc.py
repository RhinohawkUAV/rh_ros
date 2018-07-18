#!/usr/bin/env python
#
# Upload a QGroundControl plan as a Rhinohawk mission
# 

import sys
import json

from rh_autonomy.util import get_proxy
from rh_msgs.srv import SetMission
from rh_msgs.msg import Mission, GPSCoord, GPSCoordList

filepath = sys.argv[1]
print("Loading %s" % filepath)

file = open(filepath).read()
data = json.loads(file)

if "geoFence" in data:
    geofence = data["geoFence"]["polygon"]
    print("Loading geofence polygon with %d points" % len(geofence))

wps = []
for item in data["mission"]["items"]:
    params = item["params"]
    lat = params[4]
    lon = params[5]
    print("Waypoint %s,%s" %(lat,lon))
    wps.append(GPSCoord(lat, lon, 1))

mission = Mission()
mission.geofence = GPSCoordList([GPSCoord(p[0], p[1], 1) for p in geofence])
mission.mission_wps = GPSCoordList(wps)

set_mission = get_proxy('/rh/command/set_mission', SetMission)
if set_mission(mission):
    print("Successfully set mission")
    sys.exit(0)
else:
    print("Problem setting mission")
    sys.exit(1)

