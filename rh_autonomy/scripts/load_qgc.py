#!/usr/bin/env python
#
# Upload a QGroundControl plan as a Rhinohawk mission
# 

import sys
import json

from rh_autonomy.util import get_proxy
from rh_msgs.srv import SetMission
from rh_msgs.msg import Mission, GPSCoord, GPSCoordList

def points2coords(points):
    return GPSCoordList([GPSCoord(p[0], p[1], 1) for p in points])

mission_filepath = sys.argv[1]
print("Loading %s" % mission_filepath)

if len(sys.argv)>2:
    nfz_filepath = sys.argv[2]
else:
    nfz_filepath = None

file = open(mission_filepath).read()
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
mission.geofence = points2coords(geofence)
mission.mission_wps = GPSCoordList(wps)

if nfz_filepath:
    file = open(nfz_filepath).read()
    data = json.loads(file)
    if "geoFence" in data:
        nfz = data["geoFence"]["polygon"]
        print("Loading NFZ polygon with %d points" % len(nfz))
        mission.static_nfzs = [points2coords(nfz),]

set_mission = get_proxy('/rh/command/set_mission', SetMission)
if set_mission(mission):
    print("Successfully set mission")
    sys.exit(0)
else:
    print("Problem setting mission")
    sys.exit(1)

