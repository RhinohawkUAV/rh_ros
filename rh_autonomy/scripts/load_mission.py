#!/usr/bin/env python
#
# Upload a QGroundControl plan as a Rhinohawk mission
# 

import sys

from rh_autonomy.util import get_proxy
from rh_msgs.srv import SetMission
from rh_msgs.msg import Mission, GPSCoord, GPSCoordList

import yaml

mission_filepath = sys.argv[1]
print("Loading %s" % mission_filepath)

stream = open(mission_filepath, "r")
data = yaml.load_all(stream)
for doc in data:
    for k,v in doc.items():
        print("%s -> %s" % (k,v))
    print("\n")

m = Mission()

if "geofence" in doc:
    print("Loading geofence polygon")
    points = []
    for p in doc["geofence"]['points']:
        coord = GPSCoord(p['lat'], p['lon'], p['alt'])
        points.append(coord)
    m.geofence.points = points




print(m)

sys.exit(0)

wps = []
for item in data["mission"]["items"]:
    params = item["params"]
    lat = params[4]
    lon = params[5]
    print("Waypoint %s,%s" %(lat,lon))
    wps.append(GPSCoord(lat, lon, 1))

set_mission = get_proxy('/rh/command/set_mission', SetMission)
if set_mission(m):
    print("Successfully set mission")
    sys.exit(0)
else:
    print("Problem setting mission")
    sys.exit(1)

