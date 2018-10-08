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
data = yaml.load(stream)

m = Mission()

def load_points(data):
    if 'points' in data:
        points = []
        for p in data['points']:
            coord = GPSCoord(p['lat'], p['lon'], p['alt'])
            points.append(coord)
    return points

if 'geofence' in data:
    geofence = data['geofence']
    m.geofence.points = load_points(geofence)
    print("Loaded geofence polygon with %d points" % len(m.geofence.points))

if 'mission_wps' in data:
    mission_wps = data['mission_wps']
    m.mission_wps.points = load_points(mission_wps)
    print("Loaded %d mission waypoints" % len(m.mission_wps.points))

if 'static_nfzs' in data:
    for static_nfz in data['static_nfzs']:
        points = load_points(static_nfz)
        m.static_nfzs.append(GPSCoordList(points))
        print("Loaded static NFZ with %d points" % len(points))

if 'roads' in data:
    for road in data['roads']:
        points = load_points(road)
        m.roads.append(GPSCoordList(points))
        print("Loaded road with %d points" % len(points))
        
set_mission = get_proxy('/rh/command/set_mission', SetMission)
res = set_mission(m)
if res and res.success:
    print("Successfully set mission")
    sys.exit(0)
else:
    print("Problem setting mission")
    sys.exit(1)

