#!/usr/bin/env python
#
# Upload a KML as a Rhinohawk mission
#

import sys
from zipfile import ZipFile
from pykml import parser

from rh_autonomy.util import get_proxy
from rh_msgs.srv import SetMission
from rh_msgs.msg import Mission, GPSCoord, GPSCoordList

DEBUG = False

mission_filepath = sys.argv[1]
print("Loading %s" % mission_filepath)

if mission_filepath.endswith('.kmz'):
    kmz = ZipFile(mission_filepath, 'r')
    kml = kmz.open('doc.kml', 'r')
else:
    kml = open(mission_filepath, "r")

kml_str = kml.read()
root = parser.fromstring(kml_str)

mission_name = root.Document.name
print("Loading mission '%s'" % mission_name)

m = Mission()
base = None
target = None
transit_wps = []

def coordsToPoint(coords):
    return GPSCoord(coords[1], coords[0], coords[2])

for folder in root.Document.Folder:

    fn = folder.name.pyval.lower()
    if fn == 'no fly zones':
        
        for placemark in folder.Placemark:

            points = []
            print("Loading no fly zone '%s'" % placemark.name)
            coordStrList = placemark.Polygon.outerBoundaryIs.LinearRing.coordinates.pyval.strip()
            for coordStr in coordStrList.split(" "):
                coords = [float(n) for n in coordStr.split(",")]
                if DEBUG: print(coords)
                points.append(coordsToPoint(coords))
            
            if points[0]==points[-1]:
                print("Removing redundant close point")
                points = points[0:-1]

            m.static_nfzs.append(GPSCoordList(points))

    elif fn == 'flight plan waypoints':

        for placemark in folder.Placemark:

            pname = placemark.name.pyval.lower()
            print("Loading waypoint '%s'" % pname)
            coordStr = placemark.Point.coordinates.pyval.strip()
            coords = [float(n) for n in coordStr.split(",")]
            if DEBUG: print(coords)

            if pname == 'base':
                base = coords
            elif pname.startswith('joe'):
                target = coords
            elif pname.startswith('transit'):
                transit_wps.append(coords)
            else:
                print("Ignoring waypoint '%s'" % pname)

    elif fn == 'geofence waypoints':

        points = []
        for placemark in folder.Placemark:

            pname = placemark.name.pyval.lower()
            print("Loading geofence waypoint '%s'" % pname)
            coordStr = placemark.Point.coordinates.pyval.strip()
            coords = [float(n) for n in coordStr.split(",")]
            if DEBUG: print(coords)
            points.append(coordsToPoint(coords))

        if points[0]==points[-1]:
            print("Removing redundant close point")
            points = points[0:-1]
            
        m.geofence.points = points

    else:
        print("Ignoring folder: %s" % fn)


points = []
points.append(coordsToPoint(base))
for transit_wp in transit_wps:
    points.append(coordsToPoint(transit_wp))
points.append(coordsToPoint(target))
m.mission_wps.points = points

#print(m)
print("Base location: %f,%f" % (base[1], base[0]))

print("Setting mission...")
set_mission = get_proxy('/rh/command/set_mission', SetMission)
if set_mission(m):
    print("Successfully set mission")
    sys.exit(0)
else:
    print("Problem setting mission")
    sys.exit(1)

