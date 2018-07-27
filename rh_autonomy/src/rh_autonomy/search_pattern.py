#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from rh_msgs.srv import *
from rh_msgs.msg import GPSCoord, GPSCoordList
import rospy

from math import ceil, pow, sqrt, atan2, cos, sin

class Point:
    """ Point class represents and manipulates x,y coords. 
        Per ROS convention, this should be in meters.
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y

"""
target: GPS coordinates for the center of the search area

current: current location of the drone. Note, this can be approximate.
It defines the orientation in which we lay out the strips, which will
be perpendicular to the line between current and target.

strip_width: As per ROS convention, this should be in meters. If you want
overlap in the images, simply set stripWidth less than the actual
width seen by the camera.
"""
def create_waypoints(req):
    target = req.target
    current = req.current_location
    search_radius = req.search_radius
    strip_width=req.strip_width
    search_altitude=req.search_altitude
    
    num_strips = int(ceil(float(2.0 * search_radius) / float(strip_width)))
    rospy.loginfo("Beginning search centered around ({}, {})".format(target.lat, target.lon))
    rospy.loginfo("Search radius (m): {}".format(search_radius))
    rospy.loginfo("Number of strips needed: {}".format(num_strips))
                    
    """
    Define our search coordinate system, where the target lives at zero, and make
    waypoints in that system.
    
    We always move down in y by strip width. The only complex part is determining x,
    which we do with help from Pythagoras. Note, we calculate x based on the value of y 
    at the edge of the strip closest to the center of circle, whereas the waypoint value
    of y is in the center of the strip. This adjustment ensures we cover the full circle
    at the end of each strip.
    """
    pointList = []
    y = (num_strips - 1) * strip_width / 2.0
    sign = 1
    for _ in range(num_strips):
        x = sqrt(pow(search_radius, 2) - pow((abs(y) - (strip_width / 2.0)), 2))       
        pointList.append(Point(sign * x, y))
        sign = -1 * sign
        pointList.append(Point(sign * x, y))
        y = y - strip_width
                
    """ 
    Convert into GPS coordinates
    """
    # ref: https://en.wikipedia.org/wiki/Geographic_coordinate_system#Expressing_latitude_and_longitude_as_linear_units
    phi = target.lat
    meters_per_degree_latitude = 111132.92 - (559.82 * cos(phi)) \
        + (1.175 * (cos(4 * phi))) - (0.0023 * cos(6 * phi))
    meters_per_degree_longitude = (111412.84 * cos(phi)) - (93.5 * cos(3 * phi)) \
        + (0.118 * cos (5 * phi))
            
    """
    Calculate x-y coordinate system rotation angle relative to latitude-longtitude 
    Use atan2, not atan, to avoid possible issues near 90 degrees
    """
    theta = atan2((current.lat - target.lat) , (current.lon - target.lon))
    
    """
    Rotate points to align axes. We must rotate by NEGATIVE theta.
    cos(-theta) = cos(theta), sin(-theta) = -sin(theta)
    """
    rotatedPointList = []
    for p in pointList:
        rotatedPointList.append(Point((p.x * cos(theta)) + (p.y * sin(theta)), \
            (p.x * sin(-theta) + (cos(theta) * p.y))))
        
    """
    Finally, we can scale and translate.
    """
    GPSCoordList = []
    for p in rotatedPointList:
        GPSCoordList.append(GPSCoord(
            lat=(p.y / meters_per_degree_latitude) + target.lat, \
	    lon=(p.x / meters_per_degree_longitude) + target.lon, \
	    alt=search_altitude))
     
    return GPSCoordList

def generate_search_pattern_server():
    rospy.init_node('generate_search_pattern')
    s = rospy.Service('generate_search_pattern', GenerateSearchPattern, create_waypoints)
    print("Ready to generate search pattern.")
    rospy.spin()

if __name__ == "__main__":
    generate_search_pattern_server()
