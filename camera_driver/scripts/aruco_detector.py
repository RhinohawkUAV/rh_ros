#!/usr/bin/env python

import cv2
import numpy as np
import yaml

import rospy
import rospkg

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header

global aruco_publisher
global arucobox_publisher
global arucobox_location_publisher
global bridge
global seq
seq = 0

# Parameters for arUco marker detection
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
params = cv2.aruco.DetectorParameters_create()
params.minMarkerPerimeterRate = 0.01
params.maxMarkerPerimeterRate = 3.0
params.minMarkerDistanceRate = 0.10


def name():
    return "aruco_detector"


def process_image(image):

    global seq
    rospy.loginfo("process_image")

    # Convert to opencv image
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    annotated = cv_image.copy()
    annotated2 = cv_image.copy()

    # Check to see if anyone has subscribed to us
    c1 = aruco_publisher.get_num_connections()
    c2 = arucobox_publisher.get_num_connections()
    c3 = arucobox_location_publisher.get_num_connections()

    if c1>0 or c2>0 or c3>0:

        # Find markers
        res = cv2.aruco.detectMarkers(cv_image, dictionary, parameters=params)
        markerCorners = res[0]
        markerIds = res[1]

        if markerIds is None or len(markerIds)==0:
            rospy.loginfo("No aruco markers detected")
        else:
            rospy.loginfo("Found %d aruco markers" % len(markerIds))
             
            # For now, just take the first marker and roll with it
            markerId = markerIds[0]
            marker = markerCorners[0]
            topLeft = tuple(marker[0][0])
            bottomRight = tuple(marker[0][2])

            if c1>0:
                # draw all marker annotations
                cv2.aruco.drawDetectedMarkers(annotated, res[0], res[1])

            if c2>0:
                # draw bounding box for chosen marker 
                cv2.rectangle(annotated2,topLeft,bottomRight,(255,0,0),2)
                    
            if c3>0:
                # find center location for chosen marker
                x,y = topLeft
                w = bottomRight[0] - topLeft[0]
                h = bottomRight[1] - topLeft[1]
                cx = int(round(x + w/2.))
                cy = int(round(y + h/2.))

                # publish bounding box center
                seq += 1
                header = Header(frame_id="landing_box", \
                        seq=image.header.seq, stamp=image.header.stamp)
                point = Point(x=cx, y=cy, z=1.)
                pointStamped = PointStamped(point=point, header=header)
                arucobox_location_publisher.publish(pointStamped)
        
        if c1>0:
            # publish annotated image
            image_message = bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            aruco_publisher.publish(image_message)

        if c2>0:
            # publish bounding box image
            image_message = bridge.cv2_to_imgmsg(annotated2, encoding="bgr8")
            arucobox_publisher.publish(image_message)


def detect_aruco():

    global aruco_publisher
    global arucobox_publisher
    global arucobox_location_publisher
    global bridge

    # Hoookup ROS stuff
    rospy.init_node(name())
    aruco_publisher = rospy.Publisher("aruco_marker", Image, queue_size = 5)
    arucobox_publisher = rospy.Publisher("landing_box", Image, queue_size = 5)
    arucobox_location_publisher = rospy.Publisher("landing_box_location", PointStamped, queue_size = 5)
    
    rospy.Subscriber("image", Image, process_image)
    
    # ROS to OpenCV
    bridge = CvBridge()

    rospy.loginfo("Ready... 'spinning'")
    rospy.spin()


if __name__ == "__main__":
    detect_aruco()
