#!/usr/bin/env python
# 
# Detects aruco markers in the /image topic, and publishes:
#   aruco/location - centers of all aruco markers in the image 
#   aruco/image - annotated raw image
#   aruco/image/compression - annotated compressed image
# 

import cv2
import numpy as np

import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CompressedImage
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from rh_msgs.msg import Point2D, PointList

global aruco_raw_publisher
global aruco_cmp_publisher
global aruco_loc_publisher
global aruco_box_publisher
global bridge

# ArUco dictionary
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

# Parameters for marker detection
params = cv2.aruco.DetectorParameters_create()
# contour filtering parameters
# 800*0.01 = 8 pixel min perimeter
params.minMarkerPerimeterRate = 0.01
# 800*3 = 2400 pixel max perimeter
params.maxMarkerPerimeterRate = 3.0
# Minimum distance between any pair of corners from two different markers, 
# relative to the minimum marker perimeter of the two markers.
params.minMarkerDistanceRate = 0.10


def name():
    return "aruco_detector"


def publish_marker_loc(markerId, marker, src_image, c1, c2, c3, c4):

    header = Header(frame_id="camera", \
            seq=src_image.header.seq, stamp=src_image.header.stamp)

    if c4>0:
        points = [Point2D(corner[0], corner[1]) for corner in marker]
        point_list = PointList(points=points, header=header)
        aruco_box_publisher.publish(point_list)

    if c2>0 or c3>0:
        # find center location for chosen marker
        topLeft = tuple(marker[0])
        bottomRight = tuple(marker[2])
        x,y = topLeft
        w = bottomRight[0] - topLeft[0]
        h = bottomRight[1] - topLeft[1]
        cx = int(round(x + w/2.))
        cy = int(round(y + h/2.))

        # publish bounding box center
        point = Point(x=cx, y=cy, z=1.)
        pointStamped = PointStamped(point=point, header=header)
        aruco_loc_publisher.publish(pointStamped)


def process_image(image):

    global aruco_raw_publisher
    global aruco_cmp_publisher
    rospy.logdebug("Finding ArUco markers")

    # Convert to opencv image
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    annotated = cv_image.copy()

    # Check to see if anyone has subscribed to us
    c1 = aruco_raw_publisher.get_num_connections()
    c2 = aruco_cmp_publisher.get_num_connections()
    c3 = aruco_loc_publisher.get_num_connections()
    c4 = aruco_box_publisher.get_num_connections()

    rospy.logdebug("num connections: aruco:%d, aruco_compressed:%d, location:%d, box:%d" % (c1, c2, c3, c4))

    if c1>0 or c2>0 or c3>0 or c4>0:

        # Find markers
        res = cv2.aruco.detectMarkers(cv_image, dictionary, parameters=params)
        markerCorners = res[0]
        markerIds = res[1]

        if markerIds is None or len(markerIds)==0:
            rospy.logdebug("No aruco markers detected")

        else:
            rospy.logdebug("Found %d aruco markers" % len(markerIds))
            
            for i, markerId in enumerate(markerIds):
                marker = markerCorners[i][0]
                publish_marker_loc(markerId, marker, image, c1, c2, c3, c4)

            if c1>0 or c2>0:
                # draw all marker annotations
                cv2.aruco.drawDetectedMarkers(annotated, markerCorners, markerIds)
             
        if c1>0:
            # publish annotated image
            image_message = bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            aruco_raw_publisher.publish(image_message)

        if c2>0:
            # published annotated compressed image
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "png"
            msg.data = np.array(cv2.imencode('.png', annotated)[1]).tostring()
            aruco_cmp_publisher.publish(msg)


def detect_aruco():

    global aruco_raw_publisher
    global aruco_cmp_publisher
    global aruco_loc_publisher
    global aruco_box_publisher
    global bridge

    rospy.init_node(name(), log_level=rospy.INFO)
    aruco_raw_publisher = rospy.Publisher("aruco/image", Image, queue_size = 5)
    aruco_cmp_publisher = rospy.Publisher("aruco/image/compressed", CompressedImage, queue_size = 5)
    aruco_loc_publisher = rospy.Publisher("aruco/location", PointStamped, queue_size = 5)
    aruco_box_publisher = rospy.Publisher("aruco/box", PointList, queue_size = 5)
    
    rospy.Subscriber("image", Image, process_image)
    
    bridge = CvBridge()

    rospy.loginfo("Ready... 'spinning'")
    rospy.spin()


if __name__ == "__main__":
    detect_aruco()
