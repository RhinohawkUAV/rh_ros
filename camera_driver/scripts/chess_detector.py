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

global chessboard_publisher
global chessbox_publisher
global chessbox_location_publisher
global bridge
global seq
seq = 0

# Chessboard size
nx = 6
ny = 8


def name():
    return "chess_detector"


def process_image(image):

    global seq
    rospy.loginfo("process_image")

    # Convert to opencv image
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    annotated = cv_image.copy()
    annotated2 = cv_image.copy()

    c1 = chessboard_publisher.get_num_connections()
    c2 = chessbox_publisher.get_num_connections()
    c3 = chessbox_location_publisher.get_num_connections()

    if c1>0 or c2>0 or c3>0:

        # Find chessboard
        found, corners = cv2.findChessboardCorners(cv_image, (nx, ny), None, cv2.CALIB_CB_FAST_CHECK)
        
        if not(found):
            rospy.loginfo("No chessboard corners detected")
        else:
            rospy.loginfo("Found chessboard corners")

            if c1>0:
                # draw chessboard annotations
                cv2.drawChessboardCorners(annotated, (nx, ny), corners, found)

            x,y,w,h = cv2.boundingRect(corners)

            if c2>0:
                # draw bounding box 
                cv2.rectangle(annotated2,(x,y),(x+w,y+h),(255,0,0),2)

            if c3>0:
                # find center location
                cx = int(round(x + w/2.))
                cy = int(round(y + h/2.))

                # publish bounding box center
                seq += 1
                header = Header(frame_id="chess", \
                        seq=image.header.seq, stamp=image.header.stamp)
                point = Point(x=cx, y=cy, z=1.)
                pointStamped = PointStamped(point=point, header=header)
                chessbox_location_publisher(pointStamped)
        
        if c1>0:
            # publish chessboard
            image_message = bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            chessboard_publisher.publish(image_message)

        if c2>0:
            # publish bounding box
            image_message = bridge.cv2_to_imgmsg(annotated2, encoding="bgr8")
            chessbox_publisher.publish(image_message)


def detect_chessboard():

    global chessboard_publisher
    global chessbox_publisher
    global chessbox_location_publisher
    global bridge

    # Hoookup ROS stuff
    rospy.init_node(name())
    chessboard_publisher = rospy.Publisher("chessboard", Image, queue_size = 5)
    chessbox_publisher = rospy.Publisher("chessbox", Image, queue_size = 5)
    chessbox_location_publisher = rospy.Publisher("chessbox_location", PointStamped, queue_size = 5)
    
    rospy.Subscriber("image", Image, process_image)
    
    # ROS to OpenCV
    bridge = CvBridge()

    rospy.loginfo("Ready... 'spinning'")
    rospy.spin()


if __name__ == "__main__":
    detect_chessboard()
