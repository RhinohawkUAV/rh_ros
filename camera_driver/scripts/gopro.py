#!/usr/bin/env python
from threading import Lock
import os
import logging

import cv2 

import rospy
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo
from polled_camera.srv import GetPolledImage, GetPolledImageResponse
from cv_bridge import CvBridge, CvBridgeError


# Implement Polled Camera Node for GoPro
#
# http://wiki.ros.org/camera_drivers
#
# Services 
#      request_image (polled_camera/GetPolledImage)
#      set_camera_info (sensor_msgs/SetCameraInfo) 
#
# Published Topics:
#       /gopro/image_raw (sensor_msgs/Image)

global publisher
global mutex
mutex = Lock()


def name():
    return 'gopro'

def capture_image(getPolledImage):

    rospy.loginfo("Capturing image")

    filename = 'none'
    success = False
    message = 'No photo for you'
    stamp = rospy.Time.now()

    # run the camera
    mutex.acquire()
    try:
        camera_stream = cv2.VideoCapture('http://10.5.5.9:8080/live/amba.m3u8');
        if not camera_stream.isOpened():
            message = 'Cannot open gopro'
            rospy.logerr(message)
            success = False
        else:    
            success, cv2_image = camera_stream.read()
        if success == False:
            message = "Could not read image from gopro"
            rospy.logerr(message)
    finally:
        mutex.release()


    if success:
        rospy.loginfo("Capture sucessful, publishing image")
        # convert to ROS image and publish
        try:
            ros_image = CvBridge().cv2_to_imgmsg(cv2_image, 'bgr8')
            ros_image.header.stamp = stamp
            publisher.publish(ros_image)
            message = "Image capture completed successfully"
        except TypeError as e:
            message = "CvBrideg could not convert image: %s" % e
            rospy.logerr(message)
            success  = False

    return GetPolledImageResponse(success, message, stamp)


def set_camera_info(msg):
    rospy.loginfo('set_camera_info called: ', msg)

def image_capture():
    global publisher

    rospy.init_node(name())

    publisher = rospy.Publisher(name() + "/image_raw", Image, queue_size = 10)
    rospy.Service(name() + '/request_image', GetPolledImage, capture_image)
    rospy.Service(name() + '/set_camera_info', SetCameraInfo, set_camera_info) 

    rospy.loginfo("Ready")
    rospy.spin()


if __name__ == "__main__":
    image_capture()
    
