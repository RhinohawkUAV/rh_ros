#!/usr/bin/env python

# Implement Polled Camera Node for Logitech, Inc. QuickCam E2500 series
#
# http://wiki.ros.org/camera_drivers
#
# Services 
#      request_image (polled_camera/GetPolledImage)
#      set_camera_info (sensor_msgs/SetCameraInfo) 
#
# Published Topics:
#       /e2500/image_raw (sensor_msgs/Image)
#       /e2500/camera_info  (sensor_msgs/CameraInfo)


from threading import Lock
import os
import logging

import cv2 
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.srv import SetCameraInfo
from polled_camera.srv import GetPolledImage, GetPolledImageResponse
from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import CameraInfoManager

global image_publisher
global camera_info_publisher
global camera_info
global camera_stream
global mutex
mutex = Lock()


def name():
    return 'webcam'


def capture_image(getPolledImage):

    global camera_info

    rospy.loginfo("Capturing image")

    success = False
    message = 'No photo for you'
    stamp = rospy.Time.now()

    # run the camera
    mutex.acquire()
    try:
        if not camera_stream.isOpened():
            message = 'Cannot open %s' % name()
            rospy.logerr(message)
            success = False
        else:    
            success, cv2_image = camera_stream.read()
        if success == False:
            message = "Could not read image from %s" % name()
            rospy.logerr(message)
    finally:
        mutex.release()

    if success:
        rospy.loginfo("Capture successful, publishing image")
        # convert to ROS image and publish
        try:
            ros_image = CvBridge().cv2_to_imgmsg(cv2_image, 'bgr8')
            ros_image.header.stamp = stamp
            image_publisher.publish(ros_image)
            camera_info.header.stamp = stamp
            camera_info_publisher.publish(camera_info)
            message = "Image capture completed successfully"
        except TypeError as e:
            message = "CvBrideg could not convert image: %s" % e
            rospy.logerr(message)
            success  = False

    return GetPolledImageResponse(success, message, stamp)


def image_capture():

    global image_publisher
    global camera_info_publisher
    global camera_info
    global camera_stream

    rospy.init_node(name())

    image_publisher = rospy.Publisher("image_raw", Image, queue_size = 5)
    camera_info_publisher = rospy.Publisher("camera_info", CameraInfo, queue_size = 5)
    
    camera_stream = cv2.VideoCapture(0)
    rospy.Service('request_image', GetPolledImage, capture_image)

    # set_camera_info service
    webcam_model = rospy.get_param('~webcam_model', 'e2500')
    camera_info_url = 'package://camera_driver/calibrations/%s.yaml' % webcam_model
    camera_info_manager = CameraInfoManager(webcam_model, camera_info_url)
    camera_info_manager.loadCameraInfo()
    camera_info = camera_info_manager.getCameraInfo()

    rospy.loginfo("Ready")
    rospy.spin()


if __name__ == "__main__":
    image_capture()
    
