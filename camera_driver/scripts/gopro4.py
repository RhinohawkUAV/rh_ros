#!/usr/bin/env python

# Implement Polled Camera Node for GoPro Hero4
#
# The Hero4 protocol is adapted from https://github.com/KonradIT/GoProStream.git 

# http://wiki.ros.org/camera_drivers
#
# Services 
#      request_image (polled_camera/GetPolledImage)
#      set_camera_info (sensor_msgs/SetCameraInfo) 
#
# Published Topics:
#       /gopro/image_raw (sensor_msgs/Image)
#       /gopro/camera_info  (sensor_msgs/CameraInfo)


from threading import Lock
import os
import logging
import urllib2
import json
import socket
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
global mutex
global camera_stream
mutex = Lock()

MAX_FAILURES_BEFORE_RESTART = 10
UDP_IP = "10.5.5.9"
UDP_PORT = 8554
KEEP_ALIVE_RATE = 0.25 # send keep-alive message every 4 seconds

num_failures = 0

def name():
    return 'gopro'


def init_stream():
    rospy.loginfo("Starting gopro stream...")
    try:
        response = urllib2.urlopen("http://10.5.5.9/gp/gpControl/execute?p1=gpStream&a1=proto_v2&c1=restart")
        data = json.load(response)
        rospy.loginfo("got json: %s"%data)
        if ("status" not in data) or data["status"]!="0":
            rospy.logerr("Cannot start gopro stream")
        else:
            global camera_stream
            camera_stream = cv2.VideoCapture('udp://@10.5.5.9:8554');
            if not camera_stream.isOpened():
                message = 'Cannot open gopro stream'
                rospy.logerr(message)
                success = False
    except urllib2.URLError as e:
        rospy.logerr("Error starting gopro stream: %s" % e)


def send_keep_alive():
    message = "_GPHD_:%u:%u:%d:%1lf\n" % (0, 0, 2, 0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(message, (UDP_IP, UDP_PORT))


def capture_image(getPolledImage):

    global num_failures
    global camera_info
    global camera_stream

    rospy.loginfo("Capturing image")

    success = False
    message = 'No photo for you'
    stamp = rospy.Time.now()

    # run the camera
    mutex.acquire()
    try:
        if not camera_stream.isOpened():
            message = 'Cannot open gopro. Attempting to restart stream...'
            rospy.logerr(message)
            success = False
            num_failures += 1
        else:    
            success, cv2_image = camera_stream.read()
        if success == False:
            message = "Could not read image from gopro"
            rospy.logerr(message)
            num_failures += 1
    finally:
        mutex.release()

    if success:
        num_failures = 0
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
            message = "CvBridge could not convert image: %s" % e
            rospy.logerr(message)
            success  = False
    else:
        if num_failures>MAX_FAILURES_BEFORE_RESTART:
            init_stream()

    return GetPolledImageResponse(success, message, stamp)


def image_capture():

    global image_publisher
    global camera_info_publisher
    global camera_info

    rospy.loginfo("Initializing GoPro Hero4 stream...")
    rospy.init_node(name())

    image_publisher = rospy.Publisher("image_raw", Image, queue_size = 2)
    camera_info_publisher = rospy.Publisher("camera_info", CameraInfo, queue_size = 2)
    
    init_stream()
    rospy.Service('request_image', GetPolledImage, capture_image)

    # set_camera_info service
    camera_info_url = 'package://camera_driver/calibrations/%s.yaml' % name()
    camera_info_manager = CameraInfoManager(name(), camera_info_url)
    camera_info_manager.loadCameraInfo()
    camera_info = camera_info_manager.getCameraInfo()

    rospy.loginfo("Ready")

    rate = rospy.Rate(KEEP_ALIVE_RATE)
    while True:
        rospy.logdebug('Sending stream keep-alive message to gopro')
        send_keep_alive()
        rate.sleep()


if __name__ == "__main__":
    image_capture()
    
