#!/usr/bin/env python

import os
import cv2
import socket
import rospy
import sys

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo
from camera_info_manager import CameraInfoManager

MAX_BAD_FRAMES = 100000
ENV_VAR_CAMERA_NAME = "CAMERA_NAME"

class Gopro:

    def __init__(self):
        self.image_raw_publisher = rospy.Publisher("image_raw", Image, queue_size = 10)
        self.camera_info_publisher = rospy.Publisher("camera_info", CameraInfo, queue_size = 5)
        name = 'gopro'
        if ENV_VAR_CAMERA_NAME in os.environ:
            name = os.environ[ENV_VAR_CAMERA_NAME]
        camera_info_url = 'package://camera_driver/calibrations/%s.yaml' % name
        rospy.loginfo("Camera calibration: %s" % camera_info_url)
        self.camera_info_manager = CameraInfoManager(name, camera_info_url)
        self.camera_info_manager.loadCameraInfo()
        self.camera_info = self.camera_info_manager.getCameraInfo()
        self.sololink_config = rospy.myargv(argv=sys.argv)[1]
        rospy.loginfo("Solo link config: %s" % self.sololink_config)

    def stream(self):
        rospy.loginfo("Requesting video stream from solo camera...")
        try:
            socket.setdefaulttimeout(5)
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                self.socket.connect(('10.1.1.1', 5502))
                rospy.loginfo("Connected to solo camera")
                self.stream_frames()
            except socket.timeout:
                rospy.logerr("Timed out connecting to solo camera")
            except socket.error as msg:
                rospy.logerr("Socket error: %s"%msg)
        finally:
            if self.socket is not None:
                self.socket.close()
                self.socket = None
        rospy.logdebug("Disconnected from solo camera")

    def stream_frames(self):
        rospy.loginfo("Opening video stream")
        stream = cv2.VideoCapture(self.sololink_config)
        running = True
        bad_frames = 0
        while(stream.isOpened() and running):
            frame_read_stat, cv2_image = stream.read()
            if frame_read_stat:
                # Reset the bad frame counter since we got a good frames
                if (bad_frames > 0): 
                    rospy.logerror("Recovering from %d dropped frames"%bad_frames)
                bad_frames = 0 
                # Publish the captured image
                try:
                    ros_image = CvBridge().cv2_to_imgmsg(cv2_image, 'bgr8')
                    stamp = rospy.Time.now()
                    ros_image.header.stamp = stamp
                    self.image_raw_publisher.publish(ros_image)
                    self.camera_info.header.stamp = stamp
                    self.camera_info_publisher.publish(self.camera_info)
                except TypeError as e:
                    rospy.logerr("CvBridge could not convert image: %s", e)
            else:
                #rospy.logerr("Could not capture video frame (%d unreadable frames)" % bad_frames)
                bad_frames += 1

            if bad_frames > MAX_BAD_FRAMES:
                rospy.logerr("Closing stream after encountering %d unreadable frames" % MAX_BAD_FRAMES)
                running = False

        else:
            rospy.logerr("Video stream is now closed")
        

if __name__ == "__main__":
    rospy.init_node('cameraDriver')
    rospy.sleep(1)
    rospy.logdebug("Camera driver starting up")
    r = rospy.Rate(0.2)
    gopro = Gopro()
    while not rospy.is_shutdown():
        gopro.stream()
        r.sleep()

