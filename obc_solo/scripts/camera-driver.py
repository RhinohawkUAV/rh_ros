#!/usr/bin/env python

import cv2
import socket
import rospy
import sys

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo
from camera_info_manager import CameraInfoManager



class Gopro:

    def __init__(self):
        self.image_raw_publisher = rospy.Publisher("image_raw", Image, queue_size = 10)
        self.camera_info_publisher = rospy.Publisher("camera_info", CameraInfo, queue_size = 5)
        name = 'gopro'
        camera_info_url = 'package://camera_driver/calibrations/%s.yaml' % name
        camera_info_manager = CameraInfoManager(name, camera_info_url, name)
        camera_info_manager.loadCameraInfo()
        self.camera_info = camera_info_manager.getCameraInfo()
        self.sololink_config = rospy.myargv(argv=sys.argv)[1]
        rospy.loginfo("Solo link config %s" % self.sololink_config)

    def stream(self):
        rospy.loginfo("Requesting video stream from solo camera")
        try:
            socket.setdefaulttimeout(5)
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                self.socket.connect(('10.1.1.1', 5502))
                rospy.loginfo("Connected to solo camera")
                self.stream_frames()
            except socket.timeout:
                rospy.logerr("Timed out connecting to solo camera")
            except socket.error:
                rospy.logerr("Socket error")
        finally:
            if self.socket is not None:
                self.socket.close()
                self.socker = None
        rospy.loginfo("Disconnected from solo camera")

    def stream_frames(self):
        rospy.loginfo("Opening stream")
        stream = cv2.VideoCapture(self.sololink_config)
        frame_read_stat = True
        while(stream.isOpened() and frame_read_stat):
            frame_read_stat, cv2_image = stream.read()
            if frame_read_stat:
                try:
                    ros_image = CvBridge().cv2_to_imgmsg(cv2_image, 'bgr8')
                    stamp = rospy.Time.now()
                    ros_image.header.stamp = stamp
                    self.image_raw_publisher.publish(ros_image)
                    self.camera_info.header.stamp = stamp
                    self.camera_info_publisher.publish(self.camera_info)
                except TypeError as e:
                    rospy.logerr("CvBrideg could not convert image: %s", e)
            else:
                rospy.logerr("Could not capture video frame")
        else:
            rospy.logerr("VideoCaputure stream is closed")
        

if __name__ == "__main__":
    rospy.init_node('cameraDriver')
    rospy.sleep(1)
    rospy.loginfo("Camera_driver staring up")
    Gopro().stream()
    ros.spin()
