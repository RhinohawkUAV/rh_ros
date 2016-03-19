#!/usr/bin/env python

#
# Camera driver for a web cam attached to the computer
# webcam/image_raw

import cv2 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def name():
    return 'webcam'

def stream_video():
    rospy.init_node(name())
    publisher = rospy.Publisher(name() + "/image_raw", Image, queue_size = 5)
    rospy.loginfo('Starting video stream')
    camera_stream = cv2.VideoCapture(0);
    window_name = 'webcam'
    #cv2.namedWindow(window_name)
    if not camera_stream.isOpened():
        rospy.logerr("Cannot open web cam")
        return
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        success, cv2_image = camera_stream.read()
        if success:
            rospy.loginfo("Publishing image")
            #cv2.imshow(window_name, cv2_image)
            #cv2.waitKey(1)
            ros_image = CvBridge().cv2_to_imgmsg(cv2_image, 'bgr8')
            ros_image.header.stamp = rospy.Time.now()
            publisher.publish(ros_image)
        else:
            rospy.logerr('Cannot capture image')
        rate.sleep()

if __name__ == "__main__":
    stream_video()
