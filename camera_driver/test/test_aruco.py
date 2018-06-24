#!/usr/bin/env python
# 
# Unit tests for aruco_detector.py
#

PKG = 'camera_driver'
NAME = 'test_aruco'

import sys
import unittest
import rospy
import rostest
import os
import rospkg
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

rp = rospkg.RosPack()
global filename

class TestAruco(unittest.TestCase):
    
    def __init__(self, *args):
        super(TestAruco, self).__init__(*args)
        self.success = False
        self.targets = []
        self.bridge = CvBridge()


    def get_test_images(self):
        image_dir = os.path.join(rp.get_path(PKG), "test", "test_images")
        image_filenames = ["%s/%s"%(image_dir,f) for f in os.listdir(image_dir)]
        rospy.logdebug("Found test images: %s", image_filenames)
        return sorted(image_filenames)


    def test_aruco(self):

        global filename
        def box_callback(data):
            rospy.loginfo("Found target at (%d,%d)" % (data.point.x,data.point.y))
            self.targets.append(data)
        
        def image_callback(filepath, image):
            global filename
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            image_filepath = os.path.join(rp.get_path(PKG), "test", "test_output", filename)
            cv2.imwrite(image_filepath,cv_image)


        image_publisher = rospy.Publisher("/image", Image, queue_size = 2)
        rospy.Subscriber("/aruco/location", PointStamped, box_callback)
        rospy.Subscriber("/aruco/image/compressed", Image, image_callback)
        # wait for all node connections
        rospy.sleep(0.5)
        
        for filename in self.get_test_images():
            rospy.loginfo("Testing %s" % filename)
            image = cv2.imread(filename, 1)
            ros_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            ros_image.header.stamp = rospy.Time.now()

            self.targets = []
            image_publisher.publish(ros_image)

            rospy.sleep(0.2)
            n = len(self.targets)
            self.assertTrue(n==1,"Found %d targets instead of one"%n)


if __name__ == '__main__':
    rospy.init_node(NAME, log_level=rospy.INFO)
    rostest.rosrun(PKG, NAME, TestAruco, sys.argv)


