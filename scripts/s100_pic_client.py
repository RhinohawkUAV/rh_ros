#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Listen to topic      
#   /s100/image_raw (sensor_msgs/Image)
# and show images 

def pic_handler(image):
    print 'Inbound!'
    cv_image = CvBridge().imgmsg_to_cv2(image, 'bgr8')
    cv2.namedWindow('S100 Pics', cv2.CV_WINDOW_AUTOSIZE)
    cv2.imshow("S100 Pics", cv_image)
    cv2.waitKey(5)

def show_pics():
    rospy.init_node('s100PicClient', anonymous=True)
    rospy.Subscriber('/s100/image_raw', Image, pic_handler)
    print 'Listening'
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    show_pics()
