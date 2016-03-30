#!/usr/bin/env python

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from camera_driver.srv import SetBlobInfo

global image_publisher
global mask_publisher
global contours_publisher
global bridge


def name():
    return "blob_detector"

# Color Mask window
#mask_window = 'Color Mask'
#cv2.namedWindow(mask_window, cv2.WINDOW_NORMAL)

#Picker
control_window = "Picker"
window = cv2.namedWindow(control_window)


##################
# Color filtering
#################

# Low cut off

hue_low = 57
def set_hue_low(new_value):
    global hue_low
    hue_low = new_value
cv2.createTrackbar('Hue_Low', control_window, hue_low, 179, set_hue_low)

hue_high = 90
def set_hue_high(new_value):
    global hue_high
    hue_high = new_value
cv2.createTrackbar('Hue_High', control_window, hue_high, 179, set_hue_high)

saturation_low = 57
def set_saturation_low(new_value):
    global saturation_low
    saturation_low = new_value
cv2.createTrackbar('Saturation_Low', control_window, saturation_low, 255, set_saturation_low)

saturation_high = 232
def set_saturation_high(new_value):
    global saturation_high
    saturation_high = new_value
cv2.createTrackbar('Saturation_High', control_window, saturation_high, 255, set_saturation_high)

value_low = 23
def set_value_low(new_value):
    global value_low
    value_low = new_value
cv2.createTrackbar('Value_Low', control_window, value_low, 255, set_value_low)

value_high = 252
def set_value_high(new_value):
    global value_high
    value_high = new_value
cv2.createTrackbar('Value_High', control_window, value_high, 255, set_value_high)


def set_blob_info(msg):
    rospy.loginfo("set_blob_info")
    return True


def process_image(image):
    rospy.loginfo("process_image")

    # Convert to opencv image
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    
    # process image and show
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (hue_low, saturation_low, value_low), (hue_high, saturation_high, value_high))
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, hierarchy = cv2.findContours(np.copy(mask), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # publish binary image
    mask_message = bridge.cv2_to_imgmsg(mask, encoding="mono8")
    mask_publisher.publish(mask_message)

    # publish binary image with contours
    rospy.loginfo("Countours: %s", len(contours))
    contours_image = np.copy(cv_image)
    cv2.drawContours(contours_image, contours, -1, (0,255,0), 3)
    contours_message = bridge.cv2_to_imgmsg(contours_image, encoding="bgr8")
    contours_publisher.publish(contours_message)

    # publish image annotated with circle for blob location
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 3)
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    image_publisher.publish(image_message)


def detect_blobs():

    global image_publisher
    global mask_publisher
    global contours_publisher
    global bridge

    rospy.init_node(name())

    image_publisher = rospy.Publisher("image_blob", Image, queue_size = 2)
    mask_publisher = rospy.Publisher("image_mask", Image, queue_size = 2)
    contours_publisher = rospy.Publisher("image_contours", Image, queue_size = 2)
    rospy.Subscriber("image", Image, process_image)
    service = rospy.Service('set_blob_info', SetBlobInfo, set_blob_info)
    
    bridge = CvBridge()

    rospy.loginfo("Ready... 'spinning'")
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        cv2.waitKey(1)
        r.sleep()


if __name__ == "__main__":
    detect_blobs()
