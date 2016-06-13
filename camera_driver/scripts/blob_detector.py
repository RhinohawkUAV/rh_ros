#!/usr/bin/env python

import cv2
import numpy as np
import yaml

import rospy
import rospkg

from camera_driver.srv import SetBlobInfo

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header

global joe_location_publisher
global image_publisher
global mask_publisher
global contours_publisher
global bridge
global seq
seq = 0

def name():
    return "blob_detector"

# Color Mask window
#mask_window = 'Color Mask'
#cv2.namedWindow(mask_window, cv2.WINDOW_NORMAL)

#Picker
control_window = "Picker"
global window


##################
# Color filtering
#################

# Low cut off

global hue_low
def set_hue_low(new_value):
    global hue_low
    hue_low = new_value

global hue_high 
def set_hue_high(new_value):
    global hue_high
    hue_high = new_value

global saturation_low
def set_saturation_low(new_value):
    global saturation_low
    saturation_low = new_value

global saturation_high
def set_saturation_high(new_value):
    global saturation_high
    saturation_high = new_value

global value_low
def set_value_low(new_value):
    global value_low
    value_low = new_value

global value_high
def set_value_high(new_value):
    global value_high
    value_high = new_value

def set_blob_info(msg):
    rospy.loginfo("set_blob_info")
    package_path = rospkg.RosPack().get_path('camera_driver')
    full_path = "%s/calibrations/%s.yaml" % (package_path, name())
    data = dict(
        hue_low = hue_low,
        hue_high = hue_high,
        saturation_low = saturation_low,
        saturation_high = saturation_high,
        value_low = value_low,
        value_high = value_high
    )
    with open(full_path, 'w') as outfile:
        yaml.dump(data, outfile, default_flow_style=False)
    return True

def get_blob_info():
    rospy.loginfo("get_blob_info")

    global hue_low
    global hue_high
    global saturation_low
    global saturation_high
    global value_low
    global value_high

    package_path = rospkg.RosPack().get_path('camera_driver')
    full_path = "%s/calibrations/%s.yaml" % (package_path, name())
    rospy.loginfo("get_blob_info: loading settings from %s" % full_path)
    yaml_file = open(full_path, "r")
    blob_params = yaml.load(yaml_file)

    hue_low = blob_params['hue_low']
    hue_high = blob_params['hue_high']
    saturation_low = blob_params['saturation_low']
    saturation_high = blob_params['saturation_high']
    value_low = blob_params['value_low']
    value_high = blob_params['value_high']

    rospy.loginfo("get_blob_info: setting hue_low to %s" % hue_low)


def process_image(image):

    global seq
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

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        # Annotate image with circle for blob location
        cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 3)
        # Publish image coordinates of detected blob
        seq += 1
        header = Header(frame_id="nikon", seq=image.header.seq, stamp=image.header.stamp)
        point = Point(x=x, y=y, z=1.)
        pointStamped = PointStamped(point=point, header=header)
        joe_location_publisher.publish(pointStamped)

    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    image_publisher.publish(image_message)


def detect_blobs():

    global joe_location_publisher
    global image_publisher
    global mask_publisher
    global contours_publisher
    global bridge
    global window

    # Hoookup ROS stuff
    rospy.init_node(name())
    joe_location_publisher = rospy.Publisher("joe_location", PointStamped, queue_size = 2)
    image_publisher = rospy.Publisher("image_blob", Image, queue_size = 2)
    mask_publisher = rospy.Publisher("image_mask", Image, queue_size = 2)
    contours_publisher = rospy.Publisher("image_contours", Image, queue_size = 2)
    rospy.Subscriber("image", Image, process_image)
    service = rospy.Service('set_blob_info', SetBlobInfo, set_blob_info)
    
    # ROS to OpenCV
    bridge = CvBridge()

    # Reload blob detector parameters
    get_blob_info()

    # Bring up UI
    show_picker = rospy.get_param("~show_picker", True)
    rospy.loginfo("Show picker: %s" % show_picker)
    
    if show_picker:
        window = cv2.namedWindow(control_window)
        cv2.createTrackbar('Hue_Low', control_window, hue_low, 179, set_hue_low)
        cv2.createTrackbar('Hue_High', control_window, hue_high, 179, set_hue_high)
        cv2.createTrackbar('Saturation_Low', control_window, saturation_low, 255, set_saturation_low)
        cv2.createTrackbar('Saturation_High', control_window, saturation_high, 255, set_saturation_high)
        cv2.createTrackbar('Value_Low', control_window, value_low, 255, set_value_low)
        cv2.createTrackbar('Value_High', control_window, value_high, 255, set_value_high)

    rospy.loginfo("Ready... 'spinning'")
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if show_picker:
            cv2.waitKey(1)
        r.sleep()


if __name__ == "__main__":
    detect_blobs()
