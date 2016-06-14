#!/usr/bin/env python

import cv2
import rospy
import tf
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo

global tf_listener
global cv_bridge
global image_publisher
global camera_projection_matrix
camera_projection_matrix = None

def target_image_location(image):

    circle = (5, 5, 5)
    source_frame = 'local_origin'
    target_frame = 'nikon'
    point = Point(x=3., y=3., z=0.5)

    header = Header(frame_id=source_frame, seq=image.header.seq, stamp=image.header.stamp)
    point_stamped = PointStamped(point=point, header=header)

    if tf_listener.canTransform(source_frame, target_frame, image.header.stamp):
        transform = tf_listener.lookupTransform(source_frame, target_frame, image.header.stamp)
        #rospy.loginfo("transform: %s", transform)
        # to camera frame
        point = tf_listener.transformPoint(target_frame, point_stamped)
        #rospy.loginfo("point in nikon frame: %s", point)
        # to camera normalized coordinates, in homogenous coordinates
        point = np.array([-point.point.y, -point.point.z, point.point.x, point.point.x])
        point = point / point[2]
        point.shape = (4, 1)
        #rospy.loginfo("point in camera coordinates: %s", point)
        # to image coordinates
        point = np.dot(camera_projection_matrix, point)
        rospy.loginfo("point in image coordinates: (%s, %s)", point[0][0], point[1][0])
        circle = (point[0][0], point[1][0], 5)
        
    else:
        rospy.loginfo("Cannot transform")

    return circle

def process_image(image):
    rospy.loginfo("process_image")
    if camera_projection_matrix is None:
        return
    cv_image = cv_bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    (x, y, radius) = target_image_location(image)
    cv2.circle(cv_image, (int(x), int(y)), int(radius), (255, 255, 0), 3)
    annotated_image_msg = cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    image_publisher.publish(annotated_image_msg)

def name():
    return "target_to_image"

def process_camera_info(camera_info_msg):
    global camera_projection_matrix
    if camera_projection_matrix is None:
        rospy.loginfo("process_camera_info")
        camera_info = camera_info_msg
        camera_projection_matrix = np.array(camera_info_msg.P)
        camera_projection_matrix.shape = (3, 4)
        rospy.loginfo("camera_projection_matrix: %s", camera_projection_matrix)

def annotate_images():
    
    global tf_listener 
    global cv_bridge
    global image_publisher
   
    rospy.init_node(name())
    rospy.loginfo("%s starting" % name())
    cv_bridge = CvBridge()
    image_publisher = rospy.Publisher("%s/image_annotated" % name(), Image, queue_size = 2)

    # hook up to tf and wait for frames to be available
    tf_listener = tf.TransformListener()
    rospy.loginfo("Checking for frames")

    # process images as they are received
    rospy.Subscriber("camera_info", CameraInfo, process_camera_info)
    rospy.Subscriber("image", Image, process_image)
    rospy.spin()

if __name__ == "__main__":
    annotate_images()
