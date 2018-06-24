#!/usr/bin/env python


#
# For testing a camera image to real world coordinates
# For a camera calibration and pose, and a know target pose
# make sure the calcs match the target
#


import tf
import rospy
import numpy as np

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo


global location_publisher

global inverse_camera_projection_matrix
inverse_camera_projection_matrix = None

global position, orientation
position = [0., 0., 0.770]
# 'sxyz'
orientation = [0., 0., 0.]

def name():
    return "camera_standalone"

def process_camera_info(camera_info_msg):
    global inverse_camera_projection_matrix
    if inverse_camera_projection_matrix is None:
        rospy.loginfo("process_camera_info")
        inverse_camera_projection_matrix = np.array(camera_info_msg.P + (0., 0., 0., 1.))
        rospy.loginfo("camera_projection_matrix:\n %s", inverse_camera_projection_matrix)
        inverse_camera_projection_matrix.shape = (4, 4)
        inverse_camera_projection_matrix = np.linalg.inv(inverse_camera_projection_matrix)
        rospy.loginfo("inverse_camera_projection_matrix:\n %s", inverse_camera_projection_matrix)


def process_location(point_stamped):

    rospy.loginfo("process_location:\n%s", point_stamped)
    if inverse_camera_projection_matrix is None:
        rospy.loginfo("process_location:\n%s", point_stamped)
        return

    # Normalized image coordinates, homogenous coordinates
    image_target = np.array([point_stamped.point.x, point_stamped.point.y, 1., 1.])

    # 2D to 3D normalized
    target = np.dot(inverse_camera_projection_matrix, image_target)

    # Change to vehicle coordinate system
    target = [target[2], -target[0], -target[1], 1]
    
    # We also need to camera center point in vehicle coordinates
    camera = np.array([0., 0., 0., 1.])

    # get camera frame
    frame = tf.transformations.compose_matrix(translate=position, angles=orientation)
    
    # rotate points from camera frame to local frame
    camera = np.dot(frame, camera)
    target = np.dot(frame, target)
    
    rospy.loginfo("camera in local:%s", camera)
    rospy.loginfo("target in local:%s", target)

    scale = - camera[2] / (target[2] - camera[2])
    joe = scale * target + (1. - scale) * camera

    Point(x=joe[0], y=joe[1], z=joe[2])
    header = Header(frame_id='local_origin', seq=point_stamped.header.seq, stamp=point_stamped.header.stamp)
    joe = PointStamped(point=point, header=header)
    location_publisher.publish(joe)
        
    else:
        rospy.loginfo("Not transforming")



def camera_standalone():
    global location_publisher
    rospy.init_node(name())
    rospy.loginfo("%s starting" % name())
    
    # publish 3D locations in local_origin frame
    location_publisher = rospy.Publisher("position", PointStamped, queue_size = 10)
    
    # subscribe to get camera calibration
    rospy.Subscriber("camera_info", CameraInfo, process_camera_info)
    
    # subscribe to 2D image locations
    rospy.Subscriber("image_location", PointStamped, process_location)
    rospy.spin()

    ros:spin()


if __name__ == "__main__":
    camera_standalone()
