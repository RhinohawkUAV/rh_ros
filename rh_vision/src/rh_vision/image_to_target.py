#!/usr/bin/env python


import rospy
import tf
import numpy as np

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo

import tf2_msgs.msg

global location_publisher
global inverse_camera_projection_matrix
inverse_camera_projection_matrix = None
global tf_listener


def process_camera_info(camera_info_msg):
    global inverse_camera_projection_matrix
    if inverse_camera_projection_matrix is None:
        rospy.loginfo("process_camera_info")
        inverse_camera_projection_matrix = np.array(camera_info_msg.P + (0., 0., 0., 1.))
        rospy.loginfo("camera_projection_matrix:\n %s", inverse_camera_projection_matrix)
        inverse_camera_projection_matrix.shape = (4, 4)
        inverse_camera_projection_matrix = np.linalg.inv(inverse_camera_projection_matrix)
        rospy.loginfo("inverse_camera_projection_matrix:\n %s", inverse_camera_projection_matrix)

def name():
    return "image_to_target"

def process_location(point_stamped):

    rospy.logdebug("process_location:\n%s", point_stamped)
    
    # Normalized image coordinates, homogenous coordinates
    image_target = np.array([point_stamped.point.x, point_stamped.point.y, 1., 1.])
    # rospy.loginfo("target 2D normalized:%s", image_target)

    # 2D to 3D normalized
    target = np.dot(inverse_camera_projection_matrix, image_target)
    # rospy.loginfo("target 3D normalized:%s", target)

    # Change to vehicle coordinate system
    target = [target[2], -target[0], -target[1], 1]
    # rospy.loginfo("target 3D vehicle coordinates:%s", target)
    
    # We also need to camera center point
    camera = np.array([0., 0., 0., 1.])

    # Get the frame translation to local_origin
    source_frame = 'gopro'
    target_frame = 'local_origin'
    
    if tf_listener.canTransform(source_frame, target_frame, point_stamped.header.stamp):
        rospy.logdebug("Transforming")

        target = to_point_stamped(target, source_frame, point_stamped.header.seq, point_stamped.header.stamp)
        camera = to_point_stamped(camera, source_frame, point_stamped.header.seq, point_stamped.header.stamp)
        
        #rospy.loginfo("camera in nikon:\n%s", camera)
        #rospy.loginfo("target in nikon:\n%s", target)
        
        #rospy.loginfo("Transform nikon -> fcu: %s", tf_listener.lookupTransform('fcu', 'nikon', point_stamped.header.stamp))
        target = tf_listener.transformPoint('fcu', target)
        camera = tf_listener.transformPoint('fcu', camera)

        #rospy.loginfo("camera in fcu:\n%s", camera)
        #rospy.loginfo("target in fcu:\n%s", target)

        #rospy.loginfo("Transform fcu -> local: %s", tf_listener.lookupTransform('local_origin', 'fcu', point_stamped.header.stamp))
        target = tf_listener.transformPoint(target_frame, target)
        camera = tf_listener.transformPoint(target_frame, camera)

        #rospy.loginfo("camera in local_origin:\n%s", camera)
        #rospy.loginfo("target in local_origin:\n%s", target)

        # project to intersection with ground plane
        scale = - camera.point.z / (target.point.z - camera.point.z)
        # rospy.loginfo("scale:%s", scale)
        
        point = Point(x=scale * target.point.x + (1. - scale) * camera.point.x, y=scale * target.point.y + (1. - scale) * camera.point.y, z=scale * target.point.z + (1. - scale) * camera.point.z)
        header = Header(frame_id=target_frame, seq=point_stamped.header.seq, stamp=point_stamped.header.stamp)
        joe = PointStamped(point=point, header=header)
        location_publisher.publish(joe)
        
    else:
        rospy.logdebug("Not transforming")

def to_point_stamped(np_point, frame, seq, stamp):
    point = Point(x=np_point[0], y=np_point[1], z=np_point[2])
    header = Header(frame_id=frame, seq=seq, stamp=stamp)
    point_stamped = PointStamped(point=point, header=header)
    return point_stamped



def process_tf(tfm):
    global last_tf
    global latch_tf

    for transform in tfm.transforms:
        frame = transform.header.frame_id
        child = transform.child_frame_id
        name = "%s to %s" % (frame, child)
        latch_tf[name] = transform.transform

    now = rospy.Time.now()
    if now - last_tf > rospy.Duration(3):
        info = ""
        for name in sorted(latch_tf.keys()):
            frame, child = name.split(" to ")
            transform = latch_tf[name]
            t = transform.translation
            r = transform.rotation
            info += "\n  %-12s  %-7s  translation: [%2.2f,%2.2f,%2.2f]  rotation: [%2.2f,%2.2f,%2.2f,%2.2f]" % \
                    (frame, child, t.x, t.y, t.z, r.x, r.y, r.z, r.w)
        rospy.loginfo("%s", info)
        last_tf = now


def image_to_target():
    global location_publisher
    global tf_listener
    rospy.init_node(name())
    rospy.loginfo("%s starting" % name())

    # Start listening for frame information
    tf_listener = tf.TransformListener()
    
    # publish 3D locations in local_origin frame
    location_publisher = rospy.Publisher("target_position_local", PointStamped, queue_size = 10)
    
    # subscribe to get camera calibration
    rospy.Subscriber("camera_info", CameraInfo, process_camera_info)

    global last_tf
    global latch_tf
    last_tf = rospy.Time.now()
    latch_tf = {}
    rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, process_tf)

    # subscribe to 2D image locations
    rospy.Subscriber("image_location", PointStamped, process_location)
    rospy.spin()

if __name__ == "__main__":
    image_to_target()
    
