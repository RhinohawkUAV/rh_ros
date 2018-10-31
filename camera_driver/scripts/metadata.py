#!/usr/bin/env python

import rospy
import yaml
from rh_msgs.msg import CameraMetadata
from rh_msgs.srv import GetCameraMetadata

metadata = None

def load_metadata():
    global metadata
    full_path = rospy.get_param('~camera_metadata_path')
    rospy.loginfo("Loading camera metadata: %s"%full_path)
    yaml_file = open(full_path, "r")
    m = yaml.load(yaml_file)
    metadata = CameraMetadata()
    metadata.camera_name = m['camera_name']
    metadata.focal_length = m['focal_length']
    metadata.image_width = m['image_width']
    metadata.image_height = m['image_height']
    metadata.sensor_width = m['sensor_width']
    metadata.sensor_height = m['sensor_height']


def handle_get_metadata(req):
    return metadata


if __name__ == "__main__":
    rospy.init_node("metadata")
    rospy.Service('get_metadata', GetCameraMetadata, handle_get_metadata)
    load_metadata()
    rospy.spin()




