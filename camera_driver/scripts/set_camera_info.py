#!/usr/bin/env python

import rospy
from sensor_msgs.srv import SetCameraInfo
from sensor_msgs.msg import CameraInfo


def get_polled_image():
    topic = 'set_camera_info'
    print 'Waiting for service: ', topic
    rospy.wait_for_service(topic)
    print 'Service is up'
    print 'Setting info'
    set_camera_info = rospy.ServiceProxy(topic, SetCameraInfo)
    print set_camera_info(CameraInfo())
    print 'Info set'


if __name__ == '__main__':
    get_polled_image()
