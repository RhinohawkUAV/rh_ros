#!/usr/bin/env python

import rospy
from polled_camera.srv import GetPolledImage

def get_polled_image():
    rospy.init_node("camera_pump")
    service_name = 'request_image'
    print 'Waiting for service %s' % service_name
    rospy.wait_for_service(service_name)
    print 'Service %s is up' % service_name
    get_polled_image = rospy.ServiceProxy(service_name, GetPolledImage)
    rate = rospy.Rate(1.0/2)
    while True:
        rospy.loginfo('Requesting pic')
        get_polled_image()
        rate.sleep()

if __name__ == '__main__':
    get_polled_image()
