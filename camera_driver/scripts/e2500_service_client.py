#!/usr/bin/env python

import rospy
from polled_camera.srv import GetPolledImage

def get_polled_image():
    rospy.init_node("e2500_client")
    print 'Waiting for service'
    rospy.wait_for_service('/e2500/request_image')
    print 'Service is up'
    get_polled_image = rospy.ServiceProxy('/e2500/request_image', GetPolledImage)
    rate = rospy.Rate(1.0/2)
    while True:
        rospy.loginfo('Requesting pic')
        get_polled_image()
        rate.sleep()

if __name__ == '__main__':
    get_polled_image()
