#!/usr/bin/env python

import rospy
from polled_camera.srv import GetPolledImage

def get_polled_image():
    print 'Waiting for service'
    rospy.wait_for_service('/gopro/request_image')
    print 'Service is up'
    print 'Requesting pic'
    get_polled_image = rospy.ServiceProxy('/gopro/request_image', GetPolledImage)
    print get_polled_image()
    print 'Pic complete'

if __name__ == '__main__':
    get_polled_image()
