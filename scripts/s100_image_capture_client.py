#!/usr/bin/env python

import rospy
from rhino_hawk.srv import *

def trigger_capture():
    rospy.wait_for_service('capture_image')
    try:
        capture_image = rospy.ServiceProxy('capture_image', S100ImageCapture)
        response = capture_image()
        print "Response: %s" % response.imageFilename
    except rospy.ServiceException, e:
        print "Service call failed: %s"% e


if __name__ == '__main__':
    trigger_capture()
