#!/usr/bin/env python

import subprocess
from threading import Lock

import cv2 

import rospy
from sensor_msgs.msg import Image
from polled_camera.srv import GetPolledImage, GetPolledImageResponse
from cv_bridge import CvBridge, CvBridgeError


# Implement Polled Camera Node for S100
#
# http://wiki.ros.org/camera_drivers
#
# Service 
#      request_image (polled_camera/GetPolledImage)
#
# Published Topics:
#       /s100/image_filename (sensor_msgs/Image)


def working_dir():
    return '/tmp'

def name():
    return 's100'

def take_photo():
    
    print 'Set record mode'
    subprocess.check_output(["ptpcam", "--chdk=mode 1"], cwd = working_dir()) 

    prior_image_name = get_latest_photo()[1]
    print 'Prior image: %s' % prior_image_name

    print 'Shooting'
    subprocess.check_output(["ptpcam", "--chdk=lua shoot()"], cwd = working_dir()) 
    subprocess.check_output(["ptpcam", "--chdk=script-status"], cwd = working_dir()) 

    count = 0
    max_count = 5
    sleep_period = 1

    latest_image_name = get_latest_photo()[1]
    print 'Latest image: %s' % latest_image_name

    while prior_image_name == latest_image_name and count < max_count:
        # try again
        count = count + 1
        latest_image_name = get_latest_photo()[1]
        print 'Latest image: %s count: %s' % (latest_image_name, count)
    handle = get_latest_photo()[0]

    print 'Get file'
    arg = "--get-file=" + handle
    subprocess.check_output(["ptpcam", arg], cwd = working_dir()) 

    filename = "%s/%s" % (working_dir(), latest_image_name)
    print "Returning: " + filename
    return filename


def get_latest_photo():

    # print 'get_latest_photo: list files'
    res = subprocess.check_output(["ptpcam", "--list-files"], cwd = working_dir()) 
    last_line = res.split('\n')[-3]
    # print 'get_latest_photo: last_line: %s' % last_line
    # we re missing a check for the no photox case
    handle = last_line.split(':')[0]
    # print 'get_latest_photo: handle: %s' % handle
    filename = last_line.split('\t')[-1]
    # print "get_latest_photo: filename: %s" % filename
    return (handle, filename)


global publisher
global mutex
mutex = Lock()


def capture_image(getPolledImage):
    global publisher
    global mutex
    success = False
    message = 'No photo for you'

    # run the camera
    mutex.acquire()
    try:
        print 'Taking photo'
        filename = take_photo()
        success = True
        message = filename
    finally:
        mutex.release()

    print "Photo complete: %s" % filename

    # convert to ROS image and publish
    cv2_image = cv2.imread(filename)
    ros_image = CvBridge().cv2_to_imgmsg(cv2_image, 'bgr8')
    publisher.publish(ros_image)
    return GetPolledImageResponse(success, message, rospy.Time.now())


def s100_image_capture():
    global publisher
    rospy.init_node(name())
    publisher = rospy.Publisher(name() + "/image_raw", Image, queue_size = 10)
    rospy.Service(name() + '/request_image', GetPolledImage, capture_image)
    print "Ready"
    rospy.spin()


if __name__ == "__main__":
    s100_image_capture()
    
