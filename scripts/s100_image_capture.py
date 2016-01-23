#!/usr/bin/env python

import subprocess
from threading import Thread, Lock

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty


def working_dir():
    return '/tmp'

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


def capture_image(request):
    global publisher
    global mutex
    mutex.acquire()
    try:
        print 'Taking photo'
        filename = take_photo()
    finally:
        mutex.release()
    print "Photo complete: %s" % filename
    publisher.publish(filename)


def s100_image_capture():
    global publisher
    rospy.init_node("s100")
    rospy.Subscriber("image_trigger", Empty, capture_image)
    publisher = rospy.Publisher("image_filename", String, queue_size = 10)
    print "Ready"
    rospy.spin()


if __name__ == "__main__":
    s100_image_capture()
    
