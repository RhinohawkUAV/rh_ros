#!/usr/bin/env python

# Implement Polled Camera Node for Nikon D5100
#
# http://wiki.ros.org/camera_drivers
#
# Services 
#      request_image (polled_camera/GetPolledImage)
#      set_camera_info (sensor_msgs/SetCameraInfo) 
#
# Published Topics:
#       /d5100/image_raw (sensor_msgs/Image)


#
# This node writes temporary file.  Best if we write to a ramdisk.  Make one like this:
#
# root@ubuntu:~# mkdir /tmp/ramdisk
# root@ubuntu:~# chmod 777 /tmp/ramdisk/
# root@ubuntu:~# mount -t tmpfs -o size=256M tmpfs /tmp/ramdisk/
#

import subprocess
from threading import Lock
import tempfile
import os

import cv2 

import rospy
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo
from polled_camera.srv import GetPolledImage, GetPolledImageResponse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo
from camera_info_manager import CameraInfoManager



global image_raw_publisher
global camera_info_publisher
global camera_info
global mutex
mutex = Lock()

def working_dir():
    return '/tmp/ramdisk'

def name():
    return 'd5100'

def take_photo():
    rospy.loginfo('Capturing image')
    img = subprocess.check_output(["gphoto2", "--capture-image-and-download", "--stdout"]) 
    (fd, path) = tempfile.mkstemp(dir=working_dir(), suffix=".jpg")
    f = os.fdopen(fd, 'w')
    f.write(img)
    f.close()
    return path

def capture_image(getPolledImage):

    global camera_info

    filename = 'none'
    success = False
    message = 'No photo for you'
    stamp = rospy.Time.now()

    # run the camera
    mutex.acquire()
    try:
        rospy.loginfo('Taking photo')
        filename = take_photo()
        success = True
        message = filename
    except subprocess.CalledProcessError as e:
        rospy.logerr("Could not capture image: %s", e)
    except CvBridgeError as e:
        rospy.logerr("Could not capture image: %s", e)
    finally:
        mutex.release()

    rospy.loginfo("Photo complete: %s", filename)

    if success:
        # convert to ROS image and publish
        cv2_image = cv2.imread(filename)
        try:
            ros_image = CvBridge().cv2_to_imgmsg(cv2_image, 'bgr8')
            ros_image.header.stamp = stamp
            image_raw_publisher.publish(ros_image)
            camera_info.header.stamp = stamp
            camera_info_publisher.publish(camera_info)
        except TypeError as e:
            rospy.logerr("CvBrideg could not convert image: %s", e)
            success  = False
        os.remove(filename)

    return GetPolledImageResponse(success, message, stamp)


def d5100_image_capture():

    global image_raw_publisher
    global camera_info_publisher
    global camera_info

    rospy.init_node(name())
    
    # Tell camera to write images to RAM instead of media card
    try:
        subprocess.check_call(["gphoto2", "--set-config",  "/main/settings/capturetarget=0"])
    except subprocess.CalledProcessError as e:
        rospy.logfatal("Could not configure camera: %s", e)
        return

    image_raw_publisher = rospy.Publisher(name() + "/image_raw", Image, queue_size = 10)
    camera_info_publisher = rospy.Publisher(name() + "/camera_info", CameraInfo, queue_size = 5)

    rospy.Service(name() + '/request_image', GetPolledImage, capture_image)

    # set_camera_info service
    camera_info_url = 'package://camera_driver/calibrations/%s.yaml' % name()
    camera_info_manager = CameraInfoManager(name(), camera_info_url, name())
    camera_info_manager.loadCameraInfo()
    camera_info = camera_info_manager.getCameraInfo()

    rospy.loginfo("Ready")
    rospy.spin()


if __name__ == "__main__":
    d5100_image_capture()
    
