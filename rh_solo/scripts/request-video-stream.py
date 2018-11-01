#!/usr/bin/env python

import os
import socket
import rospy
import time

class Streamer:

    def __init__(self):
        pass

    def stream(self):
        rospy.loginfo("Requesting video stream from solo camera...")
        try:
            socket.setdefaulttimeout(5)
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                self.socket.connect(('10.1.1.1', 5502))
                rospy.loginfo("Connected to solo camera")
                time.sleep(1e9)
            except socket.timeout:
                rospy.logerr("Timed out connecting to solo camera")
            except socket.error as msg:
                rospy.logerr("Socket error: %s"%msg)
        finally:
            if self.socket is not None:
                self.socket.close()
                self.socket = None
        rospy.logdebug("Disconnected from solo camera")


if __name__ == "__main__":
    rospy.init_node('RequestVideoStream')
    rospy.sleep(1)
    rospy.logdebug("Sarting video stream")
    r = rospy.Rate(0.1)
    gopro = Streamer()
    while not rospy.is_shutdown():
        gopro.stream()
        r.sleep()

