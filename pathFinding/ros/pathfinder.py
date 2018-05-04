#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from pathfinding.msg import Input
from rosConstants import PATHFINDER_INPUT_TOPIC, PATHFINDER_DEBUG_NODE_ID

def inputCallback(input):
    print input

def setupInputListener():
    rospy.Subscriber(PATHFINDER_INPUT_TOPIC, Input, inputCallback, queue_size=100)

if __name__ == '__main__':
    try:
        rospy.init_node(PATHFINDER_DEBUG_NODE_ID, anonymous=True)
        setupInputListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass