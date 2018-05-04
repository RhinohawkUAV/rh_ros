#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from pathfinding.msg import Input, Vehicle
from rosConstants import PATHFINDER_INPUT_TOPIC, PATHFINDER_DEBUG_NODE_ID


def inputCallback(input):
    print input

def setupPublisher():
    rospy.Publisher(PATHFINDER_INPUT_TOPIC, Input, queue_size=100)

if __name__ == '__main__':
    try:
        rospy.init_node(PATHFINDER_DEBUG_NODE_ID, anonymous=True)
        setupPublisher()
        input = Input()
        input.vehicle.acceleration=400.0
        print dir(input)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

