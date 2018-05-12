from pathfinding.msg._Scenario import Scenario
import rospy

import messageUtils
from rosConstants import PATHFINDER_INPUT_TOPIC


def inputCallback(inputMsg):
    print messageUtils.msgToScenario(inputMsg)


def setupInputListener():
    rospy.Subscriber(PATHFINDER_INPUT_TOPIC, Scenario, inputCallback, queue_size=100)


def main():
    try:
        rospy.init_node("pathfindernode", anonymous=True)
        setupInputListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
