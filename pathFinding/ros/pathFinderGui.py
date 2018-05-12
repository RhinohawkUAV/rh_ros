from pathfinding.msg._Scenario import Scenario
import rospy

from engine.interface.fileUtils import loadInput, SCENARIO_KEY
import messageUtils
from rosConstants import PATHFINDER_INPUT_TOPIC


def setupPublisher():
    return rospy.Publisher(PATHFINDER_INPUT_TOPIC, Scenario, queue_size=100)


def main(loadPath):
    try:
        rospy.init_node("pathfinderguinode", anonymous=True)
        publisher = setupPublisher()
        inputDict = loadInput(loadPath)
        scenario = inputDict[SCENARIO_KEY]
        outputMsg = messageUtils.scenarioToMsg(scenario)
        while not rospy.is_shutdown():        
            publisher.publish(outputMsg)
            rate = rospy.Rate(0.5)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

