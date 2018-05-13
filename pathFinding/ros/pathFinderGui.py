from pathfinding.msg._Scenario import Scenario
from pathfinding.srv._InitiateFindPath import InitiateFindPath
import rospy

from engine.interface.fileUtils import loadInput, SCENARIO_KEY, VEHICLE_KEY
import gui
from gui.pathFinder.pathFindViewer import PathFindViewer
import messageUtils
from ros.rosConstants import INITIATE_FINDPATH_SERVICE
from ros.rosPathFinderInterface import RosPathFinderInterface
from rosConstants import PATHFINDER_INPUT_TOPIC


def setupPublisher():
    return rospy.Publisher(PATHFINDER_INPUT_TOPIC, Scenario, queue_size=100)

    
def main(loadPath):
    rospy.init_node("pathfinderguinode", anonymous=True)
    publisher = setupPublisher()
    inputDict = loadInput(loadPath)
    scenario = inputDict[SCENARIO_KEY]
    vehicle = inputDict[VEHICLE_KEY]
    
    PathFindViewer(RosPathFinderInterface(), 800, 800)
    gui.startGUI()
        
#         while not rospy.is_shutdown():        
#             text = raw_input("")
#             initiateFindPath(scenarioMsg, vehicleMsg)
            
#         while not rospy.is_shutdown():        
#             publisher.publish(scenarioMsg)
#             rate = rospy.Rate(0.5)
#             rate.sleep()

