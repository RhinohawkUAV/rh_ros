import os
from pathfinding.msg._Scenario import Scenario
from pathfinding.srv._InitiateFindPath import InitiateFindPath, \
    InitiateFindPathResponse
from roscpp.srv._Empty import Empty, EmptyResponse
import rospy
from threading import Thread
import threading
import traceback

from engine.dynamicPathFinder import DynamicPathFinder
from engine.interface.vehicleInput import VehicleInput
import gui
import messageUtils
from ros.rosConstants import PATHFINDER_INPUT_TOPIC, PATHFINDER_NODE_ID, INITIATE_FINDPATH_SERVICE, \
    STEP_FINDPATH_SERVICE

lock = threading.Lock()
pathFinder = None


def initiateFindPathRequest(request):
    global lock
    with lock:
        global pathFinder
        scenario = messageUtils.msgToScenario(request.scenario)
        vehicle = messageUtils.msgToVehicle(request.vehicle)
        pathFinder = DynamicPathFinder(scenario, vehicle)
        print "Initialized path finder with new problem"
        return InitiateFindPathResponse()


def stepFindPathRequest(request):

    def step():
        global lock
        with lock:
            if pathFinder is None:
                return 
            while not pathFinder.isDone() and not pathFinder.step():
                pass
            print "Completed step"

    Thread(target=step).start()
    print "Started step"
    return EmptyResponse()
    
# def showPathFind(pathFinder, scenario):        
#     # Show a slight extra buffer around the border
#     bounds = scenario.calcBounds()
#     centerX = (bounds[0] + bounds[2]) / 2.0
#     centerY = (bounds[1] + bounds[3]) / 2.0
#     rangeX = bounds[2] - bounds[0]
#     rangeY = bounds[3] - bounds[1]
#     PathFindViewer(pathFinder, scenario, 800, 800, centerX, centerY, rangeX + 1,
#                                                          rangeY + 1)


def main():
    rospy.init_node(PATHFINDER_NODE_ID, anonymous=True)
    # Setup path-finder topic for input via topic
#     rospy.Subscriber(PATHFINDER_INPUT_TOPIC, Scenario, inputCallback, queue_size=100)
    rospy.Service(INITIATE_FINDPATH_SERVICE, InitiateFindPath, initiateFindPathRequest)
    rospy.Service(STEP_FINDPATH_SERVICE, Empty, stepFindPathRequest)
    rospy.spin()
    
