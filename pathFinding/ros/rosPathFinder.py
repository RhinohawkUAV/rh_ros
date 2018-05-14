from pathfinding.msg._PathDebug import PathDebug
from pathfinding.msg._PathSolution import PathSolution
from pathfinding.srv._InitiateFindPath import InitiateFindPath, \
    InitiateFindPathResponse
from roscpp.srv._Empty import Empty, EmptyResponse
import rospy
from threading import Thread
import threading

from engine.dynamicPathFinder import DynamicPathFinder
import messageUtils
from ros.rosConstants import PATHFINDER_NODE_ID, INITIATE_FINDPATH_SERVICE, \
    STEP_FINDPATH_SERVICE, PATHFINDER_DEBUG_TOPIC, ROS_QUEUE_SIZE, \
    PATHFINDER_SOLUTION_TOPIC


class RosPathFinder:

    def __init__(self):
        self._lock = threading.Lock()
        self._pathFinder = None
        rospy.init_node(PATHFINDER_NODE_ID, anonymous=True)
        rospy.Service(INITIATE_FINDPATH_SERVICE, InitiateFindPath, self.initiateFindPathRequest)
        rospy.Service(STEP_FINDPATH_SERVICE, Empty, self.stepFindPathRequest)
        self._pathDebugPub = rospy.Publisher(PATHFINDER_DEBUG_TOPIC, PathDebug, queue_size=ROS_QUEUE_SIZE)
        self._pathSolutionPub = rospy.Publisher(PATHFINDER_SOLUTION_TOPIC, PathSolution, queue_size=ROS_QUEUE_SIZE)

    def initiateFindPathRequest(self, request):
        with self._lock:
            scenario = messageUtils.msgToScenario(request.scenario)
            vehicle = messageUtils.msgToVehicle(request.vehicle)
            self._pathFinder = DynamicPathFinder(scenario, vehicle)
            return InitiateFindPathResponse()
    
    def stepFindPathRequest(self, request):

        def step():
            with self._lock:
                if self._pathFinder is None or self._pathFinder.isDone():
                    return
                
                if self._pathFinder.step():
                    pathSolution = PathSolution()
                    pathSolution.solutionPathSegments = messageUtils.pathSegmentListToMsg(self._pathFinder.getSolution())
                    pathSolution.finished = self._pathFinder.isDone()
                    self._pathSolutionPub.publish(pathSolution)
                else:
                    debugData = self._pathFinder.getDebugData()
                    pathDebug = messageUtils.pathDebugToMsg(*debugData)
                    self._pathDebugPub.publish(pathDebug)
    
        Thread(target=step).start()
        return EmptyResponse()

