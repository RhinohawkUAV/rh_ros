from pathfinding.msg._PathDebug import PathDebug
from pathfinding.msg._PathSolution import PathSolution
from pathfinding.srv._InitiateFindPath import InitiateFindPath, \
    InitiateFindPathResponse
from roscpp.srv._Empty import Empty, EmptyResponse
import rospy
from threading import Thread, Condition
import threading
import time

from engine.pathFinder import PathFinder
import messageUtils
from ros.rosConstants import PATHFINDER_NODE_ID, INITIATE_FINDPATH_SERVICE, \
    STEP_FINDPATH_SERVICE, PATHFINDER_DEBUG_TOPIC, ROS_QUEUE_SIZE, \
    PATHFINDER_SOLUTION_TOPIC


class TerminatedException(BaseException):
    pass


class RosPathFinder:

    def __init__(self):
        self._controlLock = Condition(threading.Lock())
        self._steps = 0
        self._terminated = False
        self._activePathFinder = None
        self._thread = Thread(target=self._run)
        self._thread.start()
        rospy.init_node(PATHFINDER_NODE_ID, anonymous=True)
        rospy.core.add_shutdown_hook(self._rosShutdown)
        rospy.Service(INITIATE_FINDPATH_SERVICE, InitiateFindPath, self._initiateFindPathRequest)
        rospy.Service(STEP_FINDPATH_SERVICE, Empty, self._stepFindPathRequest)
        self._pathDebugPub = rospy.Publisher(PATHFINDER_DEBUG_TOPIC, PathDebug, queue_size=ROS_QUEUE_SIZE)
        self._pathSolutionPub = rospy.Publisher(PATHFINDER_SOLUTION_TOPIC, PathSolution, queue_size=ROS_QUEUE_SIZE)

    def checkTerminated(self):
        if self._terminated:
            raise TerminatedException                
            
    def _rosShutdown(self, shutdownMessage):
        print "Shutting down path finder cause: " + shutdownMessage
        with self._controlLock:
            self._terminated = True
            self._controlLock.notifyAll()

    def _run(self):
        try:
            while True:
                pathFinder = self._getNextStep()
                self._performStep(pathFinder)
        except TerminatedException:
            pass
        
    def _getNextStep(self):
        """
        Waits until there is a step to perform and returns the path finder to perform it on.  
        May throw a TerminatedException instead if ROS is shutting down
        """
        with self._controlLock:
            self.checkTerminated()
            while self._steps == 0:
                self._controlLock.wait()
                self.checkTerminated()

            return self._activePathFinder

    def _performStep(self, pathFinder):
        """
        Runs a step on the given path finder instance.  If when complete this is still the active path finder, then the result is published and
        the step count is decremented.  If not, then nothing is published and this returns without decrementing the step count.
        """
        # Calculate a step for the given path finder.  This takes non-zero time and is therefore not syncrhonized.
        solutionFound = pathFinder.step()
        
        with self._controlLock:
            # If ROS was shutdown while not locked, then throw exception and exit
            self.checkTerminated()
            # If the active path finder has changed, then do not publish.
            if pathFinder is not self._activePathFinder:
                return
            
            # Publish and decrement number of steps
            self._publishStepResult(pathFinder, solutionFound)
            self._steps -= 1

    def _publishStepResult(self, pathFinder, solutionFound):
        if solutionFound:
            pathSolution = PathSolution()
            pathSolution.solutionPathSegments = messageUtils.pathSegmentListToMsg(pathFinder.getSolution())
            pathSolution.finished = pathFinder.isDone()
            self._pathSolutionPub.publish(pathSolution)
        else:
            debugData = pathFinder.getDebugData()
            pathDebug = messageUtils.pathDebugToMsg(*debugData)
            self._pathDebugPub.publish(pathDebug)    
        
    def _initiateFindPathRequest(self, request):
        with self._controlLock:
            scenario = messageUtils.msgToScenario(request.scenario)
            vehicle = messageUtils.msgToVehicle(request.vehicle)
            self._activePathFinder = PathFinder(scenario, vehicle)
            self._steps = 0
            return InitiateFindPathResponse()

    def _stepFindPathRequest(self, request):
        with self._controlLock:
            if self._activePathFinder is None or self._activePathFinder.isDone():
                return EmptyResponse()
            self._steps += 1
            self._controlLock.notifyAll()
        return EmptyResponse()

