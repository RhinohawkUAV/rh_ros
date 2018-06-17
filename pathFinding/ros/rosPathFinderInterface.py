from pathfinding.msg._GPSCoord import GPSCoord
from pathfinding.msg._PathDebug import PathDebug
from pathfinding.msg._PathSolution import PathSolution
from pathfinding.srv._StepProblem import StepProblem
from pathfinding.srv._SubmitProblem import SubmitProblem
import rospy

import constants
from gui.pathFinder.pathFinderInterface import PathFinderInterface
from messageConverter import MessageConverter
from ros.rosConstants import SUBMIT_PROBLEM_SERVICE, STEP_PROBLEM_SERVICE, \
    PATHFINDER_DEBUG_TOPIC, PATHFINDER_SOLUTION_TOPIC


class RosPathFinderInterface(PathFinderInterface):
    """
    Manages a remote ROS path finder for use in PathFindViewer.
    """

    def __init__(self):
        self.messageConverter = MessageConverter(gpsRef=GPSCoord(constants.CANBERRA_GPS[0], constants.CANBERRA_GPS[1]))
        rospy.Subscriber(PATHFINDER_DEBUG_TOPIC, PathDebug, self.receiveDebug)
        rospy.Subscriber(PATHFINDER_SOLUTION_TOPIC, PathSolution, self.receiveSolution)
    
    def submitProblem(self, scenario, vehicle):
        try:
            scenarioMsg = self.messageConverter.scenarioToMsg(scenario)
            vehicleMsg = self.messageConverter.vehicleToMsg(vehicle)
            # TODO: Do we need to do this every time?  Not in GUI thread.  Need to handle below as well.
            rospy.wait_for_service(SUBMIT_PROBLEM_SERVICE)
            func = rospy.ServiceProxy(SUBMIT_PROBLEM_SERVICE, SubmitProblem)
            func(scenarioMsg, vehicleMsg)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e    
    
    def stepProblem(self, numSteps=1):
        try:
            rospy.wait_for_service(STEP_PROBLEM_SERVICE)
            func = rospy.ServiceProxy(STEP_PROBLEM_SERVICE, StepProblem)
            func(numSteps)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e    

    def solveProblem(self, timeout):
        pass

    def receiveDebug(self, pathDebug):
        (pastPathSegments, futurePathSegments, filteredPathSegments) = self.messageConverter.msgToPathDebug(pathDebug)
        self._listener.fireDebugInGuiThread(pastPathSegments, futurePathSegments, filteredPathSegments)
        
    def receiveSolution(self, pathSolution):
        solutionPathSegments = self.messageConverter.msgToPathSegmentList(pathSolution.solutionPathSegments)
        self._listener.fireSolutionInGuiThread(solutionPathSegments, pathSolution.finished)
        
