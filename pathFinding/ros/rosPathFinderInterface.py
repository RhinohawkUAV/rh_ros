import actionlib
import rospy
from threading import Thread

from gui.pathFinder.pathFinderInterface import PathFinderInterface
from messageConverter import MessageConverter
import pathfinding.msg as pfm
import pathfinding.srv as pfs
from ros.rosConstants import SUBMIT_PROBLEM_SERVICE, STEP_PROBLEM_SERVICE, \
    PATHFINDER_DEBUG_TOPIC, PATHFINDER_INPUT_TOPIC


class RosPathFinderInterface(PathFinderInterface):
    """
    Manages a remote ROS path finder for use in PathFindViewer.
    """

    def __init__(self, gpsReference):
        self._gpsReference = gpsReference
        self._messageConverter = MessageConverter(self._gpsReference)
        # TODO: Figure out how to interface with other namespace
#         rospy.Subscriber("rh/" + PATHFINDER_INPUT_TOPIC, PathDebug, self.receiveDebug)
#         rospy.Subscriber("rh/" + PATHFINDER_DEBUG_TOPIC, PathSolution, self.receiveSolution)
        rospy.Subscriber(PATHFINDER_INPUT_TOPIC, pfm.PathInput, self._rosInputAccepted)
        rospy.Subscriber(PATHFINDER_DEBUG_TOPIC, pfm.PathDebug, self._rosStepPerformed)
        
        # "/rh/"+
        self._solveAction = actionlib.SimpleActionClient("pathfinder/action", pfm.SolvePathProblemAction)
    
    def submitProblem(self, params, scenario, vehicle):
        try:
            paramsMsg = self._messageConverter.paramsToMsg(params)
            scenarioMsg = self._messageConverter.scenarioToMsg(scenario)
            vehicleMsg = self._messageConverter.vehicleToMsg(vehicle)
            # TODO: Do we need to do this every time?  Not in GUI thread.  Need to handle below as well.
            rospy.wait_for_service(SUBMIT_PROBLEM_SERVICE)
            func = rospy.ServiceProxy(SUBMIT_PROBLEM_SERVICE, pfs.SubmitPathProblem)
            func(paramsMsg, scenarioMsg, vehicleMsg, self._gpsReference)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            
    def stepProblem(self, numSteps=1):
        try:
            rospy.wait_for_service(STEP_PROBLEM_SERVICE)
            func = rospy.ServiceProxy(STEP_PROBLEM_SERVICE, pfs.StepPathProblem)
            func(numSteps)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e    
        
    def solveProblem(self, params, scenario, vehicle, timeout):
        paramsMsg = self._messageConverter.paramsToMsg(params)
        scenarioMsg = self._messageConverter.scenarioToMsg(scenario)
        vehicleMsg = self._messageConverter.vehicleToMsg(vehicle)
        self._solveAction.wait_for_server()
        goal = pfm.SolvePathProblemGoal()
        goal.params = paramsMsg
        goal.scenario = scenarioMsg
        goal.vehicle = vehicleMsg
        goal.timeout = timeout
        goal.emitDebug = True
        self._solveAction.send_goal(goal)
        # TODO: Needs to determine an appropriate extra time buffer
        EXTRA_TIME = 1.0
        Thread(target=self._rosProblemSolved, args=[timeout + EXTRA_TIME]).start()
        
    def _rosProblemSolved(self, timeout):
        
        # TODO: Should not be able to fail, but we should probably do something if it does.
        success = self._solveAction.wait_for_result(rospy.Duration.from_sec(timeout))
        if success:
            bestPathMsg = self._solveAction.get_result().bestPath
            bestPath = self._messageConverter.msgToOutputPath(bestPathMsg)
            self._fireSolved(bestPath)
        else:
            print "Solve Failed!"

    def _rosInputAccepted(self, pathInputMsg):

        (params, scenario, vehicle) = self._messageConverter.msgToInput(pathInputMsg)
        self._fireInputAccepted(params, scenario, vehicle)
        
    def _rosStepPerformed(self, pathDebugMsg):

        (finished, bestPath, pastPathSegments, futurePathSegments, filteredPathSegments) = self._messageConverter.msgToPathDebug(pathDebugMsg)
        self._fireStepPerformed(finished, bestPath, pastPathSegments, futurePathSegments, filteredPathSegments)
