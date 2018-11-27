import actionlib
import rospy
from threading import Thread

from gui.pathFinder.pathFinderInterface import PathFinderInterface
from messageConverter import MessageConverter
import rh_pathfinding.msg as pfm
import rh_pathfinding.srv as pfs
from ros.rosConstants import SUBMIT_PROBLEM_SERVICE, STEP_PROBLEM_SERVICE, \
    PATHFINDER_DEBUG_TOPIC, PATHFINDER_INPUT_TOPIC


class RosPathFinderInterface(PathFinderInterface):
    """
    Manages a remote ROS path finder for use in PathFindViewer.
    """

    def __init__(self, outgoingGPSConversion):
        PathFinderInterface.__init__(self)
        self._outgoingGPSConversion = outgoingGPSConversion
        rospy.Subscriber(PATHFINDER_INPUT_TOPIC, pfm.PathInput, self._rosInputAccepted)
        rospy.Subscriber(PATHFINDER_DEBUG_TOPIC, pfm.PathDebug, self._rosStepPerformed)
        
        # "/rh/"+
        self._solveAction = actionlib.SimpleActionClient("pathfinder/action", pfm.SolvePathProblemAction)
    
    def submitProblem(self, params, scenario, vehicle):
        try:
            outgoingMessageConverter = MessageConverter(self._outgoingGPSConversion)
            paramsMsg = outgoingMessageConverter.paramsToMsg(params)
            scenarioMsg = outgoingMessageConverter.scenarioToMsg(scenario)
            vehicleMsg = outgoingMessageConverter.vehicleToMsg(vehicle)
            # TODO: Do we need to do this every time?  Not in GUI thread.  Need to handle below as well.
            rospy.wait_for_service(SUBMIT_PROBLEM_SERVICE)
            func = rospy.ServiceProxy(SUBMIT_PROBLEM_SERVICE, pfs.SubmitPathProblem)
            func(paramsMsg, scenarioMsg, vehicleMsg, self._outgoingGPSConversion)
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
        outgoingMessageConverter = MessageConverter(self._outgoingGPSConversion)
        paramsMsg = outgoingMessageConverter.paramsToMsg(params)
        scenarioMsg = outgoingMessageConverter.scenarioToMsg(scenario)
        vehicleMsg = outgoingMessageConverter.vehicleToMsg(vehicle)
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

        self._messageConverter = MessageConverter(pathInputMsg.scenario.startPoint)        
        (params, scenario, vehicle) = self._messageConverter.msgToInput(pathInputMsg)
        self._fireInputAccepted(params, scenario, vehicle)
        
    def _rosStepPerformed(self, pathDebugMsg):
        (finished, bestPath, pastPathSegments, futurePathSegments, filteredPathSegments) = self._messageConverter.msgToPathDebug(pathDebugMsg)
        self._fireStepPerformed(finished, bestPath, pastPathSegments, futurePathSegments, filteredPathSegments)
