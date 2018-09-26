import rospy

import constants
from gui.pathFinder.pathFinderInterface import PathFinderInterface
from messageConverter import MessageConverter
import pathfinding.msg as pfm
import pathfinding.srv as pfs
from ros.rosConstants import SUBMIT_PROBLEM_SERVICE, STEP_PROBLEM_SERVICE, \
    PATHFINDER_DEBUG_TOPIC, PATHFINDER_SOLUTION_TOPIC, PATHFINDER_INPUT_TOPIC


class RosPathFinderInterface(PathFinderInterface):
    """
    Manages a remote ROS path finder for use in PathFindViewer.
    """

    def __init__(self):
        self._gpsReference = pfm.GPSCoord(constants.CANBERRA_GPS[0], constants.CANBERRA_GPS[1])
#         rospy.Subscriber("rh/" + PATHFINDER_DEBUG_TOPIC, PathDebug, self.receiveDebug)
#         rospy.Subscriber("rh/" + PATHFINDER_SOLUTION_TOPIC, PathSolution, self.receiveSolution)
        rospy.Subscriber(PATHFINDER_INPUT_TOPIC, pfm.PathInput, self.receiveProblem)
        rospy.Subscriber(PATHFINDER_DEBUG_TOPIC, pfm.PathDebug, self.receiveDebug)
        rospy.Subscriber(PATHFINDER_SOLUTION_TOPIC, pfm.PathSolution, self.receiveSolution)

    def submitProblem(self, params, scenario, vehicle):
        try:
            messageConverter = MessageConverter(self._gpsReference)
            paramsMsg = messageConverter.paramsToMsg(params)
            scenarioMsg = messageConverter.scenarioToMsg(scenario)
            vehicleMsg = messageConverter.vehicleToMsg(vehicle)
            # TODO: Do we need to do this every time?  Not in GUI thread.  Need to handle below as well.
            rospy.wait_for_service(SUBMIT_PROBLEM_SERVICE)
            func = rospy.ServiceProxy(SUBMIT_PROBLEM_SERVICE, pfs.SubmitProblem)
            func(paramsMsg, scenarioMsg, vehicleMsg, self._gpsReference)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e    
    
    def stepProblem(self, numSteps=1):
        try:
            rospy.wait_for_service(STEP_PROBLEM_SERVICE)
            func = rospy.ServiceProxy(STEP_PROBLEM_SERVICE, pfs.StepProblem)
            func(numSteps)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e    

    def solveProblem(self, timeout):
        pass

    def receiveProblem(self, pathInput):
        messageConverter = MessageConverter(self._gpsReference)
        (params, scenario, vehicle) = messageConverter.msgToInput(pathInput)
        self._listener.fireInputInGuiThread(params, scenario, vehicle)
        
    def receiveDebug(self, pathDebug):
        messageConverter = MessageConverter(self._gpsReference)
        (pastPathSegments, futurePathSegments, filteredPathSegments) = messageConverter.msgToPathDebug(pathDebug)
        self._listener.fireDebugInGuiThread(pastPathSegments, futurePathSegments, filteredPathSegments)
        
    def receiveSolution(self, pathSolution):
        messageConverter = MessageConverter(self._gpsReference)
        solutionWaypoints = messageConverter.msgToSolutionWaypointList(pathSolution.solutionWaypoints)
        solutionPathSegments = messageConverter.msgToPathSegmentList(pathSolution.solutionPathSegments)
        self._listener.fireSolutionInGuiThread(solutionWaypoints, solutionPathSegments, pathSolution.finished)
        
