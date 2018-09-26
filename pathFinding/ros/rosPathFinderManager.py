import rospy

from engine.pathFinderManager import PathFinderManager
from messageConverter import MessageConverter
import pathfinding.msg as pfm
import pathfinding.srv as pfs
from ros.rosConstants import PATHFINDER_NODE_ID, SUBMIT_PROBLEM_SERVICE, \
    STEP_PROBLEM_SERVICE, PATHFINDER_DEBUG_TOPIC, ROS_QUEUE_SIZE, \
    PATHFINDER_SOLUTION_TOPIC, SOLVE_PROBLEM_SERVICE, PATHFINDER_INPUT_TOPIC


class ShutdownException(BaseException):
    pass


class RosPathFinderManager(PathFinderManager):

    def __init__(self):
        PathFinderManager.__init__(self)
        self._referenceGPS = None
        rospy.init_node(PATHFINDER_NODE_ID, anonymous=True)
        rospy.core.add_shutdown_hook(self._rosShutdown)
        rospy.Service(SUBMIT_PROBLEM_SERVICE, pfs.SubmitProblem, self._rosSubmitProblem)
        rospy.Service(STEP_PROBLEM_SERVICE, pfs.StepProblem, self._rosStepProblem)
        rospy.Service(SOLVE_PROBLEM_SERVICE, pfs.SolveProblem, self._rosSolveProblem)
        self._pathInputPub = rospy.Publisher(PATHFINDER_INPUT_TOPIC, pfm.PathInput, queue_size=ROS_QUEUE_SIZE)
        self._pathDebugPub = rospy.Publisher(PATHFINDER_DEBUG_TOPIC, pfm.PathDebug, queue_size=ROS_QUEUE_SIZE)
        self._pathSolutionPub = rospy.Publisher(PATHFINDER_SOLUTION_TOPIC, pfm.PathSolution, queue_size=ROS_QUEUE_SIZE)

    def publishInput(self, params, scenario, vehicle):
        messageConverter = MessageConverter(self._referenceGPS)
        inputMsg = messageConverter.inputToMsg(params, scenario, vehicle)
        self._pathInputPub.publish(inputMsg)

    def publishDebug(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        messageConverter = MessageConverter(self._referenceGPS)
        pathDebug = messageConverter.pathDebugToMsg(pastPathSegments, futurePathSegments, filteredPathSegments)
        self._pathDebugPub.publish(pathDebug)    
        
    def publishSolution(self, solutionWaypoints, solutionPathSegments, finished):
        messageConverter = MessageConverter(self._referenceGPS)
        pathSolution = pfm.PathSolution()
        pathSolution.solutionWaypoints = messageConverter.solutionWaypointListToMsg(solutionWaypoints)
        pathSolution.solutionPathSegments = messageConverter.pathSegmentListToMsg(solutionPathSegments)
        pathSolution.finished = finished
        self._pathSolutionPub.publish(pathSolution)
        
    def _rosShutdown(self, shutdownMessage):
        rospy.loginfo("ROS is shutting down pathfinder cause: " + shutdownMessage)
        self.shutdown()

    def _rosSubmitProblem(self, request):
        messageConverter = MessageConverter(request.referenceGPS)
        params = messageConverter.msgToParams(request.inputParams)
        scenario = messageConverter.msgToScenario(request.scenario)
        vehicle = messageConverter.msgToVehicle(request.vehicle)
        with self._lock:
            self._referenceGPS = request.referenceGPS
            self.submitProblem(params, scenario, vehicle)
            
        return pfs.SubmitProblemResponse()

    def _rosStepProblem(self, request):
        self.stepProblem(request.numSteps)
        return pfs.StepProblemResponse()
    
    def _rosSolveProblem(self, request):
        return pfs.SolveProblemResponse()
