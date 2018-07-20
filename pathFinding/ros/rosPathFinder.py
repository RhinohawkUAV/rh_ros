import pathfinding.msg as pfm
import pathfinding.srv as pfs
import rospy

from engine.pathFinderManager import PathFinderManager
from messageConverter import MessageConverter
from ros.rosConstants import PATHFINDER_NODE_ID, SUBMIT_PROBLEM_SERVICE, \
    STEP_PROBLEM_SERVICE, PATHFINDER_DEBUG_TOPIC, ROS_QUEUE_SIZE, \
    PATHFINDER_SOLUTION_TOPIC, SOLVE_PROBLEM_SERVICE, PATHFINDER_INPUT_TOPIC


class ShutdownException(BaseException):
    pass


class RosPathFinder(PathFinderManager):

    def __init__(self):
        PathFinderManager.__init__(self)
        rospy.init_node(PATHFINDER_NODE_ID, anonymous=True)
        rospy.core.add_shutdown_hook(self._rosShutdown)
        rospy.Service(SUBMIT_PROBLEM_SERVICE, pfs.SubmitProblem, self._submitProblem)
        rospy.Service(STEP_PROBLEM_SERVICE, pfs.StepProblem, self._stepProblem)
        rospy.Service(SOLVE_PROBLEM_SERVICE, pfs.SolveProblem, self._solveProblem)
        self._pathInputPub = rospy.Publisher(PATHFINDER_INPUT_TOPIC, pfm.Scenario, queue_size=ROS_QUEUE_SIZE)
        self._pathDebugPub = rospy.Publisher(PATHFINDER_DEBUG_TOPIC, pfm.PathDebug, queue_size=ROS_QUEUE_SIZE)
        self._pathSolutionPub = rospy.Publisher(PATHFINDER_SOLUTION_TOPIC, pfm.PathSolution, queue_size=ROS_QUEUE_SIZE)

    def publishSolution(self, solutionWaypoints, solutionPathSegments, finished, referenceGPS):
        messageConverter = MessageConverter(referenceGPS)
        pathSolution = pfm.PathSolution()
        pathSolution.solutionWaypoints = messageConverter.solutionWaypointListToMsg(solutionWaypoints)
        pathSolution.solutionPathSegments = messageConverter.pathSegmentListToMsg(solutionPathSegments)
        pathSolution.finished = finished
        self._pathSolutionPub.publish(pathSolution)
        
    def publishDebug(self, pastPathSegments, futurePathSegments, filteredPathSegments, referenceGPS):
        messageConverter = MessageConverter(referenceGPS)
        pathDebug = messageConverter.pathDebugToMsg(pastPathSegments, futurePathSegments, filteredPathSegments)
        self._pathDebugPub.publish(pathDebug)    

    def publishInput(self, scenarioMsg):
        """
        Whenever the ROS path finder gets an input problem it publishes it on this topic.  This may be removed long term
        as the submitter of the problem may choose to directly send it to whoever needs it.
        """
        self._pathInputPub.publish(scenarioMsg)
        
    def _rosShutdown(self, shutdownMessage):
        rospy.loginfo("ROS is shutting down pathfinder cause: " + shutdownMessage)
        self.shutdown()

    def _submitProblem(self, request):
        self.publishInput(request.scenario)
        messageConverter = MessageConverter(request.referenceGPS)
        params = messageConverter.msgToParams(request.inputParams)
        scenario = messageConverter.msgToScenario(request.scenario)
        vehicle = messageConverter.msgToVehicle(request.vehicle)
        self.submitProblem(params, scenario, vehicle, request.referenceGPS)
        return pfs.SubmitProblemResponse()

    def _stepProblem(self, request):
        self.stepProblem(request.numSteps)
        return pfs.StepProblemResponse()
    
    def _solveProblem(self, request):
        return pfs.SolveProblemResponse()
