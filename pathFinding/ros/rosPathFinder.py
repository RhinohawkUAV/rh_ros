from pathfinding.msg._PathDebug import PathDebug
from pathfinding.msg._PathSolution import PathSolution
from pathfinding.srv._SolveProblem import SolveProblem, SolveProblemResponse
from pathfinding.srv._StepProblem import StepProblem, StepProblemResponse
from pathfinding.srv._SubmitProblem import SubmitProblem, SubmitProblemResponse
from roscpp.srv._Empty import EmptyResponse
import rospy
from std_msgs.msg import Int64
from engine.pathFinderManager import PathFinderManager
import messageUtils
from ros.rosConstants import PATHFINDER_NODE_ID, SUBMIT_PROBLEM_SERVICE, \
    STEP_PROBLEM_SERVICE, PATHFINDER_DEBUG_TOPIC, ROS_QUEUE_SIZE, \
    PATHFINDER_SOLUTION_TOPIC, SOLVE_PROBLEM_SERVICE


class ShutdownException(BaseException):
    pass


class RosPathFinder(PathFinderManager):

    def __init__(self):
        PathFinderManager.__init__(self)
        rospy.init_node(PATHFINDER_NODE_ID, anonymous=True)
        rospy.core.add_shutdown_hook(self._rosShutdown)
        rospy.Service(SUBMIT_PROBLEM_SERVICE, SubmitProblem, self._submitProblem)
        rospy.Service(STEP_PROBLEM_SERVICE, StepProblem, self._stepProblem)
        rospy.Service(SOLVE_PROBLEM_SERVICE, SolveProblem, self._solveProblem)
        self._pathDebugPub = rospy.Publisher(PATHFINDER_DEBUG_TOPIC, PathDebug, queue_size=ROS_QUEUE_SIZE)
        self._pathSolutionPub = rospy.Publisher(PATHFINDER_SOLUTION_TOPIC, PathSolution, queue_size=ROS_QUEUE_SIZE)
        
    def publishSolution(self, solutionPathSegments, finished):
        """
        Override me.
        Called whenever a step on the active path finder concludes with a solutionPathSegments.
        This is called from within the path finder thread and should execute quickly.
        """
        pathSolution = PathSolution()
        pathSolution.solutionPathSegments = messageUtils.pathSegmentListToMsg(solutionPathSegments)
        pathSolution.finished = finished
        self._pathSolutionPub.publish(pathSolution)        
        
    def publishDebug(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        """
        Override me.
        Called whenever a step on the active path finder concludes with debug data.
        This is called from within the path finder thread and should execute quickly.
        """
        pathDebug = messageUtils.pathDebugToMsg(pastPathSegments, futurePathSegments, filteredPathSegments)
        self._pathDebugPub.publish(pathDebug)    

    def _rosShutdown(self, shutdownMessage):
        print "ROS is shutting down pathfinder cause: " + shutdownMessage
        self.shutdown()

    def _submitProblem(self, request):
        scenario = messageUtils.msgToScenario(request.scenario)
        vehicle = messageUtils.msgToVehicle(request.vehicle)
        self.submitProblem(scenario, vehicle)
        return SubmitProblemResponse()

    def _stepProblem(self, request):
        self.stepProblem(request.numSteps)
        return StepProblemResponse()
    
    def _solveProblem(self, request):
        return SolveProblemResponse()
