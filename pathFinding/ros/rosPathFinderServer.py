import rospy
from engine.pathFinderManager import PathFinderManager
import pathfinding.msg as pfm
import pathfinding.srv as pfs
from ros.messageConverter import MessageConverter
from ros.rosConstants import PATHFINDER_NODE_ID, SUBMIT_PROBLEM_SERVICE, \
    STEP_PROBLEM_SERVICE, PATHFINDER_DEBUG_TOPIC, ROS_QUEUE_SIZE, \
    PATHFINDER_INPUT_TOPIC


class RosPathFinderServer:

    def __init__(self):
        rospy.init_node(PATHFINDER_NODE_ID, anonymous=True)
        rospy.core.add_shutdown_hook(self._rosShutdown)
        rospy.Service(SUBMIT_PROBLEM_SERVICE, pfs.SubmitProblem, self._rosInputReceived)
        rospy.Service(STEP_PROBLEM_SERVICE, pfs.StepProblem, self._rosStepRequested)
        self._inputAcceptedPub = rospy.Publisher(PATHFINDER_INPUT_TOPIC, pfm.PathInput, queue_size=ROS_QUEUE_SIZE)
        self._stepPerfomredPub = rospy.Publisher(PATHFINDER_DEBUG_TOPIC, pfm.PathDebug, queue_size=ROS_QUEUE_SIZE)
        self._pathFinderManager = PathFinderManager()
        self._pathFinderManager.setListeners(self._publishInputAccepted, self._publishStepPerformed)

    def _publishInputAccepted(self, params, scenario, vehicle, refGPS=None):
        messageConverter = MessageConverter(refGPS)
        inputMsg = messageConverter.inputToMsg(params, scenario, vehicle)
        self._inputAcceptedPub.publish(inputMsg)

    def _publishStepPerformed(self, isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments, refGPS=None):
        messageConverter = MessageConverter(refGPS)
        pathDebug = messageConverter.pathDebugToMsg(isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments)
        self._stepPerfomredPub.publish(pathDebug)

    def _rosInputReceived(self, inputAcceptedMsg):
        refGPS = inputAcceptedMsg.scenario.startPoint
        messageConverter = MessageConverter(refGPS)
        params = messageConverter.msgToParams(inputAcceptedMsg.inputParams)
        scenario = messageConverter.msgToScenario(inputAcceptedMsg.scenario)
        vehicle = messageConverter.msgToVehicle(inputAcceptedMsg.vehicle)
        self._pathFinderManager.submitProblem(params, scenario, vehicle, refGPS=refGPS)
        return pfs.SubmitProblemResponse()

    def _rosStepRequested(self, stepRequestedMsg):
        self._pathFinderManager.stepProblem(stepRequestedMsg.numSteps)
        return pfs.StepProblemResponse()

    def _rosShutdown(self, shutdownMessage):
        rospy.loginfo("ROS is shutting down pathfinder, because: " + shutdownMessage)
        self.shutdown()
