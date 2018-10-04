
from __builtin__ import True
import actionlib
import errno
import os
import rospy
from threading import Lock, Condition
import time

from engine.interface import fileUtils
from engine.pathFinder import PathFinder
from engine.pathFinderManager import PathFinderManager
import pathfinding.msg as pfm
import pathfinding.srv as pfs
from ros.messageConverter import MessageConverter
from ros.rosConstants import PATHFINDER_NODE_ID, SUBMIT_PROBLEM_SERVICE, \
    STEP_PROBLEM_SERVICE, PATHFINDER_DEBUG_TOPIC, ROS_QUEUE_SIZE, \
    PATHFINDER_INPUT_TOPIC, PATHFINDER_SERVER

# Keep dumps of the last 20 solved problems
MAX_FILE_DUMPS = 1000


class RosPathFinderServer:

    def __init__(self):
        self._lock = Condition(Lock())
        
        # While true, we are in the process performing the solve action. All messages are ignored (no one should be sending any) 
        self.solving = False
        self.solveID = 0
        self._pathFinderManager = PathFinderManager()
        rospy.init_node(PATHFINDER_NODE_ID, anonymous=True)
        rospy.core.add_shutdown_hook(self._rosShutdown) 
        rospy.Service(SUBMIT_PROBLEM_SERVICE, pfs.SubmitPathProblem, self._rosInputReceived)
        rospy.Service(STEP_PROBLEM_SERVICE, pfs.StepPathProblem, self._rosStepRequested)
        self._inputAcceptedPub = rospy.Publisher(PATHFINDER_INPUT_TOPIC, pfm.PathInput, queue_size=ROS_QUEUE_SIZE)
        self._stepPerfomredPub = rospy.Publisher(PATHFINDER_DEBUG_TOPIC, pfm.PathDebug, queue_size=ROS_QUEUE_SIZE)
        self._pathFinderManager.setListeners(self._publishInputAccepted, self._publishStepPerformed)
        self.server = actionlib.SimpleActionServer(PATHFINDER_SERVER, pfm.SolvePathProblemAction, self._rosSolveRequested, False)
        self.server.start()

#******************************Serivce/action calls to server*******************************
    def _rosInputReceived(self, inputMsg):
        with self._lock:
            if self.solving:
                rospy.logerr("Cannot start a debug operation while a solve action is taking place!")
                return pfs.SubmitPathProblemResponse()
            refGPS = inputMsg.scenario.startPoint
            messageConverter = MessageConverter(refGPS)
            params = messageConverter.msgToParams(inputMsg.params)
            scenario = messageConverter.msgToScenario(inputMsg.scenario)
            vehicle = messageConverter.msgToVehicle(inputMsg.vehicle)
            self._pathFinderManager.submitProblem(params, scenario, vehicle, refGPS=refGPS)
        return pfs.SubmitPathProblemResponse()

    def _rosStepRequested(self, stepRequestedMsg):
        with self._lock:
            if self.solving:
                rospy.logerr("Cannot step a debug operation while a solve action is taking place!")
                return pfs.StepPathProblemResponse()
        self._pathFinderManager.stepProblem(stepRequestedMsg.numSteps)
        return pfs.StepPathProblemResponse()

    def _rosSolveRequested(self, inputMsg):
        startTime = time.time()
        with self._lock:
            # TODO: Not clear what happen when multiple solve actions are started...just checking for solving==True does not work.
            if self._pathFinderManager.getStepsRemaining() > 0:
                rospy.logerr("Cannot start a solve operation while other operations are in progress!")
                self.server.set_aborted(None, "Cannot start a solve operation while other operations are in progress!")   
                return
            
            self.solving = True
        
        refGPS = inputMsg.scenario.startPoint
        messageConverter = MessageConverter(refGPS)
        params = messageConverter.msgToParams(inputMsg.params)
        scenario = messageConverter.msgToScenario(inputMsg.scenario)
        vehicle = messageConverter.msgToVehicle(inputMsg.vehicle)
        
        # If debugging, then emit problem to topic and dump file to disk
        if inputMsg.emitDebug:
            self._publishInputAccepted(params, scenario, vehicle, refGPS)
            
            self.solveID += 1
            solveID = self.solveID % MAX_FILE_DUMPS
            dumpDirectory = os.path.join(os.path.expanduser("~"), "pathDumps")
            if not os.path.exists(dumpDirectory):
                try:
                    os.makedirs(dumpDirectory)
                except OSError as exc:  # Guard against race condition
                    if exc.errno != errno.EEXIST:
                        raise
            path = os.path.join(dumpDirectory, "dump" + str(solveID) + ".json")
            rospy.loginfo("Solving problem ID = " + str(solveID) + ".  Problem stored in: " + path)        
            fileUtils.save(path, params, scenario, vehicle)     
        
        pathFinder = PathFinder(params, scenario, vehicle)
        totalTime = 0.0
        numSteps = 0
        
        # Timeout occurs when total_time + avg_time_per_step > timeout
        while pathFinder.step():
            # Not bothering with intermediate results.
            if inputMsg.emitDebug:
                self._publishPathFinderState(False, pathFinder, messageConverter)
            totalTime = time.time() - startTime
            numSteps += 1
            if (totalTime * (numSteps + 1)) / numSteps > inputMsg.timeout:
                break
            
        if inputMsg.emitDebug:
            self._publishPathFinderState(True, pathFinder, messageConverter)

        bestPath = pathFinder.getBestPath()
        self.server.set_succeeded(pfm.SolvePathProblemResult(messageConverter.outputPathToMsg(bestPath)))

        # Don't know Python's exact memory protocol regarding this kind of situation
        with self._lock:
            self.solving = False

#******************************Methods to Publishon ROS topics*******************************
    def _publishPathFinderState(self, isFinished, pathFinder, messageConverter):
        bestPath = pathFinder.getBestPath()
        (previousPathSegments, futurePathSegments, filteredPathSegments) = pathFinder.getDebugData()
        pathDebugMsg = messageConverter.pathDebugToMsg(isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments)
        self._stepPerfomredPub.publish(pathDebugMsg)
        
    def _publishInputAccepted(self, params, scenario, vehicle, refGPS=None):
        messageConverter = MessageConverter(refGPS)
        inputMsg = messageConverter.inputToMsg(params, scenario, vehicle)
        self._inputAcceptedPub.publish(inputMsg)

    def _publishStepPerformed(self, isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments, refGPS=None):
        messageConverter = MessageConverter(refGPS)
        pathDebugMsg = messageConverter.pathDebugToMsg(isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments)
        self._stepPerfomredPub.publish(pathDebugMsg)

    def _rosShutdown(self, shutdownMessage):
        rospy.loginfo("ROS is shutting down pathfinder, because: " + shutdownMessage)
        rospy.loginfo("Shutting down path finder manager...")
        self._pathFinderManager.shutdownAndWait()
        rospy.loginfo("Path finder manager is shutdown")
