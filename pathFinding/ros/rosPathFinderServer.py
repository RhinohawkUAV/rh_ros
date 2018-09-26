import actionlib
import rospy

from engine.interface import fileUtils
from engine.pathFinder import PathFinder
from messageConverter import MessageConverter
import pathfinding.msg as pfm
from ros.rosConstants import PATHFINDER_INPUT_TOPIC, PATHFINDER_DEBUG_TOPIC, PATHFINDER_SERVER, ROS_QUEUE_SIZE, PATHFINDER_NODE_ID


class RosPathFinderServer():

    def __init__(self):
        rospy.init_node(PATHFINDER_NODE_ID)
        self._activePathFinder = None
        # TODO: Should now publish the "PathInput" data type for this topic.
        self._pathInputPub = rospy.Publisher(PATHFINDER_INPUT_TOPIC, pfm.Scenario, queue_size=ROS_QUEUE_SIZE)
        self._pathDebugPub = rospy.Publisher(PATHFINDER_DEBUG_TOPIC, pfm.PathDebug, queue_size=ROS_QUEUE_SIZE)
        self.server = actionlib.SimpleActionServer(PATHFINDER_SERVER, pfm.PathFinderAction, self.execute, False)
        self.server.start()
 
    def execute(self, goal):
        self.publishInput(goal.scenario)
        self._referenceGPS = goal.scenario.startPoint
        
        messageConverter = MessageConverter(self._referenceGPS)
        params = messageConverter.msgToParams(goal.params)
        
        scenario = messageConverter.msgToScenario(goal.scenario)
        vehicle = messageConverter.msgToVehicle(goal.vehicle)
# TODO: Add proper save util
#         fileUtils.save("/home/shp/catkin_ws/src/pathfinding/scenarios/konrad.json", scenario)
        
        self._activePathFinder = PathFinder(params, scenario, vehicle)

        self.solveProblem()
            
    def solveProblem(self):
        pathFinder = self._activePathFinder
        i = 0
        (solutionWaypoints, pathSolution) = (None, None)
        
        while True:
            i += 1
            rospy.logdebug("Performing Path Step: %d" % (i))
            pathFinder.step()
            if pathFinder.isDone():
                if pathFinder.hasSolution():
                    rospy.logdebug("Finished Path Find (step %d)" % (i))
                    (solutionWaypoints, pathSolution) = pathFinder.getSolution()
                    solution = self.getSolution(solutionWaypoints, pathSolution, True)
                    self.server.set_succeeded(pfm.PathFinderResult(solution))
                else:
                    rospy.logdebug("Aborted Path Find (step %d)" % (i))
                    # TODO: Decide on appropriate failure signal/result
                    self.server.set_aborted(None, "Could not find path")                    
                return 
            else:
                if pathFinder.solutionUpdated():
                    (solutionWaypoints, pathSolution) = pathFinder.getSolution()
                    solution = self.getSolution(solutionWaypoints, pathSolution, False)
                    self.server.publish_feedback(pfm.PathFinderFeedback(solution))
                    
                (previousPathSegments, pathSegments, filteredPathSegments) = pathFinder.getDebugData()
                self.publishDebug(previousPathSegments, pathSegments, filteredPathSegments)                

    def getSolution(self, solutionWaypoints, solutionPathSegments, finished):
        messageConverter = MessageConverter(self._referenceGPS)
        pathSolution = pfm.PathSolution()
        if solutionWaypoints:
            pathSolution.solutionWaypoints = messageConverter.solutionWaypointListToMsg(solutionWaypoints)
        if solutionPathSegments:
            pathSolution.solutionPathSegments = messageConverter.pathSegmentListToMsg(solutionPathSegments)
        pathSolution.finished = finished
        return pathSolution

    def publishDebug(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        messageConverter = MessageConverter(self._referenceGPS)
        pathDebug = messageConverter.pathDebugToMsg(pastPathSegments, futurePathSegments, filteredPathSegments)
        self._pathDebugPub.publish(pathDebug)    

    def publishInput(self, scenarioMsg):
        """
        Whenever the ROS path finder gets an input problem it publishes it on this topic.  This may be removed long term
        as the submitter of the problem may choose to directly send it to whoever needs it.
        """
        self._pathInputPub.publish(scenarioMsg)
        
