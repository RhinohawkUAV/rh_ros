import pathfinding.msg as pfm
import rospy
import actionlib

from engine.pathFinder import PathFinder
from messageConverter import MessageConverter
from ros.rosConstants import PATHFINDER_INPUT_TOPIC, PATHFINDER_DEBUG_TOPIC, PATHFINDER_SERVER, ROS_QUEUE_SIZE


class RosPathFinderServer():

    def __init__(self):
        rospy.init_node(PATHFINDER_SERVER,)
        self._activePathFinder = None
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
        self._activePathFinder = PathFinder(params, scenario, vehicle)

        (solutionWaypoints, pathSolution) = self.solveProblem()
        solution = self.getSolution(solutionWaypoints, pathSolution, True)
        self.server.set_succeeded(pfm.PathFinderResult(solution))
        

    def solveProblem(self):
        pathFinder = self._activePathFinder
        i = 0
        (solutionWaypoints, pathSolution) = (None, None)
        while not pathFinder.isDone():

            i += 1
            solutionFound = pathFinder.step()
            rospy.logdebug("Step %d, solution found = %s" % (i, solutionFound))
            # Publish and decrement number of steps
            if solutionFound:
                (solutionWaypoints, pathSolution) = pathFinder.getSolution()
                solution = self.getSolution(solutionWaypoints, pathSolution, False)
                self.server.publish_feedback(pfm.PathFinderFeedback(solution))
            else:
                (previousPathSegments, pathSegments, filteredPathSegments) = pathFinder.getDebugData()
                self.publishDebug(previousPathSegments, pathSegments, filteredPathSegments)

        return (solutionWaypoints, pathSolution)


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
        
