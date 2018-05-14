from pathfinding.msg._PathDebug import PathDebug
from pathfinding.msg._PathSolution import PathSolution
from pathfinding.srv._InitiateFindPath import InitiateFindPath
from ros import messageUtils
import rospy
from std_srvs.srv._Empty import Empty

from gui.pathFinder.pathFinderInterface import PathFinderInterface
from ros.rosConstants import INITIATE_FINDPATH_SERVICE, STEP_FINDPATH_SERVICE, \
    PATHFINDER_DEBUG_TOPIC, PATHFINDER_SOLUTION_TOPIC


class RosPathFinderInterface(PathFinderInterface):
    """
    Manages a remote ROS path finder for use in PathFindViewer.
    """

    def __init__(self):
        rospy.Subscriber(PATHFINDER_DEBUG_TOPIC, PathDebug, self.receiveDebug)
        rospy.Subscriber(PATHFINDER_SOLUTION_TOPIC, PathSolution, self.receiveSolution)
    
    def initiate(self, scenario, vehicle):
        """
        Start a new path finding process.  Will wipe out previous process.
        """
        try:
            scenarioMsg = messageUtils.scenarioToMsg(scenario)
            vehicleMsg = messageUtils.vehicleToMsg(vehicle)
            self.initiateFindPathROS(scenarioMsg, vehicleMsg)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e    
    
    def step(self):
        """
        Perform one step of the path finding process.
        """
        try:
            self.stepFindPathROS()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e    

    def initiateFindPathROS(self, scenarioMsg, vehicleMsg):
        """
        raises: rospy.ServiceException
        """
        rospy.wait_for_service(INITIATE_FINDPATH_SERVICE)
        func = rospy.ServiceProxy(INITIATE_FINDPATH_SERVICE, InitiateFindPath)
        func(scenarioMsg, vehicleMsg)

    def stepFindPathROS(self):
        """
        raises: rospy.ServiceException
        """
        rospy.wait_for_service(STEP_FINDPATH_SERVICE)
        func = rospy.ServiceProxy(STEP_FINDPATH_SERVICE, Empty)
        func()

    def receiveDebug(self, pathDebug):
        (pastPathSegments, futurePathSegments, filteredPathSegments) = messageUtils.msgToPathDebug(pathDebug)
        self._listener.triggerDebug(pastPathSegments, futurePathSegments, filteredPathSegments)
        
    def receiveSolution(self, pathSolution):
        solutionPathSegments = messageUtils.msgToPathSegmentList(pathSolution.solutionPathSegments)
        self._listener.triggerSolution(solutionPathSegments, pathSolution.finished)
        