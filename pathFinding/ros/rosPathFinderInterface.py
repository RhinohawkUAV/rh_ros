from pathfinding.srv._InitiateFindPath import InitiateFindPath
from ros import messageUtils
from ros import messageUtils
import rospy
from std_srvs.srv._Empty import Empty

from gui.pathFinder.pathFinderInterface import PathFinderInterface
from ros.rosConstants import INITIATE_FINDPATH_SERVICE, STEP_FINDPATH_SERVICE


class RosPathFinderInterface(PathFinderInterface):
    """
    A path finder interface, for the GUI, built around the ROS pathFinder node.
    """

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
