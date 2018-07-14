#!/usr/bin/env python
"""
ROS node implementing Rhinohawk global mission state. See rh_msgs.msg.State for a full description of the state data.

The state node aggregates state from many different sources and makes it available to other nodes in the Rhinohawk System, particularly the controller node which is responsible for autonomous mission control.

Here it's important to make the distinction between two types of waypoints used in the Rhinohawk System:
    Mission waypoints, sometimes called "transit waypoints", are the top-level waypoints which need to be reached in order to satisfy the mission. In the context of the MedExpress Challenge, the mission waypoints break down as follows:
        1,2 - Must be traversed in this order
        3,N - Can be traversed in any other
        N+1 - Joe's reported location, area must be searched for a landing location
        On the way back:
        N,3 - Can be traversed in any order
        2,1 - Must be traversed in this order
    APM waypoints are the low-level waypoints used by the navigation system to navigate no-fly-zones and geofence restrictions and to (eventually) complete mission objectives. Typically, completing any of the above mission waypoints will require M number of APM waypoints. For performance purposes, only the waypoints necessary to complete the next mission objective are uploaded to the autopilot at any given time.
"""

from functools import partial

import rospy

from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import WaypointList, WaypointReached, StatusText

from rh_msgs.msg import State, Mission, GPSCoord
from rh_msgs.srv import GetState, GetStateResponse
from rh_msgs.srv import SetMission, SetMissionResponse
from rh_msgs.srv import StartMission, StartMissionResponse
from rh_msgs.srv import AbortMission, AbortMissionResponse
from rh_msgs.srv import SetNoFlyZones, SetNoFlyZonesResponse
from rh_autonomy.aggregator import LatchMap


class MissionStatus:
    NOT_READY = 1
    READY = 2
    RUNNING = 3
    ABORTING = 4

gps_topic = "/mavros/global_position/global"


def log(s):
    rospy.loginfo("STATE: %s" % s)


def warn(s):
    rospy.logwarn("STATE: %s" % s)


class StateNode():

    def __init__(self):
        self.values = LatchMap()
        self.mission = Mission()
        self.dynamic_nfzs = []
        self.apm_wps = []
        self.target_mission_wp = 0
        self.landing_location = GPSCoord()
        self.reached_wp_index = -1
        self.mission_status = MissionStatus.NOT_READY

        rospy.init_node("state")

        rospy.Subscriber(gps_topic, NavSatFix, partial(self.values.latch_value, gps_topic, max_age=10))
        rospy.Subscriber("/mavros/mission/reached", WaypointReached, self.waypoint_reached)
        rospy.Subscriber("/mavros/mission/waypoints", WaypointList, self.waypoints_changed)
        rospy.Subscriber("/mavros/statustext/recv", StatusText, self.mavlink_statustext)

        rospy.Service('command/set_mission', SetMission, self.handle_set_mission)
        rospy.Service('command/start_mission', StartMission, self.handle_start_mission)
        rospy.Service('command/abort_mission', AbortMission, self.handle_abort_mission)
        rospy.Service('command/set_dnfzs', SetNoFlyZones, self.handle_set_dnfzs)
        rospy.Service('command/get_state', GetState, self.handle_get_state)
       

    def waypoint_reached(self, msg):
        """ Called whenever an APM waypoint is reached
        """

        if self.reached_wp_index < msg.wp_seq:
            log("We have reached waypoint %s" % msg.wp_seq)
            self.reached_wp_index = msg.wp_seq

            if not self.apm_wps:
                warn("Waypoint reached, but no APM waypoints")
                return

            try:
                waypoint = self.apm_wps[msg.wp_seq]

            except:
                warn("Waypoint reached, but not in APM waypoints")


    def waypoints_changed(self, msg):
        self.apm_wps = msg.waypoints
        log("Waypoints list has changed (%d waypoints)" % len(self.apm_wps))


    def mavlink_statustext(self, msg):
        #log("Got Mavlink msg: %s " % msg.text)
        if msg == 'Land complete':
            log("On the ground")


    def handle_set_mission(self, msg):
        """ Set mission parameters
        """
        self.mission = msg.mission
        log("New mission parameters have been set")
        return SetMissionResponse(True)


    def handle_start_mission(self, msg):
        """ Start mission
        """
        self.mission_status = MissionStatus.RUNNING
        return StartMissionResponse(True)


    def handle_abort_mission(self, msg):
        """ Abort mission with extreme prejudice
        """
        self.mission_status = MissionStatus.ABORTING
        return AbortMissionResponse(True)


    def handle_set_dnfzs(self, msg):
        self.dynamic_nfzs = msg.dynamic_nfzs
        log("New dynamic no fly zones have been set")
        return SetNoFlyZonesResponse(True)

    
    def handle_get_state(self, msg):

        state = State()
        state.mission = self.mission
        state.dynamic_nfzs = self.dynamic_nfzs
        state.target_mission_wp = self.target_mission_wp
        gps_position = self.values.get_value(gps_topic)
        if gps_position:
            state.gps_position = gps_position
        else:
            state.gps_position = NavSatFix()
        state.apm_wps = self.apm_wps
        state.landing_location = self.landing_location
        state.mission_status = self.mission_status
        return GetStateResponse(state)


if __name__ == "__main__":
    node = StateNode()
    rospy.loginfo("Mission state ready.")
    rospy.spin()

