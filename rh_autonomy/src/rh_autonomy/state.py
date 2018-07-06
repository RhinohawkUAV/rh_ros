#!/usr/bin/env python
"""
ROS node implementing Rhinohawk global mission state, including:

- mission definition
    - geofence  (polygon)
    - static no fly zones
    - roads
    - base location
    - transit waypoints
    - joe's location
    
- dynamic no fly zones

- current GPS location

- current mission phase 
    BASE
    TAKE_OFF
    OUTBOUND
    SEARCH
    REMOTE_LAND
    REMOTE_WAIT
    REMOTE_TAKE_OFF
    INBOUND
    LAND

- navigation waypoints (with mapping to mission phases, and transit waypoints)

- purported marker locations

- chosen landing marker location

"""

from functools import partial

import rospy

from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import WaypointList, WaypointReached, StatusText

from rh_msgs.msg import State
from rh_msgs.srv import GetState, GetStateResponse
from rh_msgs.srv import SetMission, SetMissionResponse
from rh_msgs.srv import SetNoFlyZones, SetNoFlyZonesResponse

from rh_autonomy.aggregator import LatchMap


gps_topic = "/mavros/global_position/global"

def get_proxy(topic, serviceType):
    rospy.loginfo("Waiting for service: %s", topic)
    rospy.wait_for_service(topic)
    return rospy.ServiceProxy(topic, serviceType)


def log(s):
    rospy.loginfo("STATE: %s" % s)


def warn(s):
    rospy.logwarn("STATE: %s" % s)


class StateNode():

    def __init__(self):
        self.values = LatchMap()
        self.mission = None
        self.reached_wp_index = -1

        rospy.init_node("state")

        rospy.Subscriber(gps_topic, NavSatFix, partial(self.values.latch_value, gps_topic, max_age=10))
        rospy.Subscriber("/mavros/mission/reached", WaypointReached, self.waypoint_reached)
        rospy.Subscriber("/mavros/mission/waypoints", WaypointList, self.waypoints_changed)

        rospy.Subscriber("/mavros/statustext/recv", StatusText, self.mavlink_statustext)

        rospy.Service('command/set_mission', SetMission, self.handle_set_mission)
        rospy.Service('command/set_dnfzs', SetNoFlyZones, self.handle_set_dnfzs)
        rospy.Service('command/get_state', GetState, self.handle_get_state)

       
    def waypoint_reached(self, msg):

        if self.reached_wp_index < msg.wp_seq:
            log("We have reached waypoint %s" % msg.wp_seq)
            self.reached_wp_index = msg.wp_seq

            if not self.apm_waypoints:
                warn("Waypoint reached, but no APM waypoints")
                return

            try:
                waypoint = self.apm_waypoints[msg.wp_seq]

            except:
                warn("Waypoint reached, but not in APM waypoints")


    def waypoints_changed(self, msg):
        self.apm_waypoints = msg.waypoints
        log("Waypoints list has changed (%d waypoints)" % len(self.apm_waypoints))


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


    def handle_set_dnfzs(self, msg):
        self.dynamic_nfzs = msg.dynamic_nfzs
        log("New dynamic no fly zones have been set")
        return SetNoFlyZonesResponse(True)

    
    def handle_get_state(self, msg):

        state = State()
        state.mission = self.mission
        state.gps_position = self.values.get_value(gps_topic)

        return GetStateResponse(state)


if __name__ == "__main__":
    node = StateNode()
    rospy.loginfo("Mission state ready...")
    rospy.spin()

