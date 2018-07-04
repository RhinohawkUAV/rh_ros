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
from mavros_msgs.msg import WaypointReached
from rh_autonomy.aggregator import LatchMap

values = LatchMap()


def waypoint_reached(state):
    rospy.loginfo("We have reached waypoint %s!" % state.wp_seq)


def get_proxy(topic, serviceType):
    rospy.loginfo("Waiting for service: %s", topic)
    rospy.wait_for_service(topic)
    return rospy.ServiceProxy(topic, serviceType)


def start():

    rospy.init_node("state")

    gps_topic = "/mavros/global_position/global"
    rospy.Subscriber(gps_topic, NavSatFix, partial(values.latch_value, gps_topic, max_age=10))
    rospy.Subscriber("/mavros/mission/reached", WaypointReached, waypoint_reached)

    rospy.loginfo("Mission state ready...")
    rospy.spin()


if __name__ == "__main__":
    start()
