#!/usr/bin/env python
"""
ROS node implementing an autonomous mission controller for the MedExpress Challenge

This is the central integration node for the Rhinohawk System. It polls the global state (/rh/state), uses the path planner (/rh/planner) to plan a route, and adjusts course if necessary, using the flight api (/rh/command).
"""

import rospy
from rh_msgs.srv import StartMission, StartMissionResponse
from rh_msgs.srv import AbortMission, AbortMissionResponse
from rh_msgs.srv import GetState
from rh_autonomy.util import get_proxy
from rh_autonomy.state import MissionStatus

# check state and adjust control every second
CONTROL_RATE_HZ = 1 

get_state = None

def name():
    return "controller"

def handle_start_mission(msg):
    return StartMissionResponse(True)

def handle_abort_mission(msg):
    return AbortMissionResponse(True)


def control():

    state = get_state().state
    status = state.mission_status

    if status == MissionStatus.ABORTING:
        rospy.loginfo("Aborting mission")
        # TODO: implement abort logic
    
    elif status == MissionStatus.RUNNING:
        rospy.loginfo("Autonomous navigation")

        # TODO: call path planner

        # TODO: call flight api


    else:
        # Nothing for us to do
        pass


def start():
    global get_state
    get_state = get_proxy('/rh/command/get_state', GetState)

    rospy.Service('command/start_mission', StartMission, handle_start_mission)
    rospy.Service('command/abort_mission', AbortMission, handle_abort_mission)

    rospy.init_node(name())
    rate = rospy.Rate(CONTROL_RATE_HZ)

    rospy.loginfo("Mission controller spinning.")
    while True:
        control()
        rate.sleep()


if __name__ == "__main__":
    start()

