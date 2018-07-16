#!/usr/bin/env python
"""
ROS node implementing an autonomous mission controller for the MedExpress Challenge

This is the central integration node for the Rhinohawk System. It polls the global state (/rh/state), uses the path planner (/rh/planner) to plan a route, and adjusts course if necessary, using the flight api (/rh/command).
"""

import rospy
import threading
from rh_msgs.srv import GetState, FlyWaypoints
from rh_autonomy.util import get_proxy
from rh_autonomy.state import MissionStatus

from pathfinding.msg._PathSolution import PathSolution
from pathfinding.srv import SubmitProblem, StepProblem
from pathfinding.msg import Params, Scenario, Vehicle, GPSCoord, GPSVelocity

# check state and adjust control every second
CONTROL_RATE_HZ = 1 


class ControllerNode():

    def __init__(self):
        rospy.init_node("controller")
        self.get_state = get_proxy('/rh/command/get_state', GetState)
        self.fly_waypoints = get_proxy('/rh/command/fly_waypoints', FlyWaypoints)
        self.submit_problem = get_proxy('/rh/pathfinder/submitProblem', SubmitProblem)
        self.step_problem = get_proxy('/rh/pathfinder/stepProblem', StepProblem)


    def run_forever(self):
        rate = rospy.Rate(CONTROL_RATE_HZ)
        while True:
            self.control()
            rate.sleep()


    def control(self):

        state = self.get_state().state
        status = state.mission_status

        if status == MissionStatus.ABORTING:
            rospy.loginfo("Aborting mission")
            # TODO: implement abort logic
        
        elif status == MissionStatus.RUNNING:
            rospy.loginfo("Autonomous navigation")

            # defined mission
            mission = state.mission
            geofence = mission.geofence
            mission_wps = mission.mission_wps
            static_nfzs = mission.static_nfzs
            roads = mission.roads

            # current goal
            target = None

            # vehicle state
            vs = mission.vehicle_state

            # call path planner
            params = Params()
            params.waypointAcceptanceRadii = 5.0
            params.nfzBufferSize = 10.0

            scenario = Scenario()
            scenario.boundaryPoints = []
            scenario.noFlyZones = []
            scenario.roads = []
            scenario.startPoint = GPSCoord(vs.lat, vs.lon)
            scenario.startVelocity = GPSVelocity(vs.heading, vs.airspeed)
            scenario.wayPoints = GPSCoord(target.lat, target.long)

            vehicle = Vehicle()
            vehicle.maxSpeed = 10.0
            vehicle.acceleration = 2.0

            refgps = GPSCoord()

            self.submit_problem(params, scenario, vehicle, refgps)

            done_evt = threading.Event()

            def receive_solution(msg):
                rospy.loginfo("Got path solution")
                global solution_sub
                done_evt.set()
                solution_sub.unregister()
                solution_sub = None

                wps = [] 
                for wp in msg.solutionWaypoints:
                    rospy.loginfo("Solution waypoint: (%s,%s) (radius=%s)" % (wp.position.lat, wp.position.lon, wp.radius))
                    wps.append(GPSCoord(wp.lat, wp.lon))
                 
                if not self.fly_waypoints(wps):
                    rospy.logerr("Could not fly waypoints")


            global solution_sub
            solution_sub = rospy.Subscriber('/rh/pathfinder/pathFinderSolution', PathSolution, receive_solution)

            if not done_evt.wait(2):
                rospy.logerr("Path finder did not return solution in time")

            #solution = rospy.wait_for_message('/rh/pathfinder/pathFinderSolution', PathSolution)


        else:
            # Nothing for us to do
            pass


if __name__ == "__main__":
    node = ControllerNode()
    rospy.loginfo("Mission controller spinning.")
    node.run_forever()

