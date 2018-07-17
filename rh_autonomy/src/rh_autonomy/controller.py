#!/usr/bin/env python
"""
ROS node implementing an autonomous mission controller for the MedExpress Challenge

This is the central integration node for the Rhinohawk System. It polls the global state (/rh/state), uses the path planner (/rh/planner) to plan a route, and adjusts course if necessary, using the flight api (/rh/command).
"""

import rospy
import threading
from rh_msgs.msg import GPSCoord
from rh_msgs.srv import GetState, FlyWaypoints
from rh_autonomy.util import get_proxy
from rh_autonomy.state import MissionStatus

from pathfinding.msg._PathSolution import PathSolution
import pathfinding.srv as pps
import pathfinding.msg as ppm

# check state and adjust control every second
CONTROL_RATE_HZ = 1 


class ControllerNode():

    def __init__(self):
        rospy.init_node("controller")
        self.get_state = get_proxy('/rh/command/get_state', GetState)
        self.fly_waypoints = get_proxy('/rh/command/fly_waypoints', FlyWaypoints)
        self.submit_problem = get_proxy('/rh/pathfinder/submitProblem', pps.SubmitProblem)
        self.step_problem = get_proxy('/rh/pathfinder/stepProblem', pps.StepProblem)


    def run_forever(self):
        rate = rospy.Rate(CONTROL_RATE_HZ)
        while True:
            self.control()
            rate.sleep()
    

    def control_aborting(self, state):
        # TODO: implement abort logic
        pass


    def control_running(self, state):
        # defined mission
        mission = state.mission
        geofence = mission.geofence
        mission_wps = mission.mission_wps
        static_nfzs = mission.static_nfzs
        roads = mission.roads
        dynamic_nfzs = state.dynamic_nfzs

        # current goal
        tmi = state.target_mission_wp
        target = mission.mission_wps[tmi]

        # TODO: figure out what we want to do
        

        after_target = mission.mission_wps[tmi+1]

        # vehicle state
        vs = mission.vehicle_state

        # set up path planner parameters
        params = ppm.Params()
        params.waypointAcceptanceRadii = 5.0
        params.nfzBufferSize = 10.0

        # convert to path planner messages
        pp_geofence = [ppm.GPSCoord(p.lat, p.lon) for p in geofence.points]
        pp_roads = [ppm.Road( \
                startPoint=ppm.GPSCoord(road.points[0].lat, road.points[0].lon), \
                endPoint=ppm.GPSCoord(road.points[1].lat, road.points[1].lon)) \
                for road in roads]
        pp_static_nfzs = [ppm.NoFlyZone(points=[ppm.GPSCoord(p.lat, p.lon) for p in nfz.points]) for nfz in static_nfzs]
        pp_dynamic_nfzs = [ppm.NoFlyZone(points=[ppm.GPSCoord(p.lat, p.lon) for p in nfz.points]) for nfz in dynamic_nfzs]
        pp_waypoints = [ppm.GPSCoord(target.lat, target.lon),\
                ppm.GPSCoord(after_target.lat, after_target.lon)]

        # create path planner scenario
        scenario = ppm.Scenario()
        scenario.boundaryPoints = pp_geofence
        scenario.noFlyZones = pp_static_nfzs + pp_dynamic_nfzs
        scenario.roads = pp_roads
        scenario.startPoint = ppm.GPSCoord(vs.lat, vs.lon)
        scenario.startVelocity = ppm.GPSVelocity(vs.heading, vs.airspeed)
        scenario.wayPoints = pp_waypoints

        vehicle = ppm.Vehicle()
        vehicle.maxSpeed = 10.0
        vehicle.acceleration = 2.0

        refgps = ppm.GPSCoord()

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

        self.submit_problem(params, scenario, vehicle, refgps)

        if not done_evt.wait(2):
            rospy.logerr("Path finder did not return solution in time")


    def control(self):

        state = self.get_state().state
        status = state.mission_status

        if status == MissionStatus.ABORTING:
            rospy.loginfo("Aborting mission")
            self.control_aborting(state)

        elif status == MissionStatus.RUNNING:
            rospy.loginfo("Autonomous navigation")
            self.control_running(state)

        else:
            # Nothing for us to do
            pass


if __name__ == "__main__":
    node = ControllerNode()
    rospy.loginfo("Mission controller spinning.")
    node.run_forever()

