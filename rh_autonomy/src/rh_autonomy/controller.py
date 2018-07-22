#!/usr/bin/env python
"""
ROS node implementing an autonomous mission controller for the MedExpress Challenge

This is the central integration node for the Rhinohawk System. It polls the global state (/rh/state), uses the path planner (/rh/planner) to plan a route, and adjusts course if necessary, using the flight api (/rh/command).
"""

import rospy
import actionlib

from rh_msgs.msg import GPSCoord, GPSCoordList
from rh_msgs.srv import GetState, FlyWaypoints
from rh_autonomy.util import get_proxy
from rh_autonomy.state import MissionStatus, VehicleStatus
import pathfinding.msg as pfm

# check state and adjust control every other second
CONTROL_RATE_HZ = 0.5
CRUISE_ALTITUDE = 20
WAYPOINT_ACCEPTANCE_RADIUS = 10
NOFLYZONE_BUFFER_SIZE = 10


class ControllerNode():

    def __init__(self):
        rospy.init_node("controller")
        self.get_state = get_proxy('/rh/command/get_state', GetState)
        self.fly_waypoints = get_proxy('/rh/command/fly_waypoints', FlyWaypoints)
        self.cruise_alt = CRUISE_ALTITUDE
        self.wp_radius = WAYPOINT_ACCEPTANCE_RADIUS
        self.nfz_buffer_size = NOFLYZONE_BUFFER_SIZE
        self.pfclient = actionlib.SimpleActionClient("path_finder_server", pfm.PathFinderAction)
        rospy.logdebug("Waiting for path finder server...")
        self.pfclient.wait_for_server()
        rospy.logdebug("Connection to path finder established.")


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
        mission_wps = mission.mission_wps.points

        # current goal
        tmi = state.target_mission_wp

        if tmi == len(mission_wps)-1:
            # last mission waypoint
            # TODO: perform search and precision landing
            target = mission_wps[tmi]
            self.control_waypoint(state, target)
        elif tmi < len(mission_wps):
            target = mission_wps[tmi]
            next_target = mission_wps[tmi+1]
            self.control_waypoint(state, target, next_target)


    def control_waypoint(self, state, target, next_target=None): 

         # defined mission
        mission = state.mission
        geofence = mission.geofence
        static_nfzs = mission.static_nfzs
        roads = mission.roads
        
        # current state
        dynamic_nfzs = state.dynamic_nfzs
        vs = state.vehicle_state
        mission_goal_id = state.target_mission_wp

        # set up path planner parameters
        params = pfm.Params()
        params.waypointAcceptanceRadii = self.wp_radius
        params.nfzBufferSize = self.nfz_buffer_size

        # convert to path planner messages
        pp_geofence = [pfm.GPSCoord(p.lat, p.lon) for p in geofence.points]
        pp_roads = [pfm.Road( \
                startPoint=pfm.GPSCoord(road.points[0].lat, road.points[0].lon), \
                endPoint=pfm.GPSCoord(road.points[1].lat, road.points[1].lon)) \
                for road in roads]
        pp_static_nfzs = [pfm.NoFlyZone(points=[pfm.GPSCoord(p.lat, p.lon) for p in nfz.points]) for nfz in static_nfzs]
        pp_dynamic_nfzs = []#[pfm.DynamicNoFlyZone(points=[pfm.GPSCoord(p.lat, p.lon) for p in nfz.points]) for nfz in dynamic_nfzs]
        pp_waypoints = [pfm.GPSCoord(target.lat, target.lon)]
        if next_target:
            pp_waypoints.append(pfm.GPSCoord(next_target.lat, next_target.lon))

        
        # create path planner scenario
        scenario = pfm.Scenario()
        scenario.boundaryPoints = pp_geofence
        scenario.dynamicNoFlyZones = pp_dynamic_nfzs
        scenario.noFlyZones = pp_static_nfzs
        scenario.roads = pp_roads
        scenario.startPoint = pfm.GPSCoord(vs.position.lat, vs.position.lon)
        # path finder throws divide by zero if speed is zero
        speed = vs.airspeed if vs.airspeed > 0 else 0.1 
        scenario.startVelocity = pfm.GPSVelocity(vs.heading, speed)
        scenario.wayPoints = pp_waypoints

        vehicle = pfm.Vehicle()
        vehicle.maxSpeed = 10.0
        vehicle.acceleration = 2.0

        goal = pfm.PathFinderGoal()
        goal.params = params
        goal.scenario = scenario
        goal.vehicle = vehicle

        #rospy.loginfo("Submitting goal:\n%s", goal)

        self.pfclient.send_goal(goal)
        result = self.pfclient.wait_for_result(rospy.Duration.from_sec(5.0))

        wps = [] 

        if not result:
            rospy.logerr("Path finder did not return solution in time")
        else:
            solution = self.pfclient.get_result().solution
            if solution.finished:
                c = len(solution.solutionWaypoints)
                if c==0:
                    rospy.logwarn("Path finder returned no waypoints")
                else:
                    rospy.logdebug("Path finder returned %d solution waypoints" % c)
                for swp in solution.solutionWaypoints:
                    wp = swp.position
                    rospy.logdebug("Solution waypoint: (%s,%s)" % (wp.lat, wp.lon))
                    wps.append(GPSCoord(wp.lat, wp.lon, 1))
            else:
                rospy.logwarn("Path finder solution is not complete")

        if wps:
            #if next_target:
            #    wps.append(next_target)
            rospy.logdebug("Will submit %d waypoints", len(wps))
            if not self.fly_waypoints(mission_goal_id, \
                    self.cruise_alt, self.wp_radius, \
                    GPSCoordList(wps), \
                    vs.status == VehicleStatus.GROUNDED, \
                    next_target==None):
                rospy.logerr("Could not fly waypoints")
        else:
            rospy.logdebug("No solution waypoints!")


    def control(self):

        state = self.get_state().state
        status = state.mission_status

        if status == MissionStatus.ABORTING:
            rospy.logwarn("Aborting mission")
            self.control_aborting(state)

        elif status == MissionStatus.RUNNING:
            rospy.logdebug("Autonomous navigation")
            self.control_running(state)

        else:
            # Nothing for us to do
            pass


if __name__ == "__main__":
    node = ControllerNode()
    rospy.loginfo("Mission controller spinning.")
    node.run_forever()

