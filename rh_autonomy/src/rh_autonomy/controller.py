#!/usr/bin/env python
"""
ROS node implementing an autonomous mission controller for the MedExpress Challenge

This is the central integration node for the Rhinohawk System. It polls the global state (/rh/state), uses the path planner (/rh/planner) to plan a route, and adjusts course if necessary, using the flight api (/rh/command).
"""

import rospy
import actionlib

from rh_msgs.msg import GPSCoord, GPSCoordList
from rh_msgs.srv import GetState, GetCameraMetadata, FlyWaypoints, GenerateSearchPattern
from rh_autonomy.util import get_proxy
from rh_autonomy.state import MissionStatus, VehicleStatus
from rh_autonomy.search_pattern import create_waypoints
import rh_autonomy.constants as rhc
import rh_pathfinding.msg as pfm

# check state and adjust control every other second
CONTROL_RATE_HZ = 0.5
DEFAULT_STRIP_WIDTH = 10 # meters

def calculate_ground_res(m, altitude):
    """ Given camera metadata and an altitude, this function calculates the 
        resulting ground resolution (cm of ground represented by a single pixel)
    """
    x = (altitude * m.sensor_width * 100) / (m.image_width * m.focal_length)
    y = (altitude * m.sensor_height * 100) / (m.image_height * m.focal_length)
    return (x, y)


def calculate_strip_width(altitude, overlap):
    
    # Attempt to get metadata from the camera
    try:
        get_metadata = rospy.ServiceProxy('/camera/get_metadata', GetCameraMetadata)
        res = get_metadata()
        if not res: raise Exception("Null camera metadata result")
    except:
            rospy.logwarn("Could not get camera metadata. Using default strip width.")
            return DEFAULT_STRIP_WIDTH

    m = res.metadata
    ground_res = calculate_ground_res(m, altitude)
    rospy.loginfo("Ground Sampling Distance: (%2.2fmm, %2.2fm)" % ground_res)
    if ground_res[0] > 2.63:
        rospy.logwarn("Ground sampling distance (GSD) is greater than 2.63 "+\
                "and will not allow for arUco marker detection.")
    # amount of area covered by a single image
    image_res = (m.image_width * ground_res[0], m.image_height * ground_res[1])
    # for now, ignore frontal overlap
    rospy.loginfo("Ground Resolution: (%2.2fm, %2.2fm)" % image_res)
    return image_res[0] - overlap



class ControllerNode():

    def __init__(self):
        rospy.init_node("controller")
        self.get_state = get_proxy('/rh/command/get_state', GetState)
        self.fly_waypoints = get_proxy('/rh/command/fly_waypoints', FlyWaypoints)
        self.search_strip_width = calculate_strip_width(rhc.SEARCH_ALTITUDE, rhc.SEARCH_STRIP_OVERLAP)
        self.search_gcl = None
        self.cruise_alt = rhc.CRUISE_ALTITUDE
        self.wp_radius = rhc.WAYPOINT_ACCEPTANCE_RADIUS
        self.nfz_buffer_size = rhc.NOFLYZONE_BUFFER_SIZE
        self.nfz_target_offset = rhc.NOFLYZONE_TARGET_OFFSET
        self.nfz_vertex_heuristic_weight = rhc.NOFLYZONE_HEURISTIC_WEIGHT
        self.pathfinder_timeout = rhc.PATHFINDER_TIMEOUT
        self.pfclient = actionlib.SimpleActionClient("/rh/pathfinder/action", pfm.SolvePathProblemAction)
        rospy.logdebug("Waiting for path finder server...")
        self.pfclient.wait_for_server()
        rospy.logdebug("Connection to path finder established.")


    def run_forever(self):
        rate = rospy.Rate(CONTROL_RATE_HZ)
        while True:
            self.control()
            rate.sleep()
    

    def control(self):

        state = self.get_state().state
        status = state.mission_status
        
        if status == MissionStatus.ABORTING:
            rospy.logwarn("Aborting mission")
            self.control_aborting(state)

        elif status == MissionStatus.RUNNING:
            rospy.logdebug("Autonomous navigation")
            self.control_running(state)

        elif status == MissionStatus.READY:
            pass

        elif status == MissionStatus.NOT_READY:
            pass

        elif status == MissionStatus.COMPLETE:
            pass

        else:
            rospy.logwarn("Unknown mission status: %d"%status)


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
            rospy.loginfo("Control to last waypoint")

            # last mission waypoint
            target = mission_wps[tmi]

            if rhc.PERFORM_SEARCH:

                # Have we found a place to land yet?
                if state.landing_location.lat and state.landing_location.lon:
                    self.control_landing(state, state.landing_location)
                
                # If not, perform a search
                else:
                    self.control_search(state, target)

            else:
                # No search, just land at the last waypoint
                self.control_waypoint(state, target)

        elif tmi < len(mission_wps):
            rospy.loginfo("Control to goal %d"%tmi)
            target = mission_wps[tmi]
            next_target = mission_wps[tmi+1]
            self.control_waypoint(state, target, next_target)



    def control_search(self, state, target):

        vs = state.vehicle_state
        mission_goal_id = state.target_mission_wp
        
        if not self.search_gcl:
            # TODO: this currently generates the search pattern once per mission
            # it needs better logic to account for dynamic nfz changes, and other recovery
            self.search_gcl = self.generate_search_pattern(vs.position, target)

        if self.search_gcl:
            rospy.logdebug("Will submit %d waypoints", len(self.search_gcl.points))
            if not self.fly_waypoints(mission_goal_id, \
                    self.cruise_alt, self.wp_radius, \
                    self.search_gcl, \
                    False, \
                    False):
                rospy.logerr("Could not fly waypoints")
        else:
            rospy.logdebug("No search waypoints!")


    def control_landing(self, state, target):

        vs = state.vehicle_state
        mission_goal_id = state.target_mission_wp
        
        wpl = GPSCoordList()
        wpl.points = [target]
        rospy.logdebug("Will submit final landing waypoint")
        if not self.fly_waypoints(mission_goal_id, \
                self.cruise_alt, self.wp_radius, \
                wpl, \
                False, \
                True):
            rospy.logerr("Could not fly waypoints")


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
        params.nfzBufferWidth = self.nfz_buffer_size
        params.nfzTargetOffset = self.nfz_target_offset
        params.vertexHeuristicMultiplier = self.nfz_vertex_heuristic_weight

        # convert to path planner messages
        pp_geofence = [pfm.GPSCoord(p.lat, p.lon) for p in geofence.points]
        pp_roads = [pfm.Road( \
                startPoint=pfm.GPSCoord(road.points[0].lat, road.points[0].lon), \
                endPoint=pfm.GPSCoord(road.points[1].lat, road.points[1].lon)) \
                for road in roads]
        pp_static_nfzs = [pfm.NoFlyZone(points=[pfm.GPSCoord(p.lat, p.lon) for p in nfz.points]) for nfz in static_nfzs]
        pp_dynamic_nfzs = [pfm.DynamicNoFlyZone(points=[pfm.GPSCoord(p.lat, p.lon) for p in nfz.points]) for nfz in dynamic_nfzs]
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
        speed = vs.groundspeed if vs.groundspeed > 0 else 0.1 
        scenario.startVelocity = pfm.GPSVelocity(vs.heading, speed)
        scenario.wayPoints = pp_waypoints

        vehicle = pfm.Vehicle()
        vehicle.maxSpeed = rhc.CRUISE_SPEED + 10
        vehicle.acceleration = 5.0

        goal = pfm.SolvePathProblemGoal()
        goal.params = params
        goal.scenario = scenario
        goal.vehicle = vehicle
        goal.timeout = self.pathfinder_timeout
        goal.emitDebug = True

        #rospy.loginfo("Submitting goal:\n%s", goal)

        self.pfclient.send_goal(goal)
        result = self.pfclient.wait_for_result(rospy.Duration.from_sec(5.0))

        wps = [] 

        if not result:
            rospy.logwarn("Path finder did not return a solution in time.")

        else:
            solution = self.pfclient.get_result().bestPath
        
            q = solution.quality
            if q==0:
                rospy.logwarn("Path finder failed to find solution")
            elif q==1:
                rospy.loginfo("Path finder returned incomplete solution")
            elif q==3:
                rospy.loginfo("Path finder found optimal solution")

            if q>0:
                rospy.loginfo("Path finder solution: numWaypointsCompleted=%d, estimatedTime=%f" % \
                        (solution.numWaypointsCompleted, solution.estimatedTime))

            for swp in solution.solutionWaypoints:
                wp = swp.position
                rospy.logdebug("Solution waypoint: (%s,%s)" % (wp.lat, wp.lon))
                wps.append(GPSCoord(wp.lat, wp.lon, 1))

            rospy.loginfo("Pathfinder returned %d waypoints until goal" % len(wps))

        if not wps:
            rospy.logerr("No solution waypoints. Proceeding directly to goal!")
            wps.append(target)

        if next_target:
            # start heading in the right direction when we get to the target
            wps.append(next_target)

        rospy.logdebug("Will submit %d waypoints", len(wps))
        if not self.fly_waypoints(mission_goal_id, \
                self.cruise_alt, self.wp_radius, \
                GPSCoordList(wps), \
                vs.status == VehicleStatus.GROUNDED, \
                next_target==None):
            rospy.logerr("Could not fly waypoints")


    def generate_search_pattern(self, current_pos, search_target):
        req = GenerateSearchPattern()
        req.current_location = current_pos
        req.target = search_target
        req.search_radius = rhc.SEARCH_RADIUS
        req.strip_width = self.search_strip_width
        req.search_altitude = rhc.SEARCH_ALTITUDE
        res = create_waypoints(req) 
        return res.waypoints



if __name__ == "__main__":
    node = ControllerNode()
    rospy.loginfo("Mission controller spinning.")
    node.run_forever()

