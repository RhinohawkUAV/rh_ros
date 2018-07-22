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

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
import mavros_msgs.msg as mrm

from rh_msgs.msg import State, Mission, GPSCoord, VehicleState
from rh_msgs.srv import GetState, GetStateResponse
from rh_msgs.srv import SetMission, SetMissionResponse
from rh_msgs.srv import StartMission, StartMissionResponse
from rh_msgs.srv import AbortMission, AbortMissionResponse
from rh_msgs.srv import SetNoFlyZones, SetNoFlyZonesResponse
from rh_autonomy.aggregator import LatchMap
from rh_autonomy.util import waypoints_to_str, gps_dist
from rh_autonomy import constants as rhc

class MissionStatus:
    NOT_READY = 1
    READY = 2
    RUNNING = 3
    ABORTING = 4

class VehicleStatus:
    GROUNDED = 1
    FLYING = 2

CONTROL_RATE_HZ = 1
gps_topic = "/mavros/global_position/global"
compass_topic = "/mavros/global_position/compass_hdg"
vfr_topic = "/mavros/vfr_hud"
wp_change_topic = "/mavros/mission/waypoints"

def log(s): rospy.loginfo("STATE: %s" % s)
def warn(s): rospy.logwarn("STATE: %s" % s)


class StateNode():

    def __init__(self):
        
        self.values = LatchMap()
        self.mission = Mission()
        self.dynamic_nfzs = []
        self.target_mission_wp = 0
        self.apm_wps = None
        self.reached_apm_wp = -1
        self.mission_status = MissionStatus.NOT_READY
        self.vehicle_status = VehicleStatus.GROUNDED
        self.landing_location = GPSCoord()

        rospy.init_node("state")

        self.sub(gps_topic, NavSatFix)
        self.sub(compass_topic, Float64)
        self.sub(vfr_topic, mrm.VFR_HUD)
        #self.sub(wp_change_topic, mrm.WaypointList, max_age=None)

        rospy.Subscriber("/mavros/state", mrm.State, self.mavros_state_change)
        rospy.Subscriber("/mavros/mission/reached", mrm.WaypointReached, self.waypoint_reached)
        rospy.Subscriber("/mavros/statustext/recv", mrm.StatusText, self.mavlink_statustext)
        rospy.Subscriber(wp_change_topic, mrm.WaypointList, self.waypoints_changed)
    
        self.state_pub = rospy.Publisher("state", State, queue_size = 5)

        rospy.Service('command/set_mission', SetMission, self.handle_set_mission)
        rospy.Service('command/start_mission', StartMission, self.handle_start_mission)
        rospy.Service('command/abort_mission', AbortMission, self.handle_abort_mission)
        rospy.Service('command/set_dnfzs', SetNoFlyZones, self.handle_set_dnfzs)
        rospy.Service('command/get_state', GetState, self.handle_get_state)
 

    def run_forever(self):
        rate = rospy.Rate(CONTROL_RATE_HZ)
        while True:

            self.check_goal()
            
            if self.state_pub.get_num_connections() > 0:
                self.state_pub.publish(self.get_state())
            
            rate.sleep()


    def check_goal(self):

        if self.mission_status != MissionStatus.RUNNING:
            return

        if self.target_mission_wp > len(self.mission.mission_wps.points)-1:
            return

        target = self.mission.mission_wps.points[self.target_mission_wp]
        gps_position = self.values.get_value(gps_topic)
        curr_pos = GPSCoord(gps_position.latitude, gps_position.longitude, 1)

        #for i, point in enumerate(self.mission.mission_wps.points):
        #    d = gps_dist(curr_pos, point)
        #    rospy.loginfo("Goal %d - distance %f"%(i,d))
        
        # are we close to the goal?
        d = gps_dist(curr_pos, target)
        log("Distance from goal: %2.6fm" % d)

        #if d_in_meters < rhc.WAYPOINT_ACCEPTANCE_RADIUS:
        if d < 0.0002:
            self.goal_reached(self.target_mission_wp)
            self.reached_apm_wp = 0


    def goal_reached(self, index):
        log("Reached goal %d" % self.target_mission_wp)
        log("----------------------------------------------------")
        if index == len(self.mission.mission_wps.points)-1:
            rospy.loginfo("Landing at remote location")
        # get next goal
        self.target_mission_wp += 1


    def sub(self, topic, data_type, max_age=10):
        rospy.Subscriber(topic, data_type, \
                partial(self.values.latch_value, topic, max_age=max_age))


    def mavros_state_change(self, msg):

        if msg.system_status==4:
            if self.vehicle_status != VehicleStatus.FLYING:
                self.vehicle_status = VehicleStatus.FLYING
                log("Flying")
        else:
            if self.vehicle_status != VehicleStatus.GROUNDED:
                self.vehicle_status = VehicleStatus.GROUNDED
                log("Landed")


    def waypoints_changed(self, msg):
        self.apm_wps = msg.waypoints
        if self.apm_wps:

            gs = ""
            mission_goal_id = int(self.apm_wps[0].param1) - rhc.GOAL_ID_START
            if mission_goal_id<0:
                gs = " goal %d" % mission_goal_id
            rospy.loginfo("Received%s waypoints (curr=%d):\n%s" % \
                    (gs,msg.current_seq, waypoints_to_str(self.apm_wps)))

                

            rospy.logdebug("Got waypoint list for goal %d"%mission_goal_id)


    def waypoint_reached(self, msg):
        """ Called whenever an APM waypoint is reached
        """
        if not self.apm_wps:
            warn("Reached waypoint, but no waypoints known")
            return

        apm_wps = self.apm_wps
        mission_goal_id = int(apm_wps[0].param1) - rhc.GOAL_ID_START

        if mission_goal_id<0:
            rospy.logwarn("Got waypoint list with no goal id")
            return

        if mission_goal_id == self.target_mission_wp:
            # This is the waypoint list for the current mission objective
                
            if self.reached_apm_wp < msg.wp_seq:
                log("Reached APM waypoint %s" % msg.wp_seq)
                self.reached_apm_wp = msg.wp_seq

                #if msg.wp_seq in apm_wps:
                #    waypoint = apm_wps[msg.wp_seq]
                #else:
                #    warn("Waypoint %d not in APM waypoints" % msg.wp_seq)

                # the second to last waypoint is our current goal
                #if msg.wp_seq >= len(apm_wps)-2:

            else:
                log("Already reached APM waypoint %s" % msg.wp_seq)
        else:
            log("Received waypoints for goal %d, but looking for goal %d" % (mission_goal_id, self.target_mission_wp))
 

    def mavlink_statustext(self, msg):
        #log("Got Mavlink msg: %s " % msg.text)
        #if msg == 'Land complete':
        #    log("On the ground")
        pass 


    def handle_set_mission(self, msg):
        """ Set mission parameters
        """
        self.mission = msg.mission
        log("New mission parameters have been set")
        return SetMissionResponse(True)


    def handle_start_mission(self, msg):
        """ Start mission
        """
        if not self.mission.mission_wps.points:
            rospy.loginfo("No mission waypoints defined")
            return StartMissionResponse(False)

        # TODO: elsewhere, set status=READY once we're ready to fly

        # TODO: check for status==READY

        self.mission_status = MissionStatus.RUNNING
        return StartMissionResponse(True)


    def handle_abort_mission(self, msg):
        """ Abort mission with extreme prejudice
        """
        self.mission_status = MissionStatus.ABORTING
        return AbortMissionResponse(True)


    def handle_set_dnfzs(self, msg):
        self.dynamic_nfzs = msg.dynamic_nfzs
        log("New dynamic no-fly-zones have been set")
        return SetNoFlyZonesResponse(True)

    
    def handle_get_state(self, msg):
        return GetStateResponse(self.get_state())
 

    def get_state(self):
        state = State()
        state.mission = self.mission
        state.dynamic_nfzs = self.dynamic_nfzs
        state.target_mission_wp = self.target_mission_wp
        vehicle_state = VehicleState()
        vehicle_state.status = self.vehicle_status
        state.vehicle_state = vehicle_state

        gps_position = self.values.get_value(gps_topic)
        if gps_position:
            vehicle_state.position.lat = gps_position.latitude
            vehicle_state.position.lon = gps_position.longitude
            vehicle_state.position.alt = gps_position.altitude

        compass = self.values.get_value(compass_topic)
        if compass:
            vehicle_state.heading = compass.data

        vfr = self.values.get_value(vfr_topic)
        if vfr:
            vehicle_state.position.alt = vfr.altitude
            vehicle_state.airspeed = vfr.airspeed

        if self.apm_wps:
            state.apm_wps = self.apm_wps

        state.landing_location = self.landing_location
        state.mission_status = self.mission_status
        return state

if __name__ == "__main__":
    node = StateNode()
    rospy.loginfo("Mission state ready.")
    node.run_forever()

