#!/usr/bin/env python
#
# Mission controller for Rhinohawk
#

import threading
import traceback
from functools import partial

import rospy

import mavros
from mavros_msgs.msg import State, Waypoint
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool, CommandHome, CommandTOL, \
    WaypointPush, WaypointClear

from obc_solo.srv import TakeOff, TakeOffResponse
from obc_solo.srv import Land, LandResponse 
from obc_solo.srv import FlyTo, FlyToResponse 
from latch import LatchMap

MAV_GLOBAL_FRAME = 3
MAV_CMD_WAYPOINT = 16
MAV_CMD_RTL = 20
MAV_CMD_LAND = 21
MAV_CMD_TAKEOFF = 22

gps_topic = None
set_mode = None
arming = None
set_home = None
takeoff = None
land = None
wp_push = None
wp_clear = None

values = LatchMap()


def name():
    return "mission"


def arm(state):
    try:
        ret = arming(value=state)
    except rospy.ServiceException:
        rospy.logerr('Arming failed:\n' + ''.join(traceback.format_stack()))
        return False

    if not ret.success:
        rospy.logerr("Arming failed: unknown")
        return False

    return True


def find_gps_topic(op_name, any_gps=False):
    # XXX: since 0.13 global position always exists. need redo that.
    global_fix = mavros.get_topic('global_position', 'global')
    gps_fix = mavros.get_topic('global_position', 'raw', 'fix')

    topics = rospy.get_published_topics()
    # need find more elegant way
    if len([topic for topic, type_ in topics if topic == global_fix]):
        return global_fix
    elif len([topic for topic, type_ in topics if topic == gps_fix]):
        rospy.loginfo("Using GPS_RAW_INT data!")
        return gps_fix
    #elif any_gps:
    #    t = [topic for topic, type_ in topics if type_ == 'sensor_msgs/NavSatFix']
    #    if len(t) > 0:
    #        rospy.loginfo("Using %s NavSatFix topic for %s", t[0], op_name)
    #        return t[0]
    return None


def set_custom_mode(custom_mode):
    
    base_mode = 0

    done_evt = threading.Event()
    def state_cb(state):
        rospy.loginfo("Current mode: %s", state.mode)
        if state.mode == custom_mode:
            rospy.loginfo("Mode changed to %s", state.mode)
            done_evt.set()

    if custom_mode != '' and not custom_mode.isdigit():
        # with correct custom mode we can wait until it changes
        rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
    else:
        done_evt.set()

    try:
        ret = set_mode(base_mode=base_mode, custom_mode=custom_mode)
    except rospy.ServiceException:
        rospy.logerr('Set mode failed:\n' + ''.join(traceback.format_stack()))

    if not ret.success:
        rospy.logerr("Set mode failed: unknown")
        return False

    if not done_evt.wait(5):
        rospy.logerr("Set mode failed: timed out")
        return False

    return True


def do_takeoff_cur_gps(min_pitch, yaw, altitude):

    #fix = rospy.wait_for_message(gps_topic, NavSatFix, timeout=10)
    fix = values.get_value(gps_topic)
    if not fix: 
        rospy.logerr("No GPS fix is latched")
        return
        
    rospy.loginfo("Taking-off from current coord: Lat: %f Long %f", 
            fix.latitude, fix.longitude)
    rospy.loginfo("With desired Altitude: %f, Yaw: %f, Pitch angle: %f", 
            altitude, yaw, min_pitch)
    try:
        ret = takeoff(min_pitch=min_pitch,
                         yaw=yaw,
                         latitude=fix.latitude,
                         longitude=fix.longitude,
                         altitude=altitude)
    except rospy.ServiceException:
        rospy.logerr('Error taking off:\n' + ''.join(traceback.format_stack()))
        return False

    if not ret.success:
        rospy.logerr('Error taking off: unknown')
        return False

    return True


def do_land_cur_gps(yaw, altitude):

    #fix = rospy.wait_for_message(gps_topic, NavSatFix, timeout=10)
    fix = values.get_value(gps_topic)
    if not fix:
        rospy.logerr("No GPS fix is latched")
        return

    rospy.loginfo("Landing on current coord: Lat: %f Long: %f", 
            fix.latitude, fix.longitude)
    rospy.loginfo("With desired Altitude: %f, Yaw: %f", 
            altitude, yaw)

    try:
        ret = land(min_pitch=0.0,
                      yaw=yaw,
                      latitude=fix.latitude,
                      longitude=fix.longitude,
                      altitude=altitude)
    except rospy.ServiceException:
        rospy.logerr('Error landing:\n' + ''.join(traceback.format_stack()))
        return False

    if not ret.success:
        rospy.logerr('Error landing: unknown')
        return False

    return True


def handle_takeoff(req):
    
    if not(req.altitude):
        raise Exception("Parameter 'altitude' is required")

    rospy.loginfo("Take off and loiter at %f meters" % req.altitude)

    # `rosrun mavros mavsafety arm`
    arm(True)

    # `rosrun mavros mavsys mode -c GUIDED`
    # needed for APM:
    #set_mode('GUIDED')

    # `rosrun mavros mavcmd takeoffcur 0 0 5.0`
    #cmd('takeoffcur', 0, 0, req.altitude)
    do_takeoff_cur_gps(0, 0, req.altitude)

    return TakeOffResponse(True)


def handle_land(req):

    # `rosrun mavros mavcmd land 0 0 0 0.0`

    rospy.loginfo("Landing...")
 
    if not do_land_cur_gps(0, 0):
        return LandResponse(False)

    return LandResponse(True)


def push_waypoints(waypoints):
    try:
        ret = wp_push(waypoints)
    except rospy.ServiceException:
        rospy.logerr('Error setting waypoints:\n' + ''.join(traceback.format_stack()))
        return False

    if not ret.success:
        rospy.logerr('Error setting waypoints: unknown')
        return False

    return True


def waypoint(lat, lon, alt, delay):
    w = Waypoint()
    w.frame = MAV_GLOBAL_FRAME
    w.command = MAV_CMD_WAYPOINT
    w.is_current = False
    w.autocontinue = True
    w.param1 = delay # Hold time
    w.param2 = 20     # Position threshold in meters
    w.x_lat = lat
    w.y_long = lon
    w.z_alt = alt
    return w


def wp_rtl():
    w = Waypoint()
    w.frame = MAV_GLOBAL_FRAME
    w.command = MAV_CMD_RTL
    return w

def wp_land(lat, lon, alt):
    w = waypoint(lat, lon, alt, 0)
    w.command = MAV_CMD_LAND
    return w

def wp_takeoff(lat, lon, alt):
    w = waypoint(lat, lon, alt, 0)
    w.command = MAV_CMD_TAKEOFF
    return w

def mission_planner(lat0, lon0, lat, lon, cruise_alt):
    w0 = wp_takeoff(lat0, lon0, cruise_alt)
    w0.is_current = True
    w1 = waypoint(lat, lon, cruise_alt, 0)
    w2 = wp_land(lat, lon, 0) 
    return [w0, w1, w2]


def handle_flyto(req):
    """ Takeoff, fly to the given coordinates, and land
    """
    fix = values.get_value(gps_topic)

    arm(True)
    #do_takeoff_cur_gps(0, 0, req.cruise_altitude)

    set_custom_mode("STABILIZED")

    wps = mission_planner(fix.latitude, fix.longitude, \
            req.target_lat, req.target_long, req.cruise_altitude)

    if not push_waypoints(wps):
        rospy.logerr('Error pushing waypoints')
        return FlyToResponse(False)

    rospy.loginfo("Successfully pushed %d waypoints", len(wps))

    set_custom_mode("AUTO.MISSION")

    # TODO: wait until waypoint is reached
    #rospy.sleep(5.)

    #if not do_land_cur_gps(0, 0):
    #    return FlyToResponse(False)

    return FlyToResponse(True)



def get_proxy(topic, serviceType):
    rospy.loginfo("Waiting for service: %s", topic)
    rospy.wait_for_service(topic)
    return rospy.ServiceProxy(topic, serviceType)


def start():

    rospy.init_node(name())
    mavros.set_namespace()

    global set_mode, arming, set_home, takeoff, land, wp_push, wp_clear
    set_mode = get_proxy('/mavros/set_mode', SetMode)
    arming = get_proxy('/mavros/cmd/arming', CommandBool)
    set_home = get_proxy('/mavros/cmd/set_home', CommandHome)
    takeoff = get_proxy('/mavros/cmd/takeoff', CommandTOL)
    land = get_proxy('/mavros/cmd/land', CommandTOL)
    wp_push = get_proxy('/mavros/mission/push', WaypointPush)
    wp_clear = get_proxy('/mavros/mission/clear', WaypointClear)

    global gps_topic
    gps_topic = find_gps_topic("takeoff")
    if gps_topic is None:
        raise Exception("NavSatFix topic not exist")

    rospy.Service('command/takeoff', TakeOff, handle_takeoff)
    rospy.Service('command/land', Land, handle_land)
    rospy.Service('command/land2', Land, handle_land)
    rospy.Service('command/flyto', FlyTo, handle_flyto)

    rospy.Subscriber(gps_topic, NavSatFix, partial(values.latch_value, gps_topic, 10))

    wp_clear()

    rospy.loginfo("Mission controller ready...")
    rospy.spin()


if __name__ == "__main__":
    start()
