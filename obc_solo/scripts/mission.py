#!/usr/bin/env python
"""
ROS node implementing a higher level navigation controller via Mavros
"""

import threading
import traceback
from functools import partial

import rospy

import mavros
from mavros_msgs.msg import State, Waypoint, ParamValue
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import SetMode, ParamSet
from mavros_msgs.srv import CommandBool, CommandHome, CommandTOL, \
    WaypointPush, WaypointClear

from obc_solo.srv import TakeOff, TakeOffResponse
from obc_solo.srv import Land, LandResponse
from obc_solo.srv import FlyTo, FlyToResponse
from aggregator import LatchMap

isQuadPlane = True

MODE_AUTO = 'AUTO'
MODE_GUIDED = 'GUIDED'
MODE_LOITER = 'QLOITER' if isQuadPlane else 'LOITER'

MAV_FRAME_GLOBAL = 0
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

MAV_CMD_WAYPOINT = 16
MAV_CMD_RTL = 20
MAV_CMD_TAKEOFF = 84 if isQuadPlane else 22
MAV_CMD_LAND = 85 if isQuadPlane else 21
MAV_CMD_DO_SET_HOME = 179

gps_topic = None
mode_sub = None
set_param = None
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


def set_int_param(param, value):

    try:
        val = ParamValue()
        val.integer = value
        val.real = 0.0
        ret = set_param(param, val)

    except rospy.ServiceException:
        rospy.logerr('Set param failed:\n' + ''.join(traceback.format_stack()))

    if not ret.success:
        rospy.logerr("Set param failed: unknown")
        return False
    
    return True


def set_custom_mode(custom_mode):

    rospy.loginfo("Mode requested: %s", custom_mode)
    global mode_sub

    if mode_sub and mode_sub.type:
        rospy.logerr("Mode change is already in progress")
        return

    def clean():
        global mode_sub
        if mode_sub and mode_sub.type:
            mode_sub.unregister()
            mode_sub = None

    done_evt = threading.Event()
    def state_cb(state):
        global mode_sub
        if state.mode == custom_mode:
            rospy.loginfo("Mode changed to %s", state.mode)
            done_evt.set()
            clean()

    mode_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)

    try:
        ret = set_mode(base_mode=0, custom_mode=custom_mode)
    except rospy.ServiceException:
        rospy.logerr('Set mode failed:\n' + ''.join(traceback.format_stack()))

    success = False
    if not ret.success:
        rospy.logerr("Set mode failed: unknown")
    elif not done_evt.wait(5):
        rospy.logerr("Set mode failed: timed out")
    else:
        success = True

    clean()
    return success


def do_takeoff_cur_gps(min_pitch, yaw, altitude):

    #fix = rospy.wait_for_message(gps_topic, NavSatFix, timeout=10)
    fix = values.get_value(gps_topic)
    if not fix:
        rospy.logerr("No GPS fix is latched")
        return

    rospy.loginfo("Taking-off from current coord: Lat: %f Long %f", \
            fix.latitude, fix.longitude)
    rospy.loginfo("With desired Altitude: %f, Yaw: %f, Pitch angle: %f", \
            altitude, yaw, min_pitch)
    try:
        ret = takeoff(min_pitch=min_pitch, \
                         yaw=yaw, \
                         latitude=fix.latitude, \
                         longitude=fix.longitude, \
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

    rospy.loginfo("Landing on current coord: Lat: %f Long: %f", \
            fix.latitude, fix.longitude)
    rospy.loginfo("With desired Altitude: %f, Yaw: %f", \
            altitude, yaw)

    try:
        ret = land(min_pitch=0.0, \
                      yaw=yaw, \
                      latitude=fix.latitude, \
                      longitude=fix.longitude, \
                      altitude=altitude)
    except rospy.ServiceException:
        rospy.logerr('Error landing:\n' + ''.join(traceback.format_stack()))
        return False

    if not ret.success:
        rospy.logerr('Error landing: unknown')
        return False

    return True


def handle_takeoff(req):

    if not req.altitude:
        raise Exception("Parameter 'altitude' is required")

    rospy.loginfo("Take off and loiter at %f meters" % req.altitude)

    # `rosrun mavros mavsafety arm`
    arm(True)

    # `rosrun mavros mavsys mode -c GUIDED`
    # needed for APM:
    set_custom_mode(MODE_GUIDED)

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


def waypoint(lat, lon, alt):
    w = Waypoint()
    w.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT
    w.is_current = False
    w.autocontinue = True
    w.x_lat = lat
    w.y_long = lon
    w.z_alt = alt
    return w

def wp_home(lat, lon):
    w = waypoint(lat, lon, 0)
    w.frame = MAV_FRAME_GLOBAL
    w.command = MAV_CMD_DO_SET_HOME
    return w

def wp_takeoff(lat, lon, alt):
    w = waypoint(lat, lon, alt)
    w.command = MAV_CMD_TAKEOFF
    return w

def wp_fly(lat, lon, alt, delay=0, radius=20):
    w = waypoint(lat, lon, alt)
    w.command = MAV_CMD_WAYPOINT
    w.param1 = delay  # Hold time
    w.param2 = radius # Position threshold in meters
    return w

def wp_rtl():
    w = waypoint(0, 0, 0)
    w.command = MAV_CMD_RTL
    return w

def wp_land(lat, lon):
    w = waypoint(lat, lon, 0)
    w.command = MAV_CMD_LAND
    return w

def mission_planner(lat0, lon0, lat, lon, cruise_alt):
    w0 = wp_home(lat0, lon0)
    w1 = wp_takeoff(lat0, lon0, cruise_alt)
    w1.is_current = True
    w2 = wp_fly(lat, lon, cruise_alt)
    w3 = wp_land(lat, lon)
    return [w0, w1, w2, w3]


def handle_flyto(req):
    """ Takeoff, fly to the given coordinates, and land
    """
    if isQuadPlane:
        set_int_param('Q_GUIDED_MODE', 1)

    fix = values.get_value(gps_topic)

    set_custom_mode(MODE_LOITER)

    wps = mission_planner(fix.latitude, fix.longitude, \
            req.target_lat, req.target_long, req.cruise_altitude)

    #for i, wp in enumerate(wps):
    #    rospy.loginfo("Waypoint %d" % i)
    #    rospy.loginfo(wp)

    if not push_waypoints(wps):
        rospy.logerr('Error pushing waypoints')
        return FlyToResponse(False)

    rospy.loginfo("Successfully pushed %d waypoints", len(wps))

    # wait for waypoints to be accepted
    # TODO: this should be event driven
    rospy.sleep(5.)

    # try arming a few times
    c = 0
    while not arm(True):
        rospy.sleep(0.5)
        c += 1
        if c>10: break

    set_custom_mode(MODE_GUIDED)
    do_takeoff_cur_gps(0, 0, req.cruise_altitude)
     
    # wait a second
    rospy.sleep(0.2)
    
    # now execute mission waypoints
    set_custom_mode(MODE_AUTO)

    # transition back into copter mode
    #set_custom_mode(MODE_LOITER)

    # attempt to land
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

    global set_mode, set_param, arming, set_home, takeoff, land, wp_push, wp_clear
    set_mode = get_proxy('/mavros/set_mode', SetMode)
    set_param = get_proxy('/mavros/param/set', ParamSet)
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

    rospy.Subscriber(gps_topic, NavSatFix, partial(values.latch_value, gps_topic, max_age=10))

    wp_clear()

    rospy.loginfo("Mission controller ready...")
    rospy.spin()


if __name__ == "__main__":
    start()
