#!/usr/bin/env python
"""
ROS node implementing a higher level navigation controller via Mavros
"""

import threading
from functools import partial

import rospy

import mavros
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, Waypoint, ParamValue
from mavros_msgs.srv import SetMode, ParamSet
from mavros_msgs.srv import CommandBool, CommandHome, CommandTOL, \
    WaypointPush, WaypointClear

from rh_msgs.srv import TakeOff, TakeOffResponse
from rh_msgs.srv import Land, LandResponse
from rh_msgs.srv import FlyTo, FlyToResponse
from rh_msgs.srv import FlyWaypoints, FlyWaypointsResponse
from rh_autonomy.aggregator import LatchMap
from rh_autonomy.util import get_proxy, waypoints_to_str, wp_lists_equal, logexc
from rh_autonomy import constants as rhc

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
apm_wps = []
current_mode = None

def name():
    return "flight"


def arm(state):
    try:
        ret = arming(value=state)
    except rospy.ServiceException as e:
        logexc('Arming failed')
        return False

    if not ret.success:
        rospy.logerr("Arming failed: unknown")
        return False

    return True


def set_int_param(param, value):

    try:
        val = ParamValue()
        val.integer = value
        val.real = 0.0
        ret = set_param(param, val)

    except rospy.ServiceException as e:
        logexc('Set param failed')
        return False

    if not ret.success:
        rospy.logerr("Set param failed: unknown")
        return False
    
    return True


def set_custom_mode(custom_mode):

    global current_mode, mode_sub
    if current_mode == custom_mode:
        return

    rospy.logdebug("Mode requested: %s", custom_mode)

    if mode_sub and mode_sub.type:
        rospy.logerr("Mode change is already in progress")
        return

    def clean():
        global mode_sub
        if mode_sub and mode_sub.type:
            rospy.logdebug("Clearing mode sub")
            mode_sub.unregister()
            mode_sub = None

    done_evt = threading.Event()
    def state_cb(state):
        global mode_sub
        if state.mode == custom_mode:
            global current_mode
            current_mode = state.mode
            rospy.loginfo("Mode changed to %s", state.mode)
            done_evt.set()
            clean()

    mode_sub = rospy.Subscriber('/mavros/state', State, state_cb)
    success = False

    try:
        ret = set_mode(base_mode=0, custom_mode=custom_mode)
        if not ret.mode_sent:
            rospy.logerr("Set mode %s failed: unknown" % custom_mode)
        elif not done_evt.wait(10):
            rospy.logerr("Set mode %s failed: timed out" % custom_mode)
        else:
            success = True
    except rospy.ServiceException as e:
        logexc('Set mode failed')
        return False

    if not success:
        clean()

    return success


def do_takeoff_cur_gps(min_pitch, yaw, altitude):

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
    except rospy.ServiceException as e:
        logexc('Error taking off')
        return False

    if not ret.success:
        rospy.logerr('Error taking off: unknown')
        return False

    return True


def do_land_cur_gps(yaw, altitude):

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
    except rospy.ServiceException as e:
        logexc('Error landing')
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
    set_custom_mode(rhc.MODE_GUIDED)

    # `rosrun mavros mavcmd takeoffcur 0 0 5.0`
    #cmd('takeoffcur', 0, 0, req.altitude)
    ret = do_takeoff_cur_gps(0, 0, req.altitude)

    return TakeOffResponse(ret)


def handle_land(req):

    # `rosrun mavros mavcmd land 0 0 0 0.0`

    rospy.loginfo("Landing...")

    ret = do_land_cur_gps(0, 0)
    return LandResponse(ret)


def push_waypoints(waypoints):
    try:
        ret = wp_push(start_index=0, waypoints=waypoints)
    except rospy.ServiceException as e:
        logexc("Exception pushing waypoints")
        return False

    if not ret.success:
        rospy.logerr('Error setting waypoints: unknown')
        return False

    if not ret.wp_transfered == len(waypoints):
        rospy.logerr("Only %d/%d waypoints were transferred" % (ret.wp_transfered,len(waypoints)))
        return False

    return True


def waypoint(lat, lon, alt):
    w = Waypoint()
    w.frame = rhc.MAV_FRAME_GLOBAL_RELATIVE_ALT
    w.is_current = False
    w.autocontinue = True
    w.x_lat = lat
    w.y_long = lon
    w.z_alt = alt
    return w

def wp_copy(wp):
    w = Waypoint()
    w.command = wp.command
    w.frame = wp.frame
    w.is_current = wp.is_current
    w.autocontinue = wp.autocontinue
    w.x_lat = wp.x_lat
    w.y_long = wp.y_long
    w.z_alt = wp.z_alt
    w.param1 = wp.param1
    w.param2 = wp.param2
    w.param3 = wp.param3
    w.param4 = wp.param4
    return w

def wp_home(lat, lon):
    w = waypoint(lat, lon, 0)
    w.frame = rhc.MAV_FRAME_GLOBAL
    w.command = rhc.MAV_CMD_DO_SET_HOME
    return w

def wp_takeoff(lat, lon, alt):
    w = waypoint(lat, lon, alt)
    w.command = rhc.MAV_CMD_TAKEOFF
    return w

def wp_fly(lat, lon, alt, delay=0, radius=20):
    w = waypoint(lat, lon, alt)
    w.command = rhc.MAV_CMD_WAYPOINT
    w.param1 = delay  # Hold time
    w.param2 = radius # Position threshold in meters
    return w

def wp_rtl():
    w = waypoint(0, 0, 0)
    w.command = rhc.MAV_CMD_RTL
    return w

def wp_land(lat, lon):
    w = waypoint(lat, lon, 0)
    w.command = rhc.MAV_CMD_LAND
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
    if rhc.isQuadPlane:
        set_int_param('Q_GUIDED_MODE', 1)

    fix = values.get_value(gps_topic)

    if not fix:
        rospy.logerr('No GPS fix')
        return FlyToResponse(False)

    set_custom_mode(rhc.MODE_LOITER)

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
    rospy.sleep(2.)

    # try arming a few times
    c = 0
    while not arm(True):
        rospy.sleep(0.5)
        c += 1
        if c>10: break

    set_custom_mode(rhc.MODE_GUIDED)
    do_takeoff_cur_gps(0, 0, req.cruise_altitude)
     
    # wait a second
    rospy.sleep(0.2)
    
    # now execute mission waypoints
    set_custom_mode(rhc.MODE_AUTO)

    # transition back into copter mode
    #set_custom_mode(MODE_LOITER)

    # attempt to land
    #if not do_land_cur_gps(0, 0):
    #    return FlyToResponse(False)

    return FlyToResponse(True)


def handle_flywaypoints(msg):

    global apm_wps
    mission_goal_id = msg.mission_goal_id
    cruise_alt = msg.cruise_alt
    wp_radius = msg.wp_radius

    # create waypoint list
    wps = []
    for i, gps_coord in enumerate(msg.waypoints.points):
        
        if i==0:
            # due to a bug in Ardupilot, we need a dummy waypoint first
            dummy = wp_home(gps_coord.lat, gps_coord.lon)
            wps.append(dummy)
            # we also abuse this to encode the mission goal
            dummy.param1 = float(mission_goal_id + rhc.GOAL_ID_START)
      	
        if i==0 and msg.takeoff:
            wp = wp_takeoff(gps_coord.lat, gps_coord.lon, cruise_alt)
            wps.append(wp)
            
        wp = wp_fly(gps_coord.lat, gps_coord.lon, cruise_alt, radius=wp_radius)
        wps.append(wp)

    first = wps[1]
    # activate the first waypoint
    first.is_current = True

    if msg.land:
        last = wps[-1]
        # replicate last waypoint on the ground
        wps.append(wp_land(last.x_lat, last.y_long))

    # check if FCU already has this waypoint list
    if wp_lists_equal(apm_wps,wps):
        rospy.logdebug("Waypoint list is still good")
        return FlyWaypointsResponse(True)

    # set to GUIDED before pushing waypoints, so that mission is reset
    set_custom_mode(rhc.MODE_GUIDED)

    if not push_waypoints(wps):
        rospy.logerr('Error pushing waypoints')
        return FlyWaypointsResponse(False)

    apm_wps = wps
    rospy.logdebug("Pushed %d waypoints for goal %d\n%s" \
            % (len(wps), mission_goal_id, waypoints_to_str(wps)))

    # wait for waypoints to be accepted
    # TODO: this should be event driven
    rospy.sleep(1.)
    
    if msg.takeoff:
        # because take off waypoints do not work, 
        # we have to do a manual take off

        # cannot arm in AUTO mode
        set_custom_mode(rhc.MODE_GUIDED)

        # try arming a few times
        c = 0
        while not arm(True):
            rospy.sleep(0.5)
            c += 1
            if c>10: break

        if not do_takeoff_cur_gps(0, 0, cruise_alt):
            return FlyWaypointsResponse(False)
         
        # wait a second
        rospy.sleep(0.5)
    
    # now execute mission waypoints
    set_custom_mode(rhc.MODE_AUTO)

    return FlyWaypointsResponse(True)


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
    gps_topic = mavros.get_topic('global_position', 'global')

    rospy.Service('command/takeoff', TakeOff, handle_takeoff)
    rospy.Service('command/land', Land, handle_land)
    rospy.Service('command/flyto', FlyTo, handle_flyto)
    rospy.Service('command/fly_waypoints', FlyWaypoints, handle_flywaypoints)

    rospy.Subscriber("/mavros/global_position/global", NavSatFix, partial(values.latch_value, gps_topic, max_age=10))

    rospy.sleep(2)
    rospy.loginfo("Clearing waypoints")
    wp_clear()

    rospy.loginfo("Flight controller ready.")
    rospy.spin()


if __name__ == "__main__":
    start()
