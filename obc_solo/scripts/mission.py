#!/usr/bin/env python
#
# Mission controller for Rhinohawk
#

import threading
import traceback

import rospy

import mavros
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool, CommandHome, CommandTOL

#from geometry_msgs.msg import PointStamped
#from geometry_msgs.msg import Point
#from std_msgs.msg import Header

from obc_solo.srv import TakeOff, TakeOffResponse
from obc_solo.srv import Land, LandResponse 

gps_topic = None
set_mode = None
arming = None
set_home = None
takeoff = None
land = None


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

    fix = rospy.wait_for_message(gps_topic, NavSatFix, timeout=10)
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

    fix = rospy.wait_for_message(gps_topic, NavSatFix, timeout=10)
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
 
    if not(do_land_cur_gps(0, 0)):
        return LandResponse(False)

    return LandResponse(True)


def get_proxy(topic, serviceType):
    rospy.loginfo("Waiting for service: %s", topic)
    rospy.wait_for_service(topic)
    return rospy.ServiceProxy(topic, serviceType)


def start():

    rospy.init_node(name())
    mavros.set_namespace()

    global set_mode, arming, set_home, takeoff, land
    set_mode = get_proxy('/mavros/set_mode', SetMode)
    arming = get_proxy('/mavros/cmd/arming', CommandBool)
    set_home = get_proxy('/mavros/cmd/set_home', CommandHome)
    takeoff = get_proxy('/mavros/cmd/takeoff', CommandTOL)
    land = get_proxy('/mavros/cmd/land', CommandTOL)

    global gps_topic
    gps_topic = find_gps_topic("takeoff")
    if gps_topic is None:
        raise Exception("NavSatFix topic not exist")

    rospy.Service('command/takeoff', TakeOff, handle_takeoff)
    rospy.Service('command/land', Land, handle_land)

    rospy.loginfo("Mission controller ready...")
    rospy.spin()


if __name__ == "__main__":
    start()
