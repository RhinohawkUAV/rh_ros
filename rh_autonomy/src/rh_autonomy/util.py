# Utility functions
import math
import rospy
import traceback
from rh_autonomy import constants as rhc
from rh_msgs.msg import GPSCoord

GPS_ERROR = 0.0002

def logexc(msg):
    rospy.logerr('%s:\n%s' % (msg, ''.join(traceback.format_stack())))


def get_proxy(topic, serviceType):
    """ Waits for the given service to be ready, then returns a ServiceProxy
    """
    rospy.loginfo("Waiting for service: %s", topic)
    rospy.wait_for_service(topic)
    return rospy.ServiceProxy(topic, serviceType)

def coords_to_str(coords):
    """ Takes a list of GPSCoord and returns a pretty printed string
    """
    i,s = 0,""
    for coord in coords:
        if i>0: s += "\n"
        s += "%d - coord(%2.6f,%2.6f,%2.6f)" % \
                (i,coord.lat,coord.lon,coord.alt)
        i += 1
    return s


def waypoints_to_str(wps):
    """ Takes a list of Mavros waypoints and returns a pretty printed string
    """
    i,s = 0,""
    for wp in wps:

        if wp.command == rhc.MAV_CMD_LAND:
            cmd = "    land"
        elif wp.command == rhc.MAV_CMD_TAKEOFF:
            cmd = " takeoff"
        elif wp.command == rhc.MAV_CMD_RTL:
            cmd = "     RTL"
        else:
            cmd = "waypoint"

        msg = " - dummy" if i==0 else ""
        if wp.is_current: msg += " (current)"

        if i>0: s += "\n"
        s += "%d - %s(%2.6f,%2.6f,%2.6f) - (%2.2f,%2.2f)%s" % \
                (i,cmd,wp.x_lat,wp.y_long,wp.z_alt,wp.param1,wp.param2,msg)
        
        i += 1
    
    return s


def gps_dist(g1, g2):
    """ Distance in meters between two GPS locations
    """
    # TODO: this is a placeholder implementation
    return math.sqrt(pow(g2.lat-g1.lat,2) + pow(g2.lon-g1.lon,2))


def get_current(wps):
    """ Returns only waypoints after and including the current waypoint
    """
    cwps = []
    include = False
    for wp in wps:
        if wp.is_current:
            include = True
        if include:
            cwps.append(wp)
    return cwps


def wp2coord(wp):
    return GPSCoord(wp.x_lat, wp.y_long, wp.z_alt)


def wplist2coords(wplist):
    return [wp2coord(wp) for wp in wplist]


def wp_lists_equal(wps1, wps2, approx=True):
    cwps1 = get_current(wps1)
    cwps2 = get_current(wps2)
    return coord_lists_equal(wplist2coords(cwps1), \
            wplist2coords(cwps2), approx=approx)


def coord_lists_equal(wps1, wps2, approx=True):
    """ Returns true if the given waypoint lists are equal.
        If approx is True, then the waypoints need only be 
        within a margin of GPS error of each other.
    """
    if len(wps1)!=len(wps2): 
        rospy.logwarn("Lists lengths dont match (%d!=%d)"\
                %(len(wps1),len(wps2)))
        return False
    for wp1,wp2 in zip(wps1,wps2):
        if not coords_equal(wp1,wp2,approx=approx): return False
    return True


def coords_equal(wp1, wp2, approx=True):
    """ Returns true if the given waypoints as equal.
        If approx is True, then the waypoints need only be 
        within a margin of GPS error of each other.
    """
    if approx:
        d = gps_dist(wp1, wp2)
        if d >= GPS_ERROR:
            rospy.loginfo("Lists are not the same because WPs are %2.6f apart"%d)
        return d < GPS_ERROR
    else:
        return wp1.lat==wp2.lat and wp1.lon==wp2.lon and wp1.alt==wp2.alt

