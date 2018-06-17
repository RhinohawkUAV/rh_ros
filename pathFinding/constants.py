# TODO: Tune values
# When computing _straightPaths to vertices of no-fly-zones, choosing the exact vertex may register as a collision due to
# round-off error.  This small delta offset, outward along the vertices's normal prevents this.
import math

MAX_VEHICLE_SPEED = 10.0
TURN_ACCELERATION = 1.0

NO_FLY_ZONE_POINT_OFFSET = 0.0001

# This is used to determine if 2 points are identical (is the distance squared between them less than this)
DISTANCE_TOLERANCE_SQUARED = NO_FLY_ZONE_POINT_OFFSET * NO_FLY_ZONE_POINT_OFFSET

CANBERRA_GPS = (-35.2809, 149.1300)
