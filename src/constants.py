# TODO: Tune values
# When computing _nextEdges to vertices of no-fly-zones, choosing the exact vertex may register as a collision due to
# round-off error.  This small delta offset, outward along the vertices's normal prevents this.
import math

NO_FLY_ZONE_POINT_OFFSET = 0.0001

# TODO: Used with the assumption that speed is constant.  This probably isn't going to be true ultimately
MAX_TURN_ANGLE = 90

# For calculation convenience
MAX_TURN_ANGLE_COS = math.cos(math.radians(MAX_TURN_ANGLE))

# This is used to determine if 2 points are identical (is the distance squared between them less than this)
DISTANCE_TOLERANCE_SQUARED = NO_FLY_ZONE_POINT_OFFSET * NO_FLY_ZONE_POINT_OFFSET
