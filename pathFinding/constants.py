# TODO: Tune values
# When computing _straightPaths to vertices of no-fly-zones, choosing the exact vertex may register as a collision due to
# round-off error.  This small delta offset, outward along the vertices's normal prevents this.
import math

MAX_NFZ = 32
MAX_DNFZ = 32
MAX_TRANSIT_WAYPOINTS = 8

# Meters to closest road or NFZ
TRANSIT_WAYPOINT_BUFFER = 200

# Closest distance (meters) an NFZ can appear relative to the aircraft
MIN_DNFZ_SPAWN_DISTANCE = 500

# 26.8m/s =  60.0 mph
# 44.7m/s = 100.0 mph
DEFAULT_MAX_VEHICLE_SPEED = 44.7 

DEFAULT_TURN_ACCELERATION = 8.0
DEFAULT_WAYPOINT_ACCEPTANCE_RADII = 120.0
DEFAULT_NFZ_BUFFER_WIDTH = 25.0
DEFAULT_NFZ_TARGET_OFFSET = 50.0
DEFAULT_VERTEX_HEURISTIC_MULTIPLIER = 1.25
DEFAULT_MAXIMUM_TIME = 2000.0

# When making a buffer around an NFZ the buffer's edges will be a configurable width away from the original buffer.
# Vertices will be further away to create the desired width and very sharp vertice may be pushed out a lot.  This is maximum
# ratio vertices may be pushed out, before a new vertex is created.
NFZ_MAX_NEW_VERTEX_EXPANSION_RATIO = 2.0

CANBERRA_GPS = (-35.2809, 149.1300)

# A good worst case would be if start an end were at diagonals of a square 
# and the path were a completely straight line.  If the boundaries were just barely surrounding
# this square, then the diagonal would be ~11km (6 nautical miles).  The edges of the square would be 11km/sqrt(2)
COURSE_DIM = 11000.0 / math.sqrt(2.0)

# Don't consider arcs which seem too counter productive
MAX_ARC_LENGTH = math.radians(210.0)

# Its easy to end up in a situation where another point in the path can be found right on top of the current point.
# This is not productive in most cases and can significantly slow down the path finding process.
# Increasing this number will make the path finder miss cases where it could "just" squeeze through a tight spot.
IDENTICAL_POINT_TOLERANCE = 1.0

# This is used to determine if 2 points are identical (is the distance squared between them less than this)
DISTANCE_TOLERANCE_SQUARED = IDENTICAL_POINT_TOLERANCE * IDENTICAL_POINT_TOLERANCE

MAX_ARC_INTERPOLATION_ERROR = 20.0

UNIQUE_TREE_COINCIDENT_DISTANCE = 0.0005

