from gui import Drawable


class PathSegment(Drawable):

    def __init__(self, startTime, elapsedTime, endPoint, endSpeed, endUnitVelocity, nextLegalRotDirection):
        self.startTime = startTime
        self.elapsedTime = elapsedTime
        self.endPoint = endPoint
        self.endSpeed = endSpeed
        self.endUnitVelocity = endUnitVelocity
        
        # Tracks what direction in which direction the vehicle can be legally turn after this segment
        # 0 == either direction, 1 == CCW, -1 == CW
        self.nextLegalRotDirection = nextLegalRotDirection
        
        # Field for holding whatever data is useful for debugging.
        self.debug = None

    def calcPointDebug(self, point):
        """
        For visualization and debugging.  Computes the following, about the path, for a given point:

        The closest point on the path, to the given point.
        The distance to the closest point.
        The time (in the range [self.startTime,self.startTime + self.elapsedTime]) at the closest point

        :param point:
        :return: (closestPoint,distance,timeAtClosestPoint)
        """
        pass

    def testIntersection(self, pathIntersectionDetector):
        pass

    def draw(self, visualizer, **kwargs):
        pass


def calcSegmentsPointDebug(point, pathSegments, minimumDistance=float("inf")):
    """
    For a given point, find the closest point in a list of pathSegments.
    :param point: point to search against
    :param pathSegments: path segments to search for closest point
    :param minimumDistance: minimumDistance to allow (if nothing is closer than this, return None)
    :return: (indexOfClosestPathSegment,closestPointOnPathSegment,distanceToPoint,correspondingTime)
    """
    closestSegmentIndex = None
    closestPoint = None
    closestTime = 0.0

    for i in range(len(pathSegments)):
        (closestPathPoint, distance, time) = pathSegments[i].calcPointDebug(point)
        if distance < minimumDistance:
            closestSegmentIndex = i
            closestPoint = closestPathPoint
            minimumDistance = distance
            closestTime = time

    return (closestSegmentIndex, closestPoint, minimumDistance, closestTime)
