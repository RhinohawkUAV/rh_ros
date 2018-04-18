from gui import Drawable


class PathSegment(Drawable):
    def __init__(self, time, endPoint, endVelocity):
        self.time = time
        self.endPoint = endPoint
        self.endVelocity = endVelocity

    def calcPointDebug(self, point):
        """
        For visualization and debugging.  Computes the following, about the path, for a given point:

        The closest point on the path, to the given point.
        The distance to the closest point.
        The time (in the range [0,self.time]) at the closest point

        :param point:
        :return: (closestPoint,distance,timeAtClosestPoint)
        """
        pass

    def draw(self, canvas, **kwargs):
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
