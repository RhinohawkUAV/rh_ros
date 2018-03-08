from shapely.geometry import LineString

from gui import Drawable


class ObstacleCourse(Drawable):
    """
    Defines the no fly zones and boundary of the problem.  Allows queries such as visibility from a point and
    collision detection to be performed.
    """

    def __init__(self, noFlyZones, boundary):
        self._noFlyZones = noFlyZones
        self.boundary = boundary

    def doesLineIntersectStatic(self, startPoint, endPoint):
        """
        Does the line from startPoint to endPoint intersect and NFZs?
        DOES NOT account for motion of NFZs over time.
        """
        line = LineString([startPoint, endPoint])
        for noFlyZone in self._noFlyZones:
            if noFlyZone.blocksLineOfSight(line):
                return False
        return True

    def findVisibleVerticesStatic(self, eye):
        """
        Look through all NFZ vertices and determine which are visible from eye.
        DOES NOT account for motion of NFZs over time.
        """
        visibleVertices = []
        for noFlyZone in self._noFlyZones:
            for polygonVertex in noFlyZone.points:
                if self.doesLineIntersectStatic(eye, polygonVertex):
                    visibleVertices.append(polygonVertex)

        return visibleVertices

    def findVisibleVerticesDynamic(self, startPoint, speed):
        """
        Look through all NFZ vertices and determine which are visible from eye.
        DOES account for motion of NFZs over time.
        TODO: Does not account for intersections with NFZs over time yet...
        """
        visiblePoints = []
        for noFlyZone in self._noFlyZones:
            results = noFlyZone.findFutureHeadingCollisions(startPoint, speed)
            for result in results:
                point = result[1]
                visiblePoints.append(point)
        return visiblePoints

    def draw(self, canvas, time=0.0, **kwargs):
        for noFlyZone in self._noFlyZones:
            noFlyZone.draw(canvas, time=time, **kwargs)
