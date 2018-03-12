from gui import Drawable


class ObstacleCourse(Drawable):
    """
    Defines the no fly zones and boundary of the problem.  Allows queries such as visibility from a point and
    collision detection to be performed.
    """

    def __init__(self, noFlyZones, boundary):
        self._noFlyZones = noFlyZones
        self.boundary = boundary

    def doesLineIntersect(self, startPoint, endPoint, speed):
        """
        Does a path from startPoint to endPoint, at the given speed intersect any NFZs at any time?
        """
        for noFlyZone in self._noFlyZones:
            if noFlyZone.checkBlocksPath(startPoint, endPoint, speed):
                return True
        return False

    def findVisibleVertices(self, startPoint, speed):
        """
        Look through all NFZ vertices and determine which are visible from startPoint.
        DOES NOT account for motion of NFZs over time.
        """
        visibleVertices = []
        for noFlyZone in self._noFlyZones:
            for polygonVertex in noFlyZone.points:
                if not self.doesLineIntersect(startPoint, polygonVertex, speed):
                    visibleVertices.append(polygonVertex)

        return visibleVertices

    def findVisibleVerticesDynamic(self, startPoint, speed):
        """
        Look through all NFZ vertices and determine which can be reached by a straight-line path from
        startPoint to the vertex at the given speed without intersecting any NFZs.

        """
        visiblePoints = []
        for noFlyZone in self._noFlyZones:
            results = noFlyZone.calcVelocitiesToVertices(startPoint, speed)
            for result in results:
                endPoint = result[1]
                if not self.doesLineIntersect(startPoint, endPoint, speed):
                    visiblePoints.append(endPoint)
        return visiblePoints

    def draw(self, canvas, time=0.0, **kwargs):
        for noFlyZone in self._noFlyZones:
            noFlyZone.draw(canvas, time=time, **kwargs)
