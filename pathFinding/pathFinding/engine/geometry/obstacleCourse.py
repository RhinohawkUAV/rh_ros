from gui import Drawable, draw


class ObstacleCourse(Drawable):
    """
    Defines the no fly zones and _boundary of the problem.  Allows queries such as visibility from a point and
    collision detection to be performed.
    """

    def __init__(self, boundaryPoints, noFlyZones):
        self._boundaryPoints = boundaryPoints
        self._noFlyZones = noFlyZones

    def getFutureCopy(self, time):
        """Create a copy of this obstacle course, as it will exist at some point in the future."""

        noFlyZonesCopy = []
        for noFlyZone in self._noFlyZones:
            noFlyZonesCopy.append(noFlyZone.getFutureCopy(time))
        return ObstacleCourse(self._boundaryPoints, noFlyZonesCopy)

    def doesLineIntersect(self, startPoint, endPoint, speed):
        """
        Does a path from startPoint to endPoint, at the given speed intersect any NFZs at any time?
        """
        for noFlyZone in self._noFlyZones:
            if noFlyZone.checkBlocksPath(startPoint, endPoint, speed):
                return True
        return False

    def findStraightPathsToVertices(self, startPoint, speed, predicateFilter=(lambda path: True)):
        """
        Look through all NFZ vertices and determine which can be reached by a straight-line path from
        startPoint at the given speed without intersecting any NFZs.

        :param startPoint:
        :param speed:
        :param predicateFilter: almost all calc time is spent checking if a path intersects with any NFZ over time.
        This predicate is run BEFORE this calculation is made giving an opportunity to eliminate a path.
        :return: [StraightPathSolution1, StraightPathSolution2, ...]
        """
        paths = []
        for noFlyZone in self._noFlyZones:
            NFZPaths = noFlyZone.calcVelocitiesToVertices(startPoint, speed)

            for path in NFZPaths:
                if predicateFilter(path):
                    pathEndPoint = path.destination
                    if not self.doesLineIntersect(startPoint, pathEndPoint, speed):
                        paths.append(path)
        return paths

    def draw(self, canvas, time=0.0, boundaryColor="red", **kwargs):
        for i in range(len(self._boundaryPoints)):
            draw.drawLine(canvas, self._boundaryPoints[i - 1], self._boundaryPoints[i], color=boundaryColor)
        for noFlyZone in self._noFlyZones:
            noFlyZone.draw(canvas, time=time, **kwargs)
