from gui import Drawable


class ObstacleCourse(Drawable):
    """
    Defines the no fly zones and boundary of the problem.  Allows queries such as visibility from a point and
    collision detection to be performed.
    """

    def __init__(self, boundary, noFlyZones):
        self.boundary = boundary
        self._noFlyZones = noFlyZones

    def doesLineIntersect(self, startPoint, endPoint, speed):
        """
        Does a path from startPoint to endPoint, at the given speed intersect any NFZs at any time?
        """
        for noFlyZone in self._noFlyZones:
            if noFlyZone.checkBlocksPath(startPoint, endPoint, speed):
                return True
        return False

    def findPathsToVertices(self, startPoint, speed):
        """
        Look through all NFZ vertices and determine which can be reached by a straight-line path from
        startPoint at the given speed without intersecting any NFZs at any time.

        :param startPoint:
        :param speed:
        :return: [(velocity,pathEndPoint),(velocity2,pathEndPoint),...]
        """
        paths = []
        for noFlyZone in self._noFlyZones:
            NFZPaths = noFlyZone.calcVelocitiesToVertices(startPoint, speed)
            for path in NFZPaths:
                pathEndPoint = path[1]
                if not self.doesLineIntersect(startPoint, pathEndPoint, speed):
                    paths.append(path)
        return paths

    def draw(self, canvas, time=0.0, **kwargs):
        for noFlyZone in self._noFlyZones:
            noFlyZone.draw(canvas, time=time, **kwargs)
