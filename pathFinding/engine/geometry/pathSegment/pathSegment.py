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
        A parametric describing the passage of time at that point ie.
        timeAtClosestPoint = parametric * startTime + (1-parametric) * endPoint

        :param point:
        :return: (closestPoint,distance,timeParametric)
        """
        pass

    def draw(self, canvas, **kwargs):
        pass
