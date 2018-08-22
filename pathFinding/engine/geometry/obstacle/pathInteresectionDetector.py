from gui.core import Drawable


class PathIntersectionDetector(Drawable):

    def testStraightPathIntersections(self, points, times):
        return False

    def testStraightPathIntersection(self, startTime, startPoint, velocity, time):
        return False
