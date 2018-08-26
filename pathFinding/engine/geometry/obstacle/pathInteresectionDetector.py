from gui.core import Drawable


class PathIntersectionDetector(Drawable):

    def __init__(self, params, vehicle):
        self.params = params
        self.vehicle = vehicle
        
    def testStraightPathIntersections(self, points, times):
        return False

    def testStraightPathIntersection(self, startTime, startPoint, velocity, time):
        return False
