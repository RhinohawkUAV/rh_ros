import fastPathIntersect

from engine.geometry.obstacle.pathInteresectionDetector import PathIntersectionDetector


class CPathIntersectionDetector(PathIntersectionDetector):
    
    def __init__(self, bufferWidth, boundaryPoints, polyNFZs, circularNoFlyZones):
        PathIntersectionDetector.__init__(self, bufferWidth, boundaryPoints, polyNFZs, circularNoFlyZones)
        self.intersectionDetector = fastPathIntersect.createIntersectionDetector(self._lineObstacles, self._circularObstacles)

    def testStraightPathIntersection(self, startTime, startPoint, velocity, time):
        if fastPathIntersect.testIntersection(self.intersectionDetector, startTime, startPoint, velocity, time):
            return True
        return False
    
    def destroy(self):
        """
        Deallocates memory.  CANNOT be used after this is called!
        """
        fastPathIntersect.destroyIntersectionDetector(self.intersectionDetector)
