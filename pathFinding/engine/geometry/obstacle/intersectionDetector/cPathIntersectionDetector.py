from engine.geometry.obstacle.pathInteresectionDetector import PathIntersectionDetector
import fastPathIntersect


# TODO: Add cleanup/deallocation step to surounding code 
class CPathIntersectionDetector(PathIntersectionDetector):
    
    def __init__(self, bufferWidth, boundaryPoints, polyNFZs, circularNoFlyZones):
        PathIntersectionDetector.__init__(self, bufferWidth, boundaryPoints, polyNFZs, circularNoFlyZones)
        self.intersectionDetector = fastPathIntersect.createIntersectionDetector(self._lineObstacles, self._circularObstacles)

    def testStraightPathIntersection(self, startTime, startPoint, velocity, time):
        if fastPathIntersect.testIntersection(self.intersectionDetector, startTime, startPoint, velocity, time):
            return True
        return False
    
    def cleanup(self):
        pass
