from engine.geometry.obstacle.pathInteresectionDetector import PathIntersectionDetector


class PyPathIntersectionDetector(PathIntersectionDetector):
    
    def __init__(self, bufferWidth, boundaryPoints, polyNFZs, circularNoFlyZones):
        PathIntersectionDetector.__init__(self, bufferWidth, boundaryPoints, polyNFZs, circularNoFlyZones)
        
    def testStraightPathIntersection(self, startTime, startPoint, velocity, time):
        for obstacleLine in self._lineObstacles:
            if obstacleLine.checkPathIntersectsLine(startTime, startPoint, velocity, time):
                return True
        for circularObstacle in self._circularObstacles:
            if circularObstacle.checkPathIntersection(startTime, startPoint, velocity, time):
                return True
           
        return False