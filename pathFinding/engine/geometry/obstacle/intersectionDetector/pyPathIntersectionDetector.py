from engine.geometry import calcs
from engine.geometry.obstacle.intersectionDetector.circularObstacle import CircularObstacle
from engine.geometry.obstacle.intersectionDetector.lineSegmentObstacle import LineSegmentObstacle
from engine.geometry.obstacle.pathInteresectionDetector import PathIntersectionDetector
import numpy as np
from utils import profile


class PyPathIntersectionDetector(PathIntersectionDetector):
    
    def __init__(self, params, vehicle):
        PathIntersectionDetector.__init__(self, params, vehicle)
        self.obstacleLines = []
        self.circularObstacles = []

    def setState(self, boundaryPoints, polyNFZs, circularNoFlyZones):
        self.obstacleLines = []
        self.circularObstacles = []
        self.createObstacleLines(boundaryPoints, np.array((0, 0), np.double))
        for noFlyZone in polyNFZs:
            self.createObstacleLines(noFlyZone.points, noFlyZone.velocity)
        
        self.circularObstacles = list(map(lambda c: 
                                          CircularObstacle(c.center, c.radius + self.params.nfzBufferWidth, c.velocity),
                                          circularNoFlyZones))
        
    def createObstacleLines(self, points, velocity):
        shell = calcs.calcShell(points, self.params.nfzBufferWidth)
        for i in range(len(shell)):
            self.obstacleLines.append(
                LineSegmentObstacle(shell[i - 1], shell[i], velocity))
    
    def testStraightPathIntersections(self, points, times):
        for i in range(0, len(points) - 1):
            time = times[i + 1] - times[i]
            
            if self.testStraightPathIntersection(startTime=times[i],
                                                startPoint=points[i],
                                                velocity=(points[i + 1] - points[i]) / time,
                                                time=time):
                return True
        return False 
        
    @profile.accumulate("Collision Detection")
    def testStraightPathIntersection(self, startTime, startPoint, velocity, time):
        for obstacleLine in self.obstacleLines:
            if obstacleLine.checkPathIntersectsLine(startTime, startPoint, velocity, time):
                return True
        for circularObstacle in self.circularObstacles:
            if circularObstacle.checkPathIntersection(startTime, startPoint, velocity, time):
                return True
           
        return False
    
    def draw(self, canvas, time=0.0, **kwargs):
        for obstacleLine in self.obstacleLines:
            obstacleLine.draw(canvas, time=time, **kwargs)
        for circularObstacle in self.circularObstacles:
            circularObstacle.draw(canvas, time=time, **kwargs)
