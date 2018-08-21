from engine.geometry import calcs
from engine.geometry.obstacle.intersectionDetector.circularObstacle import CircularObstacle
from engine.geometry.obstacle.intersectionDetector.lineSegmentObstacle import LineSegmentObstacle
from engine.geometry.obstacle.intersectionDetector.pathInteresectionDetector import PathIntersectionDetector
import numpy as np
from utils import profile


class PyPathIntersectionDetector(PathIntersectionDetector):
    
    def __init__(self, bufferWidth):
        self.bufferWidth = bufferWidth
        self.obstacleLines = []
        self.circularObstacles = []

    def setInitialState(self, boundaryPoints, noFlyZones):
        del self.obstacleLines[:]

        for noFlyZone in noFlyZones:
            self.createObstacleLines(noFlyZone.points, noFlyZone.velocity)
            
        self.createObstacleLines(boundaryPoints, np.array((0, 0), np.double))

    def createObstacleLines(self, points, velocity):
        shell = calcs.calcShell(points, self.bufferWidth)
        for i in range(len(shell)):
            self.obstacleLines.append(
                LineSegmentObstacle(shell[i - 1], shell[i], velocity))

    def setDynamicNoFlyZones(self, dynamicNoFlyZones):
        self.circularObstacles = list(map(lambda d: CircularObstacle(d.center, d.radius + self.bufferWidth, d.velocity), dynamicNoFlyZones))
    
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
