from engine.geometry import calcs
from engine.geometry.obstacle.intersectionDetector.circularObstacle import CircularObstacle
from engine.geometry.obstacle.intersectionDetector.lineSegmentObstacle import LineSegmentObstacle
from gui.core import Drawable
import numpy as np
from utils import profile


class PathIntersectionDetector(Drawable):

    def __init__(self, bufferWidth, boundaryPoints, polyNFZs, circularNoFlyZones):
        self._bufferWidth = bufferWidth
        self._lineObstacles = []
        self._circularObstacles = []
        self._createObstacleLines(boundaryPoints, np.array((0, 0), np.double))
        for noFlyZone in polyNFZs:
            self._createObstacleLines(noFlyZone.points, noFlyZone.velocity)
        
        self._circularObstacles = list(map(lambda c: 
                                          CircularObstacle(c.center, c.radius + self._bufferWidth, c.velocity),
                                          circularNoFlyZones))

    def _createObstacleLines(self, points, velocity):
        shell = calcs.calcShell(points, self._bufferWidth)
        for i in range(len(shell)):
            self._lineObstacles.append(
                LineSegmentObstacle(shell[i - 1], shell[i], velocity))        

    @profile.accumulate("Collision Detection")
    def testStraightPathIntersections(self, points, times):
        for i in range(0, len(points) - 1):
            time = times[i + 1] - times[i]
            
            if self.testStraightPathIntersection(startTime=times[i],
                                                startPoint=points[i],
                                                velocity=(points[i + 1] - points[i]) / time,
                                                time=time):
                return True
        return False 

    def testStraightPathIntersection(self, startTime, startPoint, velocity, time):
        return False

    def draw(self, canvas, time=0.0, **kwargs):
        for obstacleLine in self._lineObstacles:
            obstacleLine.draw(canvas, time=time, **kwargs)
        for circularObstacle in self._circularObstacles:
            circularObstacle.draw(canvas, time=time, **kwargs)
