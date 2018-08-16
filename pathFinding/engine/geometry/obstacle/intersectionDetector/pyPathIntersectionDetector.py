from engine.geometry.obstacle.intersectionDetector.pathInteresectionDetector import PathIntersectionDetector
from engine.geometry.obstacle.intersectionDetector.obstacleLineSegment import ObstacleLineSegment
import numpy as np
from utils import profile


class PyPathIntersectionDetector(PathIntersectionDetector):
    
    def __init__(self):
        self.obstacleLines = []
        self.dynamicNoFlyZones = []

    def setInitialState(self, boundaryPoints, noFlyZones):
        del self.obstacleLines[:]

        for noFlyZoneInput in noFlyZones:
            points = noFlyZoneInput.points
            for i in range(0, len(points)):
                self.obstacleLines.append(
                    ObstacleLineSegment(points[i - 1], points[i], np.array(noFlyZoneInput.velocity, np.double)))

        for i in range(len(boundaryPoints)):
            self.obstacleLines.append(
                ObstacleLineSegment(boundaryPoints[i - 1],
                                    boundaryPoints[i], np.array((0, 0), np.double)))

    def setDynamicNoFlyZones(self, dynamicNoFlyZones):
        self.dynamicNoFlyZones = dynamicNoFlyZones
    
    def testIntersections(self, lineSets):
        pass
    
    @profile.accumulate("Collision Detection")
    def testStraightPathIntersection(self, startTime, startPoint, endPoint, speed):
        for obstacleLine in self.obstacleLines:
            if obstacleLine.checkPathIntersectsLine(startTime, startPoint, endPoint, speed):
                return True
        for dnfz in self.dynamicNoFlyZones:
            if dnfz.checkPathIntersection(startTime, startPoint, endPoint, speed):
                return True
           
        return False
