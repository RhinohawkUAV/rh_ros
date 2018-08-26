import math
import random

from engine.geometry import calcs
from engine.interface.dynamicNoFlyZone import DynamicNoFlyZone
from engine.interface.noFlyZone import NoFlyZone
import numpy as np


class Generator:

    def __init__(self, bounds, startPoint, waypoints, wayPointAvgSpeeds):
        self.bounds = bounds
        self.pathLines = []
        
        for i in range(len(waypoints)):
            distance = calcs.length(waypoints[i] - startPoint)
            self.pathLines.append((distance / wayPointAvgSpeeds[i], startPoint, waypoints[i], wayPointAvgSpeeds[i]))
            startPoint = waypoints[i]
            
        self.circularNFZs = []
        self.polyNFZs = []
    
    def setGenerationInfo(self, obstacleSize, obstacleSpeed, obstacleDistanceWaypointRatio, obstaclePerpDistance):
        self.obstacleSize = obstacleSize
        self.obstacleSpeed = obstacleSpeed
        self.obstacleDistanceWaypointRatio = obstacleDistanceWaypointRatio
        self.obstaclePerpDistance = obstaclePerpDistance
        
    def generate(self, num):
        numCircular = num
        for i in range(numCircular):
            (startTime, lineStart, lineEnd, traversalSpeed) = self.randomWaypointLine()
            if random.randint(0, 1) == 0:
                self.circularNFZs.append(self.genCircularObstacle(startTime, lineStart, lineEnd, traversalSpeed))
            else:
                self.polyNFZs.append(self.genPolyObstacle(startTime, lineStart, lineEnd, traversalSpeed))

    def randomSize(self):
        return self.obstacleSize * 0.25 + random.random() * self.obstacleSize * 0.75

    def randomWaypointLine(self):
        return self.pathLines[random.randint(0, len(self.pathLines) - 1)]
        
    def genCircularObstacle(self, startTime, lineStart, lineEnd, traversalSpeed):
        (obstacleStart, obstacleVelocity) = self.genObstacleOnLine(startTime, lineStart, lineEnd, traversalSpeed)
        return DynamicNoFlyZone(obstacleStart, self.randomSize(), obstacleVelocity)

    def genPolyObstacle(self, startTime, lineStart, lineEnd, traversalSpeed):
        (obstacleStart, obstacleVelocity) = self.genObstacleOnLine(startTime, lineStart, lineEnd, traversalSpeed)
        
        points = (obstacleStart + [-self.randomSize(), -self.randomSize()],
                  obstacleStart + [-self.randomSize(), self.randomSize()],
                  obstacleStart + [self.randomSize(), self.randomSize()],
                  obstacleStart + [self.randomSize(), -self.randomSize()])

        return NoFlyZone(points, obstacleVelocity)

    def genObstacleOnLine(self, startTime, lineStart, lineEnd, traversalSpeed):
        """
        Given a predicted start, end and traversal speed of a vehicle, create a random obstacle, along the line,
        such that when the vehicle gets to a given point, the obstacle is blocking it.
        """
        
        travelLine = lineEnd - lineStart
        (travelDir, travelDistance) = calcs.unitAndLength(travelLine)
        travelTime = travelDistance / traversalSpeed
        travelLineNormal = calcs.CCWNorm(travelDir) * self.obstaclePerpDistance
        
        # value [0,1] representing point along line to put obstacle
        lineParametric = self.obstacleDistanceWaypointRatio + random.random() * (1.0 - 2.0 * self.obstacleDistanceWaypointRatio)
        pointTime = lineParametric * travelTime + startTime
        pointAtTime = lineStart + travelLine * lineParametric + travelLineNormal * (-1.0 + random.random() * 2.0)
        
        obstacleVelocity = np.array([-1.0 + random.random() * 2.0, -1.0 + random.random() * 2.0], np.double) * self.obstacleSpeed / math.sqrt(2.0)
        obstacleStart = pointAtTime - pointTime * obstacleVelocity
        return (obstacleStart, obstacleVelocity)
