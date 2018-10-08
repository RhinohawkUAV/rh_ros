import math
import random

from engine.geometry import calcs
from engine.interface.dynamicNoFlyZone import DynamicNoFlyZone
from engine.interface.noFlyZone import NoFlyZone
import numpy as np


def randomPositionOnLine(traversalStartTime, lineStart, lineEnd, lineTraversalSpeed, maxObstacleSpeed, maxPerpDistance=0.0):
    """
    Given a predicted start, end and traversal speed of a vehicle, create a random obstacle, along the line,
    such that when the vehicle gets to a given point, the obstacle is blocking it.
    """
    
    travelLine = lineEnd - lineStart
    (travelDir, travelDistance) = calcs.unitAndLength(travelLine)
    travelTime = travelDistance / lineTraversalSpeed
    
    lineParametric = random.random()
    point = lineStart + lineParametric * travelLine + maxPerpDistance * (random.random() * 2.0 - 1.0)
    time = traversalStartTime + travelTime * lineParametric
    return (point, time)


def randomVelocity(minSpeed, maxSpeed):
    angle = 2 * math.pi * random.random()
    x = math.cos(angle)
    y = math.sin(angle)
    speed = minSpeed + random.random() * (maxSpeed - minSpeed)
    return np.array((x * speed, y * speed), np.double)


def calcStart(time, point, velocity):
    return (point - time * velocity, velocity)


class NFZGenerator:

    def __init__(self, maxSize):
        self._maxSize = maxSize

    def randomSize(self):
        return self._maxSize * (0.25 + random.random() * 0.75)
            
    def genCircularObstacle(self, position, velocity):
        return DynamicNoFlyZone(position, self.randomSize(), velocity)

    def genPolyObstacle(self, position, velocity):
        
        points = (position + [-self.randomSize(), -self.randomSize()],
                  position + [-self.randomSize(), self.randomSize()],
                  position + [self.randomSize(), self.randomSize()],
                  position + [self.randomSize(), -self.randomSize()])

        return NoFlyZone(points, velocity)


class ObstacleGenerator:

    def __init__(self, params, scenario, vehicle):
        self.params = params
        self.scenario = scenario
        self.vehicle = vehicle
        self.pathLines = []
        startPoint = self.scenario.startPoint
        waypoints = self.scenario.wayPoints
        self.avgSpeed = calcs.length(self.scenario.startVelocity)
        for i in range(len(waypoints)):
            distance = calcs.length(waypoints[i] - startPoint)
            self.pathLines.append((distance / self.avgSpeed, startPoint, waypoints[i]))
            startPoint = waypoints[i]
    
    def setGenerationInfo(self, maxObstacleSize, maxObstacleSpeedRatio, obstacleDistanceWaypointRatio):
        self.nfzGenerator = NFZGenerator(maxObstacleSize)
        self.maxObstacleSpeedRatio = maxObstacleSpeedRatio
        self.obstacleDistanceWaypointRatio = obstacleDistanceWaypointRatio
        
    def blockEstimatedPath(self, num):
        for i in range(num):
            (startTime, lineStart, lineEnd) = self.randomWaypointLine()
            (position, velocity) = self.randomPositionOnLine(startTime, lineStart, lineEnd)
            self.appendRandomObstacle(position, velocity)
            
    def appendRandomObstacle(self, position, velocity):
        if random.randint(0, 1) == 0:
            self.scenario.dynamicNoFlyZones.append(self.nfzGenerator.genCircularObstacle(position, velocity))
        else:
            self.scenario.noFlyZones.append(self.nfzGenerator.genPolyObstacle(position, velocity))

    def randomWaypointLine(self):
        return self.pathLines[random.randint(0, len(self.pathLines) - 1)]

    def block(self, pathSegments):
        if len(pathSegments) == 0:
            return
        pathSegmentIndex = random.randint(0, len(pathSegments) - 1)
        (point, velocity) = self.randomPositionOnPathSegment(pathSegments[pathSegmentIndex])
        while not self.isPointLegal(point):
            (point, velocity) = self.randomPositionOnPathSegment(pathSegments[pathSegmentIndex])
            
        self.appendRandomObstacle(point, velocity)

    def isPointLegal(self, point):
        return calcs.length(point - self.scenario.startPoint) * 1.2 > self.nfzGenerator._maxSize
            
    def randomPositionOnPathSegment(self, arcPathSegment):
        point = arcPathSegment.getPoint(random.random())
        (closestPoint, distance, time) = arcPathSegment.calcPointDebug(point)
        velocity = self.randomVelocity()
        return calcStart(time, point, velocity)

    def randomPositionOnLine(self, traversalStartTime, lineStart, lineEnd):
        """
        Given a predicted start, end and traversal speed of a vehicle, create a random obstacle, along the line,
        such that when the vehicle gets to a given point, the obstacle is blocking it.
        """
        
        travelLine = lineEnd - lineStart

        (point, time) = randomPositionOnLine(traversalStartTime,
                                     lineStart + travelLine * self.obstacleDistanceWaypointRatio,
                                     lineEnd - travelLine * self.obstacleDistanceWaypointRatio,
                                     self.avgSpeed,
                                     self.nfzGenerator._maxSize / 2.0)
        
        velocity = randomVelocity(self.avgSpeed * self.maxObstacleSpeedRatio * 0.25, self.avgSpeed * self.maxObstacleSpeedRatio)
        
        return calcStart(time, point, velocity)

    def randomVelocity(self):
        return randomVelocity(self.avgSpeed * self.maxObstacleSpeedRatio * 0.25, self.avgSpeed * self.maxObstacleSpeedRatio)

