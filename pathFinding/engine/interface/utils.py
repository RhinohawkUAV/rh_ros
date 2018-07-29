import math
import random

from engine.geometry import LineSegment
from engine.interface.dynamicNoFlyZone import DynamicNoFlyZone
from engine.interface.noFlyZone import NoFlyZone
import numpy as np


def genRandomDynamicNoFlyZoneInputs(numNoFlyZones, x, y, width, height, minRadFraction, maxRadFraction, minSpeed=0.0, maxSpeed=0.0):

    """Generates a random Geometry object with certain properties for testing"""
    xDelta = width - maxRadFraction * width
    yDelta = height - maxRadFraction * height
        
    dim = math.sqrt(width * height)
    minRad = minRadFraction * dim
    radDelta = (maxRadFraction - minRadFraction) * dim

    velocityDelta = maxSpeed - minSpeed

    noFlyZones = []
    for i in range(0, numNoFlyZones):
        zoneX = x + random.random() * xDelta
        zoneY = y + random.random() * yDelta
        zoneRadius = minRad + radDelta * random.random()

        speed = minSpeed + random.random() * velocityDelta
        angle = random.random() * 2 * math.pi
        velocity = (speed * math.cos(angle), speed * math.sin(angle))

        zone = DynamicNoFlyZone((zoneX, zoneY), zoneRadius, velocity)
        noFlyZones.append(zone)

    return noFlyZones


def genRandomDynamicNoFlyZoneInputsHard(numNoFlyZones, x, y, width, height, minRadFraction, maxRadFraction, minSpeed, maxSpeed, startPoint, endPoint, averageSpeed):

    """Generates a random Geometry object with certain properties for testing"""
    travelLine = LineSegment(startPoint, endPoint)
    totalTravelTime = np.linalg.norm(travelLine.p2 - travelLine.p1) / averageSpeed
    
    xDelta = width - maxRadFraction * width
    yDelta = height - maxRadFraction * height
        
    dim = math.sqrt(width * height)
    minRad = minRadFraction * dim
    radDelta = (maxRadFraction - minRadFraction) * dim

    velocityDelta = maxSpeed - minSpeed

    noFlyZones = []
    for i in range(0, numNoFlyZones):
        zoneX = x + random.random() * xDelta
        zoneY = y + random.random() * yDelta
        zoneRadius = minRad + radDelta * random.random()

        speed = minSpeed + random.random() * velocityDelta
        angle = random.random() * 2 * math.pi
        velocity = np.array((speed * math.cos(angle), speed * math.sin(angle)))

        center = np.array((zoneX, zoneY), np.double)
        travelTime = totalTravelTime * travelLine.closestPointParametric(center)
        offset = -velocity * travelTime
    
        zone = DynamicNoFlyZone(center + offset, zoneRadius, velocity)
        noFlyZones.append(zone)

    return noFlyZones


# Deprecated, as it generates moving static (polygon) no fly zones.
def genRandomPolyNoFlyZoneInputs(numNoFlyZones, x, y, width, height, minFraction, maxFraction, minSpeed=0.0, maxSpeed=0.0):
    """Generates a random Geometry object with certain properties for testing"""

    xDelta = width - maxFraction * width
    yDelta = height - maxFraction * height

    minWidth = minFraction * width
    minHeight = minFraction * height
    widthDelta = (maxFraction - minFraction) * width
    heightDelta = (maxFraction - minFraction) * height

    velocityDelta = maxSpeed - minSpeed

    noFlyZones = []
    for i in range(0, numNoFlyZones):
        zoneX = x + random.random() * xDelta
        zoneY = y + random.random() * yDelta
        zoneWidth = minWidth + widthDelta * random.random()
        zoneHeight = minHeight + heightDelta * random.random()
        x1 = zoneX
        x2 = zoneX + zoneWidth
        y1 = zoneY
        y2 = zoneY + zoneHeight

        speed = minSpeed + random.random() * velocityDelta
        angle = random.random() * 2 * math.pi
        velocity = (speed * math.cos(angle), speed * math.sin(angle))

        zone = NoFlyZone([(x1, y1), (x1, y2), (x2, y2), (x2, y1)], velocity)
        noFlyZones.append(zone)

    return noFlyZones


# Deprecated, as it generates moving static (polygon) no fly zones.
def genRandomPolyNoFlyZoneInputsHard(numNoFlyZones, x, y, width, height, minFraction, maxFraction, minSpeed,
                                 maxSpeed, startPoint, endPoint, averageSpeed):
    """Generates a random obstacles, that are more difficult to avoid.
    Rather than simply putting NFZs in the given box with random velocities, it offsets their starting positions
    so that they will tend to be in the given box in the future.  This depends on their distance along a straight line
    path from the given start and end location and average speed.
    """

    xDelta = width - maxFraction * width
    yDelta = height - maxFraction * height

    minWidth = minFraction * width
    minHeight = minFraction * height
    widthDelta = (maxFraction - minFraction) * width
    heightDelta = (maxFraction - minFraction) * height

    velocityDelta = maxSpeed - minSpeed

    noFlyZones = []
    for i in range(0, numNoFlyZones):
        zoneX = x + random.random() * xDelta
        zoneY = y + random.random() * yDelta
        zoneWidth = minWidth + widthDelta * random.random()
        zoneHeight = minHeight + heightDelta * random.random()
        x1 = zoneX
        x2 = zoneX + zoneWidth
        y1 = zoneY
        y2 = zoneY + zoneHeight

        speed = minSpeed + random.random() * velocityDelta
        angle = random.random() * 2 * math.pi
        velocity = np.array([speed * math.cos(angle), speed * math.sin(angle)], np.double)

        travelLine = LineSegment(startPoint, endPoint)
        totalTravelTime = np.linalg.norm(travelLine.p2 - travelLine.p1) / averageSpeed
        center = np.array([zoneX + zoneWidth / 2.0, zoneY + zoneHeight / 2.0], np.double)
        travelTime = totalTravelTime * travelLine.closestPointParametric(center)
        offset = -velocity * travelTime
        x1 += offset[0]
        x2 += offset[0]
        y1 += offset[1]
        y2 += offset[1]

        zone = NoFlyZone([(x1, y1), (x1, y2), (x2, y2), (x2, y1)], (velocity[0], velocity[1]))
        noFlyZones.append(zone)

    return noFlyZones
