import random

from findPath.obstacleCourse import ObstacleCourse
from geometry.noFlyZone import NoFlyZone


def generateNoFlyZones(numNoFlyZones, x, y, width, height, minFraction, maxFraction):
    """Generates a random Geometry object with certain properties for testing"""

    xDelta = width - maxFraction * width
    yDelta = height - maxFraction * height

    minWidth = minFraction * width
    minHeight = minFraction * height
    widthDelta = (maxFraction - minFraction) * width
    heightDelta = (maxFraction - minFraction) * height

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

        zone = NoFlyZone([(x1, y1), (x1, y2), (x2, y2), (x2, y1)], (0, 0))
        noFlyZones.append(zone)

    return noFlyZones


def generateObstacleCourse(boundary, *generateNoFlyZonesArgs):
    """Generates a random obstacle course object with certain properties for testing"""
    return ObstacleCourse(boundary, generateNoFlyZones(*generateNoFlyZonesArgs))
