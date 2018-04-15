import math
import random

from engine.geometry.noFlyZone import NoFlyZone


def loadScenarioFromJSon(fileName):
    pass


def genRandomNoFlyZones(numNoFlyZones, x, y, width, height, minFraction, maxFraction, minSpeed=0.0, maxSpeed=0.0):
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
