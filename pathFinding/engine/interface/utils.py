import json
import math
import random

import numpy as np

from engine.interface import initialPathFindingInput, pointToPointInput
from noFlyZoneInput import NoFlyZoneInput

OBSTACLE_INPUT_KEY = "obstacleInput"
PATH_INPUT_KEY = "pathInput"


class Encoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif callable(getattr(obj, "toJSONDict", None)):
            return obj.toJSONDict()
        else:
            return obj.__dict__


def saveScenario(fileName, obstacleCourseEdit, pathEdit):
    file = open(fileName, 'w')
    output = {}
    output[OBSTACLE_INPUT_KEY] = obstacleCourseEdit
    output[PATH_INPUT_KEY] = pathEdit
    json.dump(output, file, cls=Encoder, indent=4)
    file.close()


def loadScenario(fileName):
    file = open(fileName, 'r')
    scenarioDict = json.load(file)
    file.close()

    pathInput = None
    obstacleInput = None

    if scenarioDict.has_key(OBSTACLE_INPUT_KEY) and scenarioDict[OBSTACLE_INPUT_KEY] is not None:
        obstacleInput = initialPathFindingInput.fromJSONDict(scenarioDict[OBSTACLE_INPUT_KEY])

    if scenarioDict.has_key(PATH_INPUT_KEY) and scenarioDict[PATH_INPUT_KEY] is not None:
        pathInput = pointToPointInput.fromJSONDict(scenarioDict[PATH_INPUT_KEY])

    return (obstacleInput, pathInput)


def genRandomNoFlyZoneInputs(numNoFlyZones, x, y, width, height, minFraction, maxFraction, minSpeed=0.0, maxSpeed=0.0):
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

        zone = NoFlyZoneInput([(x1, y1), (x1, y2), (x2, y2), (x2, y1)], velocity)
        noFlyZones.append(zone)

    return noFlyZones
