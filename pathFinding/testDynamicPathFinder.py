import numpy as np

import engine
import gui
from constants import MAX_VEHICLE_SPEED
from demos import DynamicPathFindingVisualizer
from engine import PointToPointInput, DynamicPathFinder

start = (95, 95)
end = (5, 5)
constantSpeed = 10.0
boundaryPoints = [(0, 0), (0, 100), (100, 100), (100, 0)]
noFlyZoneInputs = engine.utils.genRandomNoFlyZoneInputs(50, 10, 10, 80, 80, 0.01, 0.1, minSpeed=0.0, maxSpeed=4.0)
initialInput = engine.InitialPathFindingInput(boundaryPoints, noFlyZoneInputs)

pointToPointInput = PointToPointInput(start, (0, 0), [end])

# Example of loading scenario
# (initialInput, pointToPointInput) = engine.utils.loadScenario("../scenarios/test2.json")

# TODO: The most basic LinePathSegment system requires the initial velocity to be pointing in a reasonable direction
pointToPointInput.startVelocity = np.array(pointToPointInput.targetPoints[0], np.double) - np.array(
    pointToPointInput.startPosition, np.double)
pointToPointInput.startVelocity /= np.linalg.norm(pointToPointInput.startVelocity)
pointToPointInput.startVelocity *= MAX_VEHICLE_SPEED

pathFinder = DynamicPathFinder(initialInput, constantSpeed)
pathFinder.initFindPath(pointToPointInput)

bounds = initialInput.calcBounds()
centerX = (bounds[0] + bounds[2]) / 2.0
centerY = (bounds[1] + bounds[3]) / 2.0
rangeX = bounds[2] - bounds[0]
rangeY = bounds[3] - bounds[1]

# Show a slight extra buffer around the border
pathFindingVisualizer = DynamicPathFindingVisualizer(pathFinder, 800, 800, centerX, centerY, rangeX + 1,
                                                     rangeY + 1)

gui.startGUI()
