import numpy as np

import engine
import gui
from constants import MAX_VEHICLE_SPEED
from demos import DynamicPathFindingVisualizer
from engine import DynamicPathFinder
from engine.interface import NoFlyZoneInput
from engine.interface import PointToPointInput

test = NoFlyZoneInput([], [])
start = (95, 95)
end = (5, 5)
constantSpeed = 10.0
boundaryPoints = [(0, 0), (0, 100), (100, 100), (100, 0)]
noFlyZoneInputs = engine.utils.genRandomNoFlyZoneInputs(50, 10, 10, 80, 80, 0.01, 0.1, minSpeed=0.0, maxSpeed=4.0)
obstacleInput = engine.InitialPathFindingInput(boundaryPoints, noFlyZoneInputs)

pathInput = PointToPointInput(start, (0, 0), [end])

# Example of loading scenario
# scenario = engine.utils.loadScenario("../scenarios/arc1.json")
# obstacleInput = scenario[engine.utils.OBSTACLE_INPUT_KEY]
# pathInput = scenario[engine.utils.PATH_INPUT_KEY]

# TODO: The most basic LinePathSegment system requires the initial velocity to be pointing in a reasonable direction
pathInput.startVelocity = np.array(pathInput.targetPoints[0], np.double) - np.array(
    pathInput.startPosition, np.double)
pathInput.startVelocity /= np.linalg.norm(pathInput.startVelocity)
pathInput.startVelocity *= MAX_VEHICLE_SPEED

pathFinder = DynamicPathFinder(obstacleInput, constantSpeed)
pathFinder.initFindPath(pathInput)

bounds = obstacleInput.calcBounds()
centerX = (bounds[0] + bounds[2]) / 2.0
centerY = (bounds[1] + bounds[3]) / 2.0
rangeX = bounds[2] - bounds[0]
rangeY = bounds[3] - bounds[1]

# Show a slight extra buffer around the border
pathFindingVisualizer = DynamicPathFindingVisualizer(pathFinder, 800, 800, centerX, centerY, rangeX + 1,
                                                     rangeY + 1)

gui.startGUI()
