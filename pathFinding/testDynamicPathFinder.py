import numpy as np

import engine
import gui
from demos import DynamicPathFindingVisualizer
from engine import DynamicPathFinder
from engine.geometry import calcs
from engine.interface import NoFlyZoneInput
from engine.interface import PointToPointInput

test = NoFlyZoneInput([], [])
start = (95, 95)
end = (5, 5)
maxSpeed = 5.0
startVelocity = calcs.unit(np.array([-1.0, -1.0], np.double)) * maxSpeed
startVelocity = (startVelocity[0], startVelocity[1])
boundaryPoints = [(0, 0), (0, 100), (100, 100), (100, 0)]

noFlyZoneInputs = engine.utils.genRandomNoFlyZoneInputsHard(50, 10, 10, 80, 80, 0.01, 0.1, minSpeed=0.0, maxSpeed=2.0,
                                                            startPoint=start, endPoint=end, averageSpeed=maxSpeed)

obstacleInput = engine.InitialPathFindingInput(boundaryPoints, noFlyZoneInputs)

pathInput = PointToPointInput(start, startVelocity, [end])

# Example of loading scenario
# scenario = engine.utils.loadScenario("../scenarios/arc1.json")
# obstacleInput = scenario[engine.utils.OBSTACLE_INPUT_KEY]
# pathInput = scenario[engine.utils.PATH_INPUT_KEY]

pathFinder = DynamicPathFinder(obstacleInput, maxSpeed)
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
