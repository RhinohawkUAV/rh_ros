import findPath
import gui
from demos import DynamicPathFindingVisualizer
from findPath import PointToPointInput, DynamicPathFinder

start = (95, 95)
initialVelocity = (0, 0)
end = (5, 5)
constantSpeed = 10.0
boundaryPoints = [(0, 0), (0, 100), (100, 100), (100, 0)]
noFlyZones = findPath.randomInput.genRandomNoFlyZones(50, 10, 10, 80, 80, 0.01, 0.1, minSpeed=0.0, maxSpeed=4.0)
initialInput = findPath.InitialPathFindingInput(boundaryPoints, noFlyZones)

acceptanceThreshold = 2.0
numBins = 20

pathFinder = DynamicPathFinder(initialInput, constantSpeed)
pathFinder.initFindPath(PointToPointInput(start, initialVelocity, [end]))

bounds = initialInput.calcBounds()
centerX = (bounds[0] + bounds[2]) / 2.0
centerY = (bounds[1] + bounds[3]) / 2.0
rangeX = bounds[2] - bounds[0]
rangeY = bounds[3] - bounds[1]

# Show a slight extra buffer around the border
pathFindingVisualizer = DynamicPathFindingVisualizer(pathFinder, 800, 800, centerX, centerY, rangeX + 1,
                                                     rangeY + 1)

gui.startGUI()
