import findPath
import gui
from demos import DynamicPathFindingVisualizer
from findPath import PointToPointInput, DynamicPathFinder

start = (95, 95)
initialVelocity = (0, 0)
end = (5, 5)
constantSpeed = 10.0
noFlyZones = findPath.randomInput.genRandomNoFlyZones(50, 10, 10, 80, 80, 0.01, 0.1, minSpeed=0.0, maxSpeed=4.0)
initialInput = findPath.InitialPathFindingInput([], noFlyZones)

acceptanceThreshold = 2.0
numBins = 20

pathFinder = DynamicPathFinder(initialInput, constantSpeed)
pathFinder.initFindPath(PointToPointInput(start, initialVelocity, [end]))

pathFindingVisualizer = DynamicPathFindingVisualizer(pathFinder, 800, 800, 50, 50, 100, 100)

gui.startGUI()
