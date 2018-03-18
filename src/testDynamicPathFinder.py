import gui
# noFly1 = NoFlyZone([(40, 50), (40, 70), (50, 75), (50, 50)], (0, 0))
# noFly2 = NoFlyZone([(40, 40), (40, 45), (45, 45), (45, 40)], (0, 0))
# noFly3 = NoFlyZone([(10, 20), (30, 20), (30, 30), (10, 30)], (0, 0))
#
# pathFinder = Geometry([noFly1, noFly2, noFly3])
from demos import DynamicPathFindingVisualizer
from findPathDynamic import GridVertexQueue, UniqueVertexQueue, DynamicPathFinder
from geometry import generator

start = (95, 95)
end = (5, 5)
speed = 10.0
obstacleCourse = generator.generateObstacleCourse(None, 50, 10, 10, 80, 80, 0.01, 0.1, minSpeed=0.0, maxSpeed=4.0)
acceptanceThreshold = 2.0
numBins = 20

gridVertexQueue = GridVertexQueue(acceptanceThreshold, numBins, 0, 0, 100, 100)

# Takes uniqueness of vertices into account
uniqueVertexQueue = UniqueVertexQueue(0, 0, 100, 100, speed)

pathFinder = DynamicPathFinder(start, end, speed, obstacleCourse, uniqueVertexQueue)
pathFindingVisualizer = DynamicPathFindingVisualizer(pathFinder, 800, 800, 50, 50, 100, 100)

gui.startGUI()
