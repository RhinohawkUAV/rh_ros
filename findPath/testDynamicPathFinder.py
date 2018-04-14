import findPath
import gui
from demos import DynamicPathFindingVisualizer
from findPath import DynamicPathFinder
# TODO: Ultimately, this should be automatically be created underneath.  This was useful for swapping out with legacy GridVertexQueue
from findPath.vertex import UniqueVertexQueue

start = (95, 95)
end = (5, 5)
speed = 10.0
obstacleCourse = findPath.geometry.generateObstacleCourse(None, 50, 10, 10, 80, 80, 0.01, 0.1, minSpeed=0.0,
                                                          maxSpeed=4.0)
acceptanceThreshold = 2.0
numBins = 20

# Takes uniqueness of vertices into account
uniqueVertexQueue = UniqueVertexQueue(0, 0, 100, 100, speed)

pathFinder = DynamicPathFinder(start, end, speed, obstacleCourse, uniqueVertexQueue)
pathFindingVisualizer = DynamicPathFindingVisualizer(pathFinder, 800, 800, 50, 50, 100, 100)

gui.startGUI()
