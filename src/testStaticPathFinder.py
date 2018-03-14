import gui
from demos import PathFindingVisualizer
from geometry import generator

# noFly1 = NoFlyZone([(40, 50), (40, 70), (50, 75), (50, 50)], (0, 0))
# noFly2 = NoFlyZone([(40, 40), (40, 45), (45, 45), (45, 40)], (0, 0))
# noFly3 = NoFlyZone([(10, 20), (30, 20), (30, 30), (10, 30)], (0, 0))
#
# pathFinder = Geometry([noFly1, noFly2, noFly3])
from findPathStatic.staticPathFinder import StaticPathFinder

start = (95, 95)
end = (5, 5)

obstacleCourse = generator.generateObstacleCourse(None, 25, 10, 10, 80, 80, 0.01, 0.1)

pathFinder = StaticPathFinder(start, end, obstacleCourse)
pathFindingVisualizer = PathFindingVisualizer(pathFinder, 800, 800, 50, 50, 100, 100)

gui.startGUI()
