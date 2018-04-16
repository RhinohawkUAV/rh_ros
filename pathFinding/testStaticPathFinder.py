import engine
import gui
from demos import PathFindingVisualizer
from engine.geometry import ObstacleCourse
from findPathStaticLegacy import StaticPathFinder

start = (95, 95)
end = (5, 5)

noFlyZones = engine.utils.genRandomNoFlyZones(25, 10, 10, 80, 80, 0.01, 0.1)
obstacleCourse = ObstacleCourse([], noFlyZones)

pathFinder = StaticPathFinder(start, end, obstacleCourse)
pathFindingVisualizer = PathFindingVisualizer(pathFinder, 800, 800, 50, 50, 100, 100)

gui.startGUI()
