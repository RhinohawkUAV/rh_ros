import engine
import gui
from demos import PathFindingVisualizer
from findPathStaticLegacy import StaticPathFinder, noFlyZoneLegacy
from findPathStaticLegacy.obstacleCourseLegacy import ObstacleCourseLegacy

startPoint = (95, 95)
endPoint = (5, 5)

noFlyZonesInputs = engine.utils.genRandomNoFlyZoneInputs(25, 10, 10, 80, 80, 0.01, 0.1)
noFlyZones = noFlyZoneLegacy.listFromInput(noFlyZonesInputs)
obstacleCourse = ObstacleCourseLegacy([], noFlyZones)

pathFinder = StaticPathFinder(startPoint, endPoint, obstacleCourse)
pathFindingVisualizer = PathFindingVisualizer(pathFinder, 800, 800, 50, 50, 100, 100)

gui.startGUI()
