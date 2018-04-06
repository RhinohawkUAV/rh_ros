import gui
from demos import findNFZIntersectDynamicVis, FindNFZIntersectDynamic
from findPath.geometry.noFlyZone import NoFlyZone
from findPath.geometry.obstacleCourse import ObstacleCourse

noFly1 = NoFlyZone([(40, 50), (40, 70), (50, 75), (50, 50)], (3, 0))
noFly2 = NoFlyZone([(40, 40), (40, 45), (45, 45), (45, 40)], (-2, 2))

initialFindTargetProblem = FindNFZIntersectDynamic(ObstacleCourse(None, [noFly1, noFly2]), (10, 85), 10.0)
window = findNFZIntersectDynamicVis(initialFindTargetProblem, 800, 800, 50, 50, 100, 100)

gui.startGUI()
