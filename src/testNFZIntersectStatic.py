import gui
from demos.rayline.findNFZIntersectStatic import FindNFZIntersectStatic
from demos.rayline.findNFZIntersectStaticVis import FindNFZIntersectStaticVis

# from findPath.noFlyZone import NoFlyZone
#
from geometry.noFlyZone import NoFlyZoneG

noFly1 = NoFlyZoneG([(40, 50), (40, 70), (50, 75), (50, 50)], (0, 0))
# noFly2 = NoFlyZone([(40, 40), (40, 45), (45, 45), (45, 40)], (-2, 2))

initialLineIntersectProblem = FindNFZIntersectStatic(noFly1, (40, 30), (60, 30))
window = FindNFZIntersectStaticVis(initialLineIntersectProblem, 800, 800, 50, 50, 100, 100)

gui.startGUI()
