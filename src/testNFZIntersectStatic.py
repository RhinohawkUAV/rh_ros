import gui
from demos.findNFZIntersectStatic.findNFZIntersectStatic import FindNFZIntersectStatic
from demos.findNFZIntersectStatic.findNFZIntersectStaticVis import FindNFZIntersectStaticVis

# from findPathStatic.noFlyZone import NoFlyZone
#
from geometry.noFlyZone import NoFlyZone

noFly1 = NoFlyZone([(40, 50), (40, 70), (50, 75), (50, 50)], (0, 0))
# noFly2 = NoFlyZone([(40, 40), (40, 45), (45, 45), (45, 40)], (-2, 2))

initialLineIntersectProblem = FindNFZIntersectStatic(noFly1, (40, 30), (60, 30))
window = FindNFZIntersectStaticVis(initialLineIntersectProblem, 800, 800, 50, 50, 100, 100)

gui.startGUI()
