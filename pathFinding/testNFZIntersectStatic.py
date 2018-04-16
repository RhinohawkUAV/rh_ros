import gui
from demos import FindNFZIntersectStatic
from demos import FindNFZIntersectStaticVis
from engine.geometry.noFlyZone import NoFlyZone

noFly1 = NoFlyZone([(40, 50), (40, 70), (50, 75), (50, 50)], (0, 0))
initialLineIntersectProblem = FindNFZIntersectStatic(noFly1, (40, 30), (60, 30))
window = FindNFZIntersectStaticVis(initialLineIntersectProblem, 800, 800, 50, 50, 100, 100)
gui.startGUI()
