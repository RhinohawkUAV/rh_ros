import gui
from demos.rayline.raylineProblem import RayLineProblem
from demos.rayline.raylineVisualizer import RayLineVisualizer

# from findPath.noFlyZone import NoFlyZone
#
# noFly1 = NoFlyZone([(40, 50), (40, 70), (50, 75), (50, 50)], (3, 0))
# noFly2 = NoFlyZone([(40, 40), (40, 45), (45, 45), (45, 40)], (-2, 2))

initialRayLineProblem = RayLineProblem((50, 10), (50, 50), (40, 30), (1, 0))
window = RayLineVisualizer(initialRayLineProblem, 800, 800, 50, 50, 100, 100)

gui.startGUI()
