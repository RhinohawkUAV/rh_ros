import gui.core
from demos import FindTargetVisualizer, FindTargetProblem
from findPath.noFlyZone import NoFlyZone

noFly1 = NoFlyZone([(40, 50), (40, 70), (50, 75), (50, 50)], (5.0, -2.0))

initialFindTargetProblem = FindTargetProblem([noFly1], (10, 30), 10.0)
window = FindTargetVisualizer(initialFindTargetProblem, 800, 800, 50, 50, 100, 100)

gui.core.startGUI()
