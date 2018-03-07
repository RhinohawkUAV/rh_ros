from gui.visualizer import Visualizer


class FindTargetVisualizer(Visualizer):
    """Visualization/tester for the geometry.hitTargetAtSpeed method.
        Click the mouse to choose a start point.
        It will show all path to follow, at the given speed, to reach each vertex of the given NFZs in the future.
        As you move the mouse it will show where each NFZ would be, in the future, if Rhinohawk headed straight
        to that point, from its current position, at the given speed.  By tracing out the calculated paths you
        can verify, that it is working correctly.
    """

    def __init__(self, findTargetProblem, *args, **kw):
        Visualizer.__init__(self, *args, **kw)
        self.findTargetProblem = findTargetProblem
        self._time = 0.0

    def leftClick(self, event):
        self.findTargetProblem.setStartPoint(self.transformCanvasToPoint((event.x, event.y)))
        self._time = 0.0
        self.drawToCanvas(self.findTargetProblem, time=self._time)

    def motion(self, event):
        point = self.transformCanvasToPoint((event.x, event.y))
        self._time = self.findTargetProblem.calcTimeToPoint(point)
        self.drawToCanvas(self.findTargetProblem, time=self._time)
