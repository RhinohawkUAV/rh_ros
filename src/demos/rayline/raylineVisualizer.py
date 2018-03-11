from gui import Visualizer


class RayLineVisualizer(Visualizer):
    def __init__(self, rayLineProblem, *args, **kw):
        Visualizer.__init__(self, *args, **kw)
        self.rayLineProblem = rayLineProblem

    def onLeftClick(self, event):
        self.rayLineProblem.setStartPoint(self.transformCanvasToPoint((event.x, event.y)))
        self.drawToCanvas(self.rayLineProblem)

    def onRightClick(self, event):
        self.rayLineProblem.setEndPoint(self.transformCanvasToPoint((event.x, event.y)))
        self.drawToCanvas(self.rayLineProblem)

    def onResize(self, event):
        self.drawToCanvas(self.rayLineProblem)
