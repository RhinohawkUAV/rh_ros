from gui import Visualizer


class LineIntersectVisualizer(Visualizer):
    def __init__(self, lineIntersectProblem, *args, **kw):
        Visualizer.__init__(self, *args, **kw)
        self.lineIntersectProblem = lineIntersectProblem

    def onLeftClick(self, event):
        self.lineIntersectProblem.setStartPoint(self.transformCanvasToPoint((event.x, event.y)))
        self.drawToCanvas(self.lineIntersectProblem)

    def onRightClick(self, event):
        self.lineIntersectProblem.setEndPoint(self.transformCanvasToPoint((event.x, event.y)))
        self.drawToCanvas(self.lineIntersectProblem)

    def onResize(self, event):
        self.drawToCanvas(self.lineIntersectProblem)
