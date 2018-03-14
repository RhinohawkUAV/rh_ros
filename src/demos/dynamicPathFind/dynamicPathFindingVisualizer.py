from graph.dynamicPathFinder import DynamicPathFinderDrawable
from gui.visualizer import Visualizer

# TODO: Not currently used
SNAP_PIXEL_DISTANCE = 10


class DynamicPathFindingVisualizer(Visualizer):
    """
    Visualizes the dynamic path finding process.
    """

    def __init__(self, dynamicPathFinder, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self._dynamicPathFinder = dynamicPathFinder
        self.pointOfInterest = None

    def onLeftClick(self, event):
        if not self._dynamicPathFinder.step():
            print "done"
        self.updateDisplay()

    def onMouseMotion(self, event):
        self.pointOfInterest = self.transformCanvasToPoint((event.x, event.y))
        self.updateDisplay()

    def onRightClick(self, event):
        self.pointOfInterest = self.transformCanvasToPoint((event.x, event.y))
        self.updateDisplay()

    def updateDisplay(self):
        drawable = DynamicPathFinderDrawable(self._dynamicPathFinder)
        self.drawToCanvas(drawable, pointOfInterest=self.pointOfInterest, snapDistance=2.0)
