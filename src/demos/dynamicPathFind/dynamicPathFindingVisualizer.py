from findPathDynamic.dynamicPathFinder import DynamicPathFinderDrawable
from gui.visualizer import Visualizer

# TODO: Not currently used
SNAP_PIXEL_DISTANCE = 10
STEPS_PER_CLICK = 1


class DynamicPathFindingVisualizer(Visualizer):
    """
    Visualizes the dynamic path finding process.
    """

    def __init__(self, dynamicPathFinder, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self._dynamicPathFinder = dynamicPathFinder
        self.pointOfInterest = None

    def onLeftClick(self, event):
        for i in range(0, STEPS_PER_CLICK):
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
