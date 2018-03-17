import copy

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
        self._resetDynamicPathFinder = dynamicPathFinder
        self._dynamicPathFinder = copy.deepcopy(self._resetDynamicPathFinder)
        self.pointOfInterest = None

    def onLeftClick(self, event):
        for i in range(0, STEPS_PER_CLICK):
            if self._dynamicPathFinder.running:
                self._dynamicPathFinder.step()
        self.updateDisplay()

    def onMouseMotion(self, event):
        self.pointOfInterest = self.transformCanvasToPoint((event.x, event.y))
        self.updateDisplay()

    def onRightClick(self, event):
        self._dynamicPathFinder = copy.deepcopy(self._resetDynamicPathFinder)
        self.updateDisplay()

    def updateDisplay(self):
        drawable = DynamicPathFinderDrawable(self._dynamicPathFinder)
        self.drawToCanvas(drawable, pointOfInterest=self.pointOfInterest, snapDistance=5.0)
