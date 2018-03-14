from graph.dynamicPathFinder import DynamicPathFinderDrawable
from gui.visualizer import Visualizer


class DynamicPathFindingVisualizer(Visualizer):
    """
    Visualizes the dynamic path finding process.
    """

    def __init__(self, dynamicPathFinder, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self._dynamicPathFinder = dynamicPathFinder

    def onLeftClick(self, event):
        # for i in range(0, 5):
        #     if not self._dynamicPathFinder.step():
        #         print "done"
        if not self._dynamicPathFinder.step():
            print "done"
        drawable = DynamicPathFinderDrawable(self._dynamicPathFinder)
        self.drawToCanvas(drawable)
