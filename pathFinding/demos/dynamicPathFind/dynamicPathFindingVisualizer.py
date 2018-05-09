import copy

from dynamicPathFinderDrawable import DynamicPathFinderDrawable
from gui.editor.pathSegmentTester.obstacleDebug import ObstacleCourseDebug
from gui.visualizer import Visualizer

# TODO: Not currently used
SNAP_PIXEL_DISTANCE = 10
STEPS_PER_CLICK = 1


class DynamicPathFindingVisualizer(Visualizer):
    """
    Visualizes the dynamic path finding process.
    """

    def __init__(self, dynamicPathFinder, scenario, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self._resetDynamicPathFinder = dynamicPathFinder
        self._dynamicPathFinder = copy.deepcopy(self._resetDynamicPathFinder)
        self._obstacleCourseDebug = ObstacleCourseDebug(scenario.boundaryPoints, scenario.noFlyZones)
        self.pointOfInterest = None
        self.bindWithTransform('<Motion>', self.onMouseMotion)
        self.bindWithTransform('<Button-1>', self.onLeftClick)
        self.bindWithTransform('<Button-3>', self.onRightClick)

    def doRealStep(self):
        while not self._dynamicPathFinder.isDone() and not self._dynamicPathFinder.step():
            pass

    def onLeftClick(self, point, event):
        for i in range(0, STEPS_PER_CLICK):
            self.doRealStep()
        self.updateDisplay()

    def onMouseMotion(self, point, event):
        self.pointOfInterest = point
        self.updateDisplay()

    def onRightClick(self, point, event):
        self._dynamicPathFinder = copy.deepcopy(self._resetDynamicPathFinder)
        self.updateDisplay()

    def updateDisplay(self):
        drawable = DynamicPathFinderDrawable(self._dynamicPathFinder, self._obstacleCourseDebug)
        self.drawToCanvas(drawable, pointOfInterest=self.pointOfInterest, snapDistance=5.0)
