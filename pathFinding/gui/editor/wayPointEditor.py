from engine.geometry import calcs
from engine.interface.fileUtils import SCENARIO_KEY
from gui.draw import DEFAULT_COLOR, DEFAULT_POINT_SIZE, VELOCITY_TO_PIXEL
from subGUI import SubGUI


class WayPointEditor(SubGUI):

    def __init__(self, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR):
        self._points = []
        self._startVelocity = None
        self._radius = radius
        self._color = color
        self._offset = None
        self._dragIndex = None

    def onSwitch(self, params, scenario, vehicle, testInput, visualizer):
        SubGUI.onSwitch(self, params, scenario, vehicle, testInput, visualizer)
        self._points = [self._scenario.startPoint]
        self._startVelocity = self._scenario.startVelocity
        self._points.extend(self._scenario.wayPoints)
    
    def sync(self):
        self._scenario.startPoint = self._points[0]
        self._scenario.startVelocity = self._startVelocity
        if len(self._points) > 1:
            self._scenario.wayPoints = self._points[1:]
    
    def onLeftPress(self, point, control=False):
        if control:
            if len(self._points) > 0:
                (trash, self._dragIndex) = calcs.findClosestPoint(point, self._points)
                self._offset = self._points[self._dragIndex] - point
        self.sync()

    def onLeftRelease(self, point, control=False):
        if control:
            self._dragIndex = None
        else:
            self._points.append(point)
        self.sync()

    def onMotion(self, point, control=False):
        if control:
            if self._dragIndex is not None:
                self._points[self._dragIndex] = point + self._offset
        self.sync()

    def onKey(self, point, key, ctrl=False):
        if key == "v":
            self._startVelocity = self._visualizer.scaleVecToPixels(point - self._points[0]) / VELOCITY_TO_PIXEL
        if key == "Delete":
            if len(self._points) > 1:
                (distance, index) = calcs.findClosestPoint(point, self._points)
                self._points.pop(index)
        self.sync()