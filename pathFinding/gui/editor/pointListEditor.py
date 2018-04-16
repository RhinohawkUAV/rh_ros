from engine.geometry import calcs
from gui import Drawable
from gui.draw import DEFAULT_COLOR, DEFAULT_POINT_SIZE
from subGUI import SubGUI


class PointListEditor(Drawable, SubGUI):
    def __init__(self, points, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR):
        self._points = points
        self._radius = radius
        self._color = color
        self._offset = None
        self._dragIndex = None

    def onLeftPress(self, point, control=False):
        if control:
            if len(self._points) > 0:
                (trash, self._dragIndex) = calcs.findClosestPoint(point, self._points)
                self._offset = self._points[self._dragIndex] - point

    def onLeftRelease(self, point, control=False):
        if control:
            self._dragIndex = None
        else:
            self._points.append(point)

    def onMotion(self, point, control=False):
        if control:
            if self._dragIndex is not None:
                self._points[self._dragIndex] = point + self._offset
