import numpy as np

import gui
from engine.geometry import LineSegment
from gui import Drawable
from gui.draw import DEFAULT_COLOR
from subGUI import SubGUI


class PolyBuilder(Drawable, SubGUI):
    def __init__(self):
        self._points = []
        self._lines = []
        self._previewPoint = None
        self._previewLine = None

    def onLeftRelease(self, point, control=False):
        if control is False:
            self._add(point)
        else:
            self._close()

    def onMotion(self, point, control=False):
        if control:
            if len(self._points) > 0:
                self._previewPoint = self._points[0]
        else:
            self._previewPoint = point

        if len(self._points) > 0:
            self._previewLine = LineSegment(self._points[-1], self._previewPoint)

    def onKey(self, point, key, ctrl=False):
        if key == "Delete":
            if len(self._lines) > 0:
                self._lines = self._lines[:-1]
            if len(self._points) > 0:
                self._points = self._points[:-1]
            self._previewPoint = point
            if len(self._points) > 0:
                self._previewLine = LineSegment(self._points[-1], self._previewPoint)
            else:
                self._previewLine = None

    def _add(self, point):
        if len(self._points) > 0 and np.array_equal(point, self._points[-1]):
            return
        self._points.append(point)
        if len(self._points) > 1:
            self._lines.append(LineSegment(self._points[-2], self._points[-1]))

    def _close(self):
        if len(self._points) < 3:
            return None
        self._polyBuilt(self._points)
        self._points = []  # We gave the points object away so don't delete it
        del self._lines[:]
        self._previewPoint = None
        self._previewLine = None

    def _polyBuilt(self, points):
        pass

    def draw(self, canvas, color=DEFAULT_COLOR, **kwargs):
        for point in self._points:
            gui.draw.drawPoint(canvas, point, color=color)
        for line in self._lines:
            line.draw(canvas, color=color)
        if not self._previewPoint is None:
            gui.draw.drawPoint(canvas, self._previewPoint, color=color)
        if not self._previewLine is None:
            self._previewLine.draw(canvas, color=color)
