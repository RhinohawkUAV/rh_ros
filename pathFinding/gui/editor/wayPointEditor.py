import Tkinter as tk
from engine.geometry import calcs
from engine.interface.fileUtils import SCENARIO_KEY
from gui import Drawable, draw
from gui.draw import DEFAULT_COLOR, DEFAULT_POINT_SIZE, VELOCITY_SCALE
import numpy as np
from subGUI import SubGUI


class WayPointEditor(SubGUI, Drawable):

    def __init__(self, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR):
        self._points = []
        self._startVelocity = None
        self._radius = radius
        self._color = color
        self._offset = None
        self._dragIndex = None

    def onSwitch(self, inputDict):
        SubGUI.onSwitch(self, inputDict)
        self._points = [inputDict[SCENARIO_KEY].startPoint]
        self._startVelocity = inputDict[SCENARIO_KEY].startVelocity
        self._points.extend(inputDict[SCENARIO_KEY].wayPoints)
    
    def sync(self):
        self._inputDict[SCENARIO_KEY].startPoint = self._points[0]
        self._inputDict[SCENARIO_KEY].startVelocity = self._startVelocity
        if len(self._points) > 1:
            self._inputDict[SCENARIO_KEY].wayPoints = self._points[1:]
    
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
            self._startVelocity = (point - self._points[0]) / VELOCITY_SCALE
        if key == "Delete":
            if len(self._points) > 1:
                (distance, index) = calcs.findClosestPoint(point, self._points)
                self._points.pop(index)
        self.sync()

    def draw(self, canvas, **kwargs):
        pass
