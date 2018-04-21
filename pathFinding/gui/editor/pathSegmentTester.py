import Tkinter as tk

import numpy as np

from gui import Drawable, draw
from gui.draw import DEFAULT_COLOR, DEFAULT_POINT_SIZE
from gui.obstacleDebug.obstacleDebug import ObstacleCourseDebug
from subGUI import SubGUI


class PathSegmentTester(Drawable, SubGUI):
    def __init__(self, initialPathFindingEdit, obstacleData):
        self._initialPathFindingEdit = initialPathFindingEdit
        self._startPoint = np.array((5, 5), np.double)
        self._startVelocity = np.array((1, 1), np.double)
        self._targetPoint = np.array((95, 95), np.double)
        self._velocityOfTarget = np.array((0, 0), np.double)
        self._pointOfInterest = None
        self._obstacleDebug = None
        self._obstacleData = obstacleData

    def onLeftRelease(self, point, control=False):
        if control:
            self._targetPoint = point
        else:
            self._startPoint = point

    def onKey(self, point, key, ctrl=False):
        if key == "v":
            if ctrl:
                self._velocityOfTarget = point - self._targetPoint
            else:
                self._startVelocity = point - self._startPoint

    def onMotion(self, point, control=False):
        self._pointOfInterest = point

    def onSwitch(self):
        self._obstacleDebug = ObstacleCourseDebug(self._initialPathFindingEdit.boundaryPoints,
                                                  self._initialPathFindingEdit.noFlyZones)
        self._obstacleData.setInitialState(self._initialPathFindingEdit)
        self._obstacleData.setQueryTime(0.0)

    def draw(self, canvas, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR, **kwargs):
        draw.drawPoint(canvas, self._startPoint, radius=radius, color=color)
        draw.drawLine(canvas, self._startPoint,
                      self._startPoint + self._startVelocity,
                      arrow=tk.LAST)

        time = 0.0
        goalSegment = self._obstacleData.findPathSegment(self._startPoint, self._startVelocity,
                                                         self._targetPoint, self._velocityOfTarget)
        if goalSegment is not None:
            goalSegment.draw(canvas)

            if self._pointOfInterest is not None:
                (closestPoint, distance, pointTime) = goalSegment.calcPointDebug(self._pointOfInterest)
                if distance < 2.0:
                    time = pointTime
                    draw.drawPoint(canvas, closestPoint, radius=radius, color="orange")
                    self._obstacleDebug.draw(canvas, time=time, boundaryColor="red", nfzColor="blue")

        targetPoint = self._targetPoint + self._velocityOfTarget * time
        draw.drawPoint(canvas, targetPoint, radius=radius, color=color)
        draw.drawLine(canvas, targetPoint,
                      targetPoint + self._velocityOfTarget,
                      arrow=tk.LAST)
