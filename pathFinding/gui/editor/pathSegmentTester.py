import Tkinter as tk

from gui import Drawable, draw
from gui.draw import DEFAULT_COLOR, DEFAULT_POINT_SIZE
from gui.obstacleDebug.obstacleDebug import ObstacleCourseDebug
from subGUI import SubGUI


class PathSegmentTester(Drawable, SubGUI):
    def __init__(self, testEdit, obstacleCourseEdit, obstacleData):
        self._obstacleCourseEdit = obstacleCourseEdit
        self._testEdit = testEdit
        self._pointOfInterest = None
        self._obstacleDebug = None
        self._obstacleData = obstacleData

    def onLeftRelease(self, point, control=False):
        if control:
            self._testEdit.targetPoint = point
        else:
            self._testEdit.startPoint = point

    def onKey(self, point, key, ctrl=False):
        if key == "v":
            if ctrl:
                self._testEdit.velocityOfTarget = point - self._testEdit.targetPoint
            else:
                self._testEdit.startVelocity = point - self._testEdit.startPoint

    def onMotion(self, point, control=False):
        self._pointOfInterest = point

    def onSwitch(self):
        self._obstacleDebug = ObstacleCourseDebug(self._obstacleCourseEdit.boundaryPoints,
                                                  self._obstacleCourseEdit.noFlyZones)
        self._obstacleData.setInitialState(self._obstacleCourseEdit)
        self._obstacleData.setQueryTime(0.0)

    def draw(self, canvas, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR, **kwargs):
        draw.drawPoint(canvas, self._testEdit.startPoint, radius=radius, color=color)
        draw.drawLine(canvas, self._testEdit.startPoint,
                      self._testEdit.startPoint + self._testEdit.startVelocity,
                      arrow=tk.LAST)

        time = 0.0
        goalSegment = self._obstacleData.findPathSegment(self._testEdit.startPoint, self._testEdit.startVelocity,
                                                         self._testEdit.targetPoint, self._testEdit.velocityOfTarget)
        if goalSegment is not None:
            goalSegment.draw(canvas)

            if self._pointOfInterest is not None:
                (closestPoint, distance, pointTime) = goalSegment.calcPointDebug(self._pointOfInterest)
                if distance < 2.0:
                    time = pointTime
                    draw.drawPoint(canvas, closestPoint, radius=radius, color="orange")
                    self._obstacleDebug.draw(canvas, time=time, boundaryColor="red", nfzColor="blue")

        targetPoint = self._testEdit.targetPoint + self._testEdit.velocityOfTarget * time
        draw.drawPoint(canvas, targetPoint, radius=radius, color=color)
        draw.drawLine(canvas, targetPoint,
                      targetPoint + self._testEdit.velocityOfTarget,
                      arrow=tk.LAST)
