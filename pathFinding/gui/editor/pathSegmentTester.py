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
        self._showPathsToPoints = False
        self._pathSegments = []
        self._filteredPathSegments = []

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

        if key == "z":
            self._showPathsToPoints = not self._showPathsToPoints

    def onMotion(self, point, control=False):
        self._pointOfInterest = point

    def onSwitch(self):
        self._obstacleDebug = ObstacleCourseDebug(self._obstacleCourseEdit.boundaryPoints,
                                                  self._obstacleCourseEdit.noFlyZones)
        self._obstacleData.setInitialState(self._obstacleCourseEdit)

    def draw(self, canvas, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR, **kwargs):
        draw.drawPoint(canvas, self._testEdit.startPoint, radius=radius, color=color)
        draw.drawLine(canvas, self._testEdit.startPoint,
                      self._testEdit.startPoint + self._testEdit.startVelocity,
                      arrow=tk.LAST)

        goalSegment = self._obstacleData.findPathSegment(startTime=0.0,
                                                         startPoint=self._testEdit.startPoint,
                                                         startVelocity=self._testEdit.startVelocity,
                                                         targetPoint=self._testEdit.targetPoint,
                                                         velocityOfTarget=self._testEdit.velocityOfTarget)

        # Draw obstacles as they appear at this time
        drawTime = 0.0
        if goalSegment is not None:
            goalSegment.draw(canvas)

            if self._pointOfInterest is not None:
                (closestPoint, distance, pointTime) = goalSegment.calcPointDebug(self._pointOfInterest)
                if distance < 2.0:
                    drawTime = pointTime
                    draw.drawPoint(canvas, closestPoint, radius=radius, color="orange")
                    self._obstacleDebug.draw(canvas, time=drawTime, boundaryColor="red", nfzColor="blue")

        if self._showPathsToPoints:
            (self._pathSegments, self._filteredPathSegments) = self._obstacleData.findPathSegments(startTime=0.0,
                                                                                                   startPoint=self._testEdit.startPoint,
                                                                                                   startVelocity=self._testEdit.startVelocity)
        else:
            self._pathSegments = []
            self._filteredPathSegments = []

        for pathSegment in self._pathSegments:
            pathSegment.draw(canvas)
        for pathSegment in self._filteredPathSegments:
            pathSegment.draw(canvas, filtered=True)

        targetPoint = self._testEdit.targetPoint + self._testEdit.velocityOfTarget * drawTime
        draw.drawPoint(canvas, targetPoint, radius=radius, color=color)
        draw.drawLine(canvas, targetPoint,
                      targetPoint + self._testEdit.velocityOfTarget,
                      arrow=tk.LAST)
