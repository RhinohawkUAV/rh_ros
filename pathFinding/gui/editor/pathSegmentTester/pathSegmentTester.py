import Tkinter as tk
from gui import Drawable, draw
from gui.draw import DEFAULT_COLOR, DEFAULT_POINT_SIZE
from gui.editor.subGUI import SubGUI
from obstacleDebug import ObstacleCourseDebug


class PathSegmentTester(Drawable, SubGUI):

    def __init__(self, obstacleData):
        self._obstacleData = obstacleData
        self._pointOfInterest = None
        self._obstacleDebug = None
        self._showPathsToPoints = False
        self._pathSegments = []
        self._filteredPathSegments = []

    def onLeftRelease(self, point, control=False):
        if control:
            self._debugInput.testInput.targetPoint = point
        else:
            self._debugInput.testInput.startPoint = point

    def onKey(self, point, key, ctrl=False):
        if key == "v":
            if ctrl:
                self._debugInput.testInput.velocityOfTarget = point - self._debugInput.testInput.targetPoint
            else:
                self._debugInput.testInput.startVelocity = point - self._debugInput.testInput.startPoint

        if key == "z":
            self._showPathsToPoints = not self._showPathsToPoints

    def onMotion(self, point, control=False):
        self._pointOfInterest = point

    def onSwitch(self, debugInput):
        SubGUI.onSwitch(self, debugInput)
        self._obstacleDebug = ObstacleCourseDebug(self._debugInput.scenario.boundaryPoints,
                                                  self._debugInput.scenario.noFlyZones)
        self._obstacleData.setInitialState(self._debugInput.scenario.boundaryPoints,
                                                  self._debugInput.scenario.noFlyZones)

    def draw(self, canvas, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR, **kwargs):
        draw.drawPoint(canvas, self._debugInput.testInput.startPoint, radius=radius, color=color)
        draw.drawLine(canvas, self._debugInput.testInput.startPoint,
                      self._debugInput.testInput.startPoint + self._debugInput.testInput.startVelocity,
                      arrow=tk.LAST)

        goalSegment = self._obstacleData.findPathSegment(startTime=0.0,
                                                         startPoint=self._debugInput.testInput.startPoint,
                                                         startVelocity=self._debugInput.testInput.startVelocity,
                                                         targetPoint=self._debugInput.testInput.targetPoint,
                                                         velocityOfTarget=self._debugInput.testInput.velocityOfTarget)

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
                                                                                                   startPoint=self._debugInput.testInput.startPoint,
                                                                                                   startVelocity=self._debugInput.testInput.startVelocity)
        else:
            self._pathSegments = []
            self._filteredPathSegments = []

        for pathSegment in self._pathSegments:
            pathSegment.draw(canvas)
        for pathSegment in self._filteredPathSegments:
            pathSegment.draw(canvas, filtered=True)

        targetPoint = self._debugInput.testInput.targetPoint + self._debugInput.testInput.velocityOfTarget * drawTime
        draw.drawPoint(canvas, targetPoint, radius=radius, color=color)
        draw.drawLine(canvas, targetPoint,
                      targetPoint + self._debugInput.testInput.velocityOfTarget,
                      arrow=tk.LAST)
