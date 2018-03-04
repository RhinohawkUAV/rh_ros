import math

from render.drawables import DrawableLine
from render.renderTargetWindow import RenderTargetWindow


class CollisionRenderTargetWindow(RenderTargetWindow):
    def __init__(self, canvasWidth, canvasHeight, viewCenterX, viewCenterY, viewWidth, viewHeight, **kw):
        RenderTargetWindow.__init__(self, canvasWidth, canvasHeight, viewCenterX, viewCenterY, viewWidth, viewHeight,
                                    **kw)
        self.bind('<Motion>', self.motion)
        self.noFlyZones = []
        self.startPoint = (0, 0)
        self.speed = 0
        self._visiblePoints = []
        self._time = 0.0
        self._mouseStartPoint = self.startPoint
        self._visiblePointsFromPoint = []

    def calcVisiblePoints(self, start, speed):
        visiblePoints = []
        for noFlyZone in self.noFlyZones:
            futurePoints = noFlyZone.findFuturePoints(start, speed)
            visiblePoints.extend(futurePoints)
        return visiblePoints

    def motion(self, event):
        self._mouseStartPoint = self.transformCanvasToPoint((event.x, event.y))
        xDiff = self._mouseStartPoint[0] - self.startPoint[0]
        yDiff = self._mouseStartPoint[1] - self.startPoint[1]
        distance = math.sqrt(xDiff * xDiff + yDiff * yDiff)
        self._time = distance / self.speed

        self._visiblePoints = self.calcVisiblePoints(self.startPoint, self.speed)
        self._visiblePointsFromPoint = self.calcVisiblePoints(self._mouseStartPoint, self.speed)
        self.render(self)

    def draw(self, canvas):
        for noFlyZone in self.noFlyZones:
            noFlyZone.time = 0.0
            noFlyZone.fill = "red"
            noFlyZone.draw(canvas)

        for visiblePoint in self._visiblePoints:
            drawLine = DrawableLine(self.startPoint[0], self.startPoint[1], visiblePoint[0],
                                    visiblePoint[1],
                                    fill="black")
            drawLine.draw(canvas)

        for noFlyZone in self.noFlyZones:
            noFlyZone.time = self._time
            noFlyZone.fill = "purple"
            noFlyZone.draw(canvas)

        for visiblePoint in self._visiblePointsFromPoint:
            drawLine = DrawableLine(self._mouseStartPoint[0], self._mouseStartPoint[1], visiblePoint[0],
                                    visiblePoint[1],
                                    fill="black")
            drawLine.draw(canvas)
