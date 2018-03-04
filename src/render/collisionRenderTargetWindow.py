import math

from render.drawables import DrawableLine
from render.renderTargetWindow import RenderTargetWindow


class CollisionRenderTargetWindow(RenderTargetWindow):
    def __init__(self, canvasWidth, canvasHeight, viewCenterX, viewCenterY, viewWidth, viewHeight, **kw):
        RenderTargetWindow.__init__(self, canvasWidth, canvasHeight, viewCenterX, viewCenterY, viewWidth, viewHeight,
                                    **kw)
        self.bind('<Motion>', self.motion)
        self.bind('<Button-1>', self.leftClick)
        self.noFlyZones = []
        self.startPoint = (0, 0)
        self.speed = 0
        self._visiblePoints = []
        self._time = 0.0
        self._visiblePointsFromPoint = []

    def calcVisiblePoints(self, start, speed):
        visiblePoints = []
        for noFlyZone in self.noFlyZones:
            results = noFlyZone.findFutureHeadingCollisions(start, speed)
            for result in results:
                point = result[1]
                visiblePoints.append(point)
        return visiblePoints

    def leftClick(self, event):
        self.startPoint = self.transformCanvasToPoint((event.x, event.y))
        self._time = 0.0
        self._visiblePoints = self.calcVisiblePoints(self.startPoint, self.speed)
        self.render(self)

    def motion(self, event):
        point = self.transformCanvasToPoint((event.x, event.y))
        xDiff = point[0] - self.startPoint[0]
        yDiff = point[1] - self.startPoint[1]
        distance = math.sqrt(xDiff * xDiff + yDiff * yDiff)
        self._time = distance / self.speed

        self._visiblePoints = self.calcVisiblePoints(self.startPoint, self.speed)
        self.render(self)

    def draw(self, canvas):
        for noFlyZone in self.noFlyZones:
            noFlyZone.time = 0.0
            noFlyZone.fill = "red"
            noFlyZone.draw(canvas)

        for noFlyZone in self.noFlyZones:
            noFlyZone.time = self._time
            noFlyZone.fill = "purple"
            noFlyZone.draw(canvas)

        for visiblePoint in self._visiblePoints:
            drawLine = DrawableLine(self.startPoint[0], self.startPoint[1], visiblePoint[0],
                                    visiblePoint[1],
                                    fill="black")
            drawLine.draw(canvas)
