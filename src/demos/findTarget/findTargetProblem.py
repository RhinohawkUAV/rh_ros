import math

from gui import DrawableLine, DrawableCircle


class FindTargetProblem:
    def __init__(self, noFlyZones, startPoint, speed):
        self.noFlyZones = noFlyZones
        self.speed = speed
        self.setStartPoint(startPoint)

    def setStartPoint(self, startPoint):
        """Change the problem's start point and update the set of visible points."""
        self.startPoint = startPoint
        self._visiblePoints = self.calcVisiblePoints(self.startPoint, self.speed)

    def calcVisiblePoints(self, startPoint, speed):
        """Calculate the list of points visible
        TODO: Create/move to noFlyZoneGroup
        """
        visiblePoints = []
        for noFlyZone in self.noFlyZones:
            results = noFlyZone.findFutureHeadingCollisions(startPoint, speed)
            for result in results:
                point = result[1]
                visiblePoints.append(point)
        return visiblePoints

    def calcTimeToPoint(self, point):
        """
        Calculates How long to get to point.
        :param point: (x,y)
        :return:
        """
        xDiff = point[0] - self.startPoint[0]
        yDiff = point[1] - self.startPoint[1]
        distance = math.sqrt(xDiff * xDiff + yDiff * yDiff)
        return distance / self.speed

    def draw(self, canvas, time=0, **kwargs):

        for noFlyZone in self.noFlyZones:
            noFlyZone.draw(canvas, fill="red", time=0)

        for noFlyZone in self.noFlyZones:
            noFlyZone.draw(canvas, fill="purple", time=time)

        DrawableCircle(self.startPoint[0], self.startPoint[1], 1).draw(canvas, fill="green")

        for visiblePoint in self._visiblePoints:
            drawLine = DrawableLine(self.startPoint[0], self.startPoint[1],
                                    visiblePoint[0], visiblePoint[1])
            drawLine.draw(canvas, fill="black")
