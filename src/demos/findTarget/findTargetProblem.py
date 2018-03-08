import math

from gui import DrawableLine, DrawableCircle


class FindTargetProblem:
    def __init__(self, obstacleCourse, startPoint, speed):
        self._obstacleCourse = obstacleCourse
        self.speed = speed
        self.setStartPoint(startPoint)

    def setStartPoint(self, startPoint):
        """Change the problem's start point and update the set of visible points."""
        self.startPoint = startPoint
        self._visiblePoints = self._obstacleCourse.findVisibleVerticesDynamic(self.startPoint, self.speed)

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
        self._obstacleCourse.draw(canvas, fill="red", time=0)
        self._obstacleCourse.draw(canvas, fill="purple", time=time)

        DrawableCircle(self.startPoint[0], self.startPoint[1], 1).draw(canvas, fill="green")

        for visiblePoint in self._visiblePoints:
            drawLine = DrawableLine(self.startPoint[0], self.startPoint[1],
                                    visiblePoint[0], visiblePoint[1])
            drawLine.draw(canvas, fill="black")
