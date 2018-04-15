import math

import numpy as np

import gui


class FindNFZIntersectDynamic:
    def __init__(self, obstacleCourse, startPoint, speed):
        self._obstacleCourse = obstacleCourse
        self.speed = speed
        self.setStartPoint(startPoint)

    def setStartPoint(self, startPoint):
        """Change the problem's start point and update the set of visible _points."""
        self.startPoint = np.array(startPoint, np.double)
        paths = self._obstacleCourse.findStraightPathsToVertices(self.startPoint, self.speed)
        self._visiblePoints = []
        for path in paths:
            self._visiblePoints.append(path.destination)

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
        self._obstacleCourse.draw(canvas, color="black", time=0)
        self._obstacleCourse.draw(canvas, color="blue", time=time)

        gui.draw.drawPoint(canvas, self.startPoint, color="green")

        for visiblePoint in self._visiblePoints:
            gui.draw.drawLine(canvas, self.startPoint, visiblePoint, color="black")
