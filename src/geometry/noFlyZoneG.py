import Tkinter as tk
from Tkinter import Canvas

import numpy as np

import geometry.intersection
import lineSegment
from geometry.lineSegment import LineSeg
from gui import Drawable


class NoFlyZoneG(Drawable):
    def __init__(self, points, velocity):
        self.points = np.array(points, np.double)
        self.velocity = np.array(velocity, np.double)
        self._lines = []
        self._midPoint = self.points.sum(axis=0) / len(self.points)

        for i in range(0, len(points) - 1):
            self._lines.append(LineSeg(self.points[i], self.points[i + 1]))
        self._lines.append(LineSeg(self.points[-1], self.points[0]))

    def blocksLineOfSight(self, start, end, speed):
        """
        Does this moving no-fly-zone, block a path from start to end and the given speed?

        To solve this we convert the problem to the case the no-fly-zone is not moving, by
        offsetting the velocity of the moving object.

        :param start:
        :param end:
        :param speed: MUST BE POSITIVE unless start==end (not moving in one place)
        :return:
        """

        direction = end - start
        distance = np.linalg.norm(direction)
        if distance == 0.0:
            return False

        # velocity vector - has magnitude in speed heading in direction from start to end
        velocity = (speed / distance) * direction

        # Offset direction by the velocity of the no-fly-zone (pretend it is not moving)
        velocity -= self.velocity

        # Time to get from start to end
        t = distance / speed

        # The new end point takes the same time to reach, but at a new offset heading
        end = start + velocity * t

        for line in self._lines:
            if line.checkLineIntersection(start, end):
                return True
        return False

    def findFutureHeadingsAndCollisions(self, startPoint, speed):
        """
        Given the startPosition a speed of travel find headings and future collision with NFZ's points and
        headings to get there.
        Note: this does not account for solutions which require traveling through the NFZ.
        All headings have magnitude==speed (they are not normalized).
        :param startPoint:
        :param speed:
        :return: [((headingX,headingY),(collisionPointX,collisionPointY)),...]
        """
        result = []
        for point in self.points:
            solution = geometry.intersection.hitTargetAtSpeed(startPoint, speed, point, self.velocity)
            if not solution is None:
                result.append(solution)
        return result

    def getPointsAtTime(self, time):
        pointsAtTime = []
        for point in self.points:
            pointsAtTime.append((point[0] + self.velocity[0] * time, point[1] + self.velocity[1] * time))
        return pointsAtTime

    def draw(self, canvas, fill="red", time=0, **kwargs):
        # type: (Canvas) -> None

        # pointsAtTime = self.getPointsAtTime(time)
        # DrawablePolygon(pointsAtTime).draw(canvas, fill=fill, **kwargs)
        for line in self._lines:
            line.draw(canvas, fill=fill, time=time, **kwargs)

        if np.linalg.norm(self.velocity) > 0.0:
            lineSegment.drawLine(canvas, self._midPoint, self._midPoint + self.velocity * 4.0, fill="black",
                                 arrow=tk.LAST)
