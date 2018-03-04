import Tkinter as tk
from Tkinter import Canvas

from shapely.geometry import Polygon

import utils.geometry
from render.Drawable import Drawable
from render.drawables import DrawablePolygon, DrawableLine


class NoFlyZone(Drawable):
    def __init__(self, points, velocity):
        self.points = points
        self.velocity = velocity
        self.time = 0.0
        self.fill = "red"

    def blocksLineOfSight(self, line):
        polygon = Polygon(self.points)
        return line.crosses(polygon) or line.within(polygon)

    def blocksLineOfSightWithTime(self, line):
        polygon = Polygon(self.points)
        return line.crosses(polygon) or line.within(polygon)

    def findFutureHeadingCollisions(self, startPosition, speed):
        """
        Given the startPosition a speed of travel find headings and future collision with NFZ's points and
        headings to get there.
        Note: this does not account for solutions which require traveling through the NFZ.
        All headings have magnitude==speed (they are not normalized).
        :param startPosition:
        :param speed:
        :return: [((headingX,headingY),(collisionPointX,collisionPointY)),...]
        """
        result = []
        for point in self.points:
            solution = utils.geometry.hitTargetAtSpeed(startPosition, speed, point, self.velocity)
            if not solution is None:
                result.append(solution)
        return result

    def getPointsAtTime(self, time):
        pointsAtTime = []
        for vertex in self.points:
            pointsAtTime.append((vertex[0] + self.velocity[0] * time, vertex[1] + self.velocity[1] * time))
        return pointsAtTime

    def draw(self, canvas):
        # type: (Canvas) -> None
        pointsAtTime = self.getPointsAtTime(self.time)
        DrawablePolygon(pointsAtTime, fill=self.fill).draw(canvas)
        if self.velocity[0] != 0 or self.velocity[1] != 0:
            x1 = self.points[0][0]
            y1 = self.points[0][1]
            x2 = self.points[0][0] + self.velocity[0]
            y2 = self.points[0][1] + self.velocity[1]

            DrawableLine(x1, y1, x2, y2, fill="black", arrow=tk.LAST).draw(canvas)
