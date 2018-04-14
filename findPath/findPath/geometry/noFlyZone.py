import Tkinter as tk
from Tkinter import Canvas

import numpy as np

import calcs
import gui.draw
from constants import NO_FLY_ZONE_POINT_OFFSET
from gui import Drawable
from lineSegment import LineSegment


class NoFlyZone(Drawable):
    def __init__(self, points, velocity):
        """
        A polygon NFZ with a given velocity.  Points must be given in CCW order.
        :param points:
        :param velocity:
        """

        # Points composing the border of the NFZ
        self._points = np.array(points, np.double)
        self._velocity = np.array(velocity, np.double)

        # Outward facing normals for each point.  These are used for offsetting when computing blocksLineOfSight.
        self._offsetPoints = []
        self._lines = []
        self._midPoint = self._points.sum(axis=0) / len(self._points)

        for i in range(0, len(points)):
            self._lines.append(LineSegment(self._points[i - 1], self._points[i]))

        for i in range(0, len(points) - 1):
            pointNormal = (self._lines[i].n + self._lines[i + 1].n) / 2.0
            pointNormal /= np.linalg.norm(pointNormal)
            offsetPoint = self._points[i] + pointNormal * NO_FLY_ZONE_POINT_OFFSET
            self._offsetPoints.append(offsetPoint)

        i = -1
        pointNormal = (self._lines[i].n + self._lines[i + 1].n) / 2.0
        pointNormal /= np.linalg.norm(pointNormal)
        offsetPoint = self._points[i] + pointNormal * NO_FLY_ZONE_POINT_OFFSET
        self._offsetPoints.append(offsetPoint)

    def checkBlocksPath(self, start, end, speed):
        """
        Does this moving no-fly-zone, block a path from start to end and the given speed?

        To solve this we convert the problem to the case the no-fly-zone is not moving, by
        offsetting the velocity of the moving object.

        :param start:
        :param end:
        :param speed: MUST BE POSITIVE unless start==end (hovering in place)
        :return:
        """

        direction = end - start
        distance = np.linalg.norm(direction)
        if distance == 0.0:
            return False

        # velocity vector - has magnitude in speed heading in velocity from start to end
        velocity = (speed / distance) * direction

        # Offset velocity by the velocity of the no-fly-zone (pretend it is not moving)
        velocity -= self._velocity

        # Time to get from start to end
        t = distance / speed

        # The new end point takes the same time to reach, but at a new offset heading
        end = start + velocity * t

        for line in self._lines:
            if line.checkLineIntersection(start, end):
                return True
        return False

    def calcVelocitiesToVertices(self, startPosition, speed):
        """
        Given the startPosition and a speed of travel find velocity vectors that will reach each vertex (if possible).
        For each velocity vector, find the corresponding position where the vertex will be reached.

        Note: this does not account for if solutions are legal (would travel through this or other NFZs)

        :param startPosition:
        :param speed:
        :return: [StraightPathSolution1, StraightPathSolution2, ...]
        """
        result = []
        for point in self._offsetPoints:
            # startPosition will typically be a NFZ vertex.  We want to eliminate search from a start position to itself.
            if not calcs.arePointsClose(startPosition, point):
                solution = calcs.hitTargetAtSpeed(startPosition, speed, point, self._velocity)
                if not solution is None:
                    result.append(solution)
        return result

    def getFutureCopy(self, time):
        """Create a copy of this NFZ, as it will exist at some point in the future."""
        futurePoints = []
        for point in self._points:
            futurePoints.append((point[0] + self._velocity[0] * time, point[1] + self._velocity[1] * time))
        return NoFlyZone(futurePoints, self._velocity)

    def draw(self, canvas, time=0.0, drawVectors=True, **kwargs):
        # type: (Canvas) -> None

        if time == 0.0:
            # If time is 0 draw NFZ as it is now
            for line in self._lines:
                line.draw(canvas, time=time, drawVectors=drawVectors, **kwargs)

            if np.linalg.norm(self._velocity) > 0.0:
                gui.draw.drawLine(canvas, self._midPoint, self._midPoint + self._velocity,
                                  arrow=tk.LAST, **kwargs)
        else:
            # For future times, generate a future noFlyZone and draw that with time=0.0.
            self.getFutureCopy(time).draw(canvas, time=0.0, **kwargs)
