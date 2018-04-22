import math

import numpy as np

from constants import DISTANCE_TOLERANCE_SQUARED
from engine.utils import quadratic


def arePointsClose(p1, p2):
    """
    Are 2 points almost equal (within tolerance of each other)
    :param p1:
    :param p2:
    :return:
    """
    x = p1[0] - p2[0]
    y = p1[0] - p2[0]
    return x * x + y * y < DISTANCE_TOLERANCE_SQUARED


def calcTravelTime(p1, p2, speed):
    """
    Calculate the time from p1 to p2 at speed.
    :param p1:
    :param p2:
    :param speed:
    :return:
    """
    return np.linalg.norm(p2 - p1) / speed


def calcTravelTimeAndDirection(p1, p2, speed):
    """
    Calculate the time and direction from p1 to p2 at speed.
    Note: direction is a unit vector.

    :param p1:
    :param p2:
    :param speed:
    :return: (time, direction)
    """
    diff = p2 - p1
    distance = np.linalg.norm(diff)
    if distance == 0.0:
        direction = np.array([0, 0], np.double)
        time = 0.0
    else:
        direction = diff / distance
        time = distance / speed
    return (time, direction)


def findClosestPoint(point, points):
    """For drawing only, not for computation."""
    closestDistanceSquared = float("inf")
    closestPointIndex = None
    for i in range(0, len(points)):
        diff = point - points[i]
        diff *= diff
        distSquared = diff.sum()
        if distSquared < closestDistanceSquared:
            closestDistanceSquared = distSquared
            closestPointIndex = i
    return (closestDistanceSquared, closestPointIndex)


class StraightPathSolution:
    """
    Holds _solution to the hitTargetAtSpeed problem.
    """

    def __init__(self, time, velocity, endPoint):
        self.time = time
        self.velocity = velocity
        self.endPoint = endPoint


def hitTargetAtSpeed(projectileStart, projectileSpeed, targetStartPoint, targetVelocity):
    """
    Solves the following problem:
    A projectile starts at a position with a given speed.  A target starts at another position and has a given velocity.

    What is the velocity vector the projectile should follow to hit the moving target?
    Where will the projectile collide with the target?

    Note: We start knowing the speed of the projectile, but not the velocity.

    :param projectileStart: where craft starts
    :param projectileSpeed: magnitude of craft's velocity
    :param targetStartPoint: the target's starting location
    :param targetVelocity: the velocity vector of the target
    :return: StraightPathSolution or None, if speed is insufficient given position
    """

    # Vector heading from start towards point
    towards = targetStartPoint - projectileStart
    towardsMagSquared = np.dot(towards, towards)
    velDotTowards = np.dot(towards, targetVelocity)

    # We need a mixture of velocity and the towardsVector.  Its magnitude needs to be projectileSpeed.
    # heading = velocity + towardsFactor * towardsVector.
    # Solve for towardsFactor:

    a = -towardsMagSquared
    b = - 2 * velDotTowards
    c = projectileSpeed * projectileSpeed - np.dot(targetVelocity, targetVelocity)

    # We always want the 1st _solution
    towardsFactor = quadratic.solveQuad(a, b, c)

    # Error!
    # Note: 0 solutions, when projectileSpeed is too low compared to point's velocity
    # A _solution of infinity when projectileSpeed matches velocity exactly
    if len(towardsFactor) == 0 or towardsFactor[0] == 0.0:
        return None

    # We can easily calculate time (t) from towardsFactor
    time = 1.0 / towardsFactor[0]

    velocity = towards * towardsFactor[0] + targetVelocity
    endPoint = targetStartPoint + time * targetVelocity

    return StraightPathSolution(time, velocity, endPoint)


def calcBounds(points):
    """Calculate a bounding rectangle around a group of points"""
    xMin = float("inf")
    yMin = float("inf")
    xMax = -float("inf")
    yMax = -float("inf")
    for point in points:
        if point[0] < xMin:
            xMin = point[0]
        if point[0] > xMax:
            xMax = point[0]
        if point[1] < yMin:
            yMin = point[1]
        if point[1] > yMax:
            yMax = point[1]
    return [xMin, yMin, xMax, yMax]


def calcEdgeAngle(prev, vertex, next):
    """
    Calculate the angle at a vertex in a polygon assuming CCW winding
    """
    diff = vertex - prev
    length = np.linalg.norm(diff)
    dir1 = diff / length

    diff = vertex - next
    length = np.linalg.norm(diff)
    dir2 = diff / length

    cosAngle = np.dot(dir1, dir2)

    diff = next - prev
    length = np.linalg.norm(diff)
    dir3 = diff / length

    angle = math.acos(cosAngle)

    if cross2(dir1, dir3) > 0.0:
        return angle
    else:
        return math.pi * 2 - angle


def woundCCW(points):
    angle = 0
    for i in range(-1, len(points) - 1):
        prev = points[i - 1]
        curr = points[i]
        next = points[i + 1]

        angle += calcEdgeAngle(prev, curr, next)
    # If the points are wound CCW then this should given an answer following the standard formula for angle inside a polygon.
    # If wound CW, we will get a much bigger number (4*pi bigger).  We add a small margin of error (0.1) to the calculation.
    if angle < math.pi * (len(points) - 2) + 0.1:
        return True
    else:
        return False


def cross2(v1, v2):
    return v1[0] * v2[1] - v1[1] * v2[0]
