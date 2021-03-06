import math

from constants import DISTANCE_TOLERANCE_SQUARED, NFZ_MAX_NEW_VERTEX_EXPANSION_RATIO
import numpy as np
from utils import quadratic

# TODO: Move rotation direction constants to here and use them in code.


# TODO: Use where appropriate
class NoSolutionException(BaseException):
    pass


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


def isPointInCircle(center, radius, point):
    return np.linalg.norm(center - point) <= radius


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


# TODO: Consolidate circle intersection calcs.  All use the same primary equation.
def getRayCircleIntersections(startPoint, direction, center, radius):
    """
    Calculate distances, from start, where a ray intersects a circle.  
    Returns a list of distances of length 0, 1 or 2.  
    The smallest distance always appears 1st and can be negative to represent the case of the intersection being "behind" the source.
    
    direction must not be (0,0)!
    """
    fromCenter = startPoint - center
    return quadratic.solveQuad(1.0,
                               2.0 * np.dot(direction, fromCenter),
                               np.dot(fromCenter, fromCenter) - radius * radius)


def lineSegmentCircleIntersect(startPoint, lineVector, center, radius):
    """
    Test if the given line segment intersects a circle.
    """
    fromCenter = startPoint - center
    solutions = quadratic.solveQuad(np.dot(lineVector, lineVector),
                               2.0 * np.dot(lineVector, fromCenter),
                               np.dot(fromCenter, fromCenter) - radius * radius)
    for solution in solutions:
        if solution >= 0.0 and solution <= 1.0:
            return True
    return False


def lineRayCircleIntersect(startPoint, direction, length, center, radius):
    """
    Test if the given line segment intersects a circle.
    """
    distances = getRayCircleIntersections(startPoint, direction, center, radius)
    for distance in distances:
        if distance >= 0.0 and distance <= length:
            return True
    return False


def movingCircleCollision(relativePosition, relativeVelocity, radius1, radius2, outside=True):
    """
    Find times when two moving circles collide with each other.
    If outside == True, then this will find times when circles 1st touch "externally"
    Otherwise, this will find times when circles touch, with one inside the other.
    
    Times can be negative, reflecting an intersection which occurred in the past.  Smaller time will appear 1st.
    """
    if outside:
        rsquared = (radius1 + radius2)
    else:
        rsquared = (radius1 - radius2)
        
    rsquared *= rsquared
    # (vx^2 + vy^2)*t^2 + (2*px*vx + 2*py*vy)*t + px^2 + py^2 - rsquared == 0
    
    return quadratic.solveQuad(
                    np.dot(relativeVelocity, relativeVelocity),
                    2.0 * np.dot(relativeVelocity, relativePosition),
                    np.dot(relativePosition, relativePosition) - rsquared)


class StraightPathSolution:
    """
    Holds _solution to the hitTargetAtSpeed problem.
    """

    def __init__(self, time, velocity, endPoint):
        self.time = time
        self.velocity = velocity
        self.endPoint = endPoint


def passTargetCircleAtSpeed(point, speed, center, velocity, radius, targetRadius):
    """
    Solves the problem of burshing a target circle's tangent, given a starting position and speed.
    
    This will brush the circle at radius, but continue past the circle and end at distance targetRadius from the circle.  
    This passing, allows a circle to be navigated around by calling this several times.  Other
    
    
    This always results in a 2 element solution vector.  Either/both solutions may be None to signify they are not possible.
    Solutions are ordered by which point is in the CCW vs. CW direction vs. the center.
    [CCW,CW] 
    
    """
    toCenter = center - point
    (toCenterUnit, distance) = unitAndLength(toCenter)
    
    # Starting point inside circle
    if distance < radius:
        return (None, None)
    
    length = math.sqrt(distance * distance - radius * radius) + math.sqrt(targetRadius * targetRadius - radius * radius)
    
    toCenterPerp = CCWNorm(toCenterUnit)
 
    y = radius / distance
    x = math.sqrt(1 - y * y)
    
    tangentPoint1 = point + (x * toCenterUnit + y * toCenterPerp) * length
    tangentPoint2 = point + (x * toCenterUnit - y * toCenterPerp) * length
    solution1 = hitTargetAtSpeed(point, speed, tangentPoint1, velocity)
    solution2 = hitTargetAtSpeed(point, speed, tangentPoint2, velocity)
    return (solution1, solution2)


def hitTargetAtSpeed(vehicleStart, vehicleSpeed, targetStartPoint, targetVelocity):
    """
    Solves the following problem:
    A vehicle starts at a position with a given speed.  A target starts at another position and has a given velocity.

    What is the velocity vector the vehicle should follow to hit the moving target?
    Where will the vehicle collide with the target?

    Again, we know the speed of the projectile, but not the velocity.

    :param vehicleStart: where craft starts
    :param vehicleSpeed: magnitude of craft's velocity
    :param targetStartPoint: the target's starting location
    :param targetVelocity: the velocity vector of the target
    :return: StraightPathSolution or None, if speed is insufficient given position
    """

    # Vector heading from startPoint towards point
    towards = targetStartPoint - vehicleStart
    towardsMagSquared = np.dot(towards, towards)
    velDotTowards = np.dot(towards, targetVelocity)

    # We need a mixture of velocity and the towardsVector.  Its magnitude needs to be vehicleSpeed.
    # heading = velocity + towardsFactor * towardsVector.
    # Solve for towardsFactor:

    a = -towardsMagSquared
    b = -2 * velDotTowards
    c = vehicleSpeed * vehicleSpeed - np.dot(targetVelocity, targetVelocity)

    solutions = quadratic.solveQuad(a, b, c)

    # If there are 2 solutions we want the larger positive solution.
    # There may be zero solutions of a solution of infinity if a projectile's speed matches the target's exactly.
    # If num solutions or negative time, when speed is too slow to catch projectile
    if len(solutions) == 0:
        return None
    else:
        towardsFactor = solutions[0]

    if towardsFactor < 0.0:
        return None

    # We can easily calculate time (t) from towardsFactor
    time = 1.0 / towardsFactor

    velocity = towards * towardsFactor + targetVelocity
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

    
def biProjection(unitVec1, unitVec2):
    """
    Given 2 unit vectors, find a vector which projects, exactly length 1.0 along both vectors.
    """
    if np.array_equal(unitVec1, unitVec2):
        return unitVec1
    
    along = np.dot(unitVec1, unitVec2)
    if along == -1.0:
        return None
    
    unitVec1Perp = unit(unitVec2 - along * unitVec1)
    
    return unitVec1 + unitVec1Perp * (1.0 - along) / np.dot(unitVec2, unitVec1Perp)


def calcShell(points, width, maxVertexExpansionRatio=NFZ_MAX_NEW_VERTEX_EXPANSION_RATIO):
    
    shellPoints = []
    for i in range(-1, len(points) - 1):
        n1 = unit(CWNorm(points[i] - points[i - 1]))
        n2 = unit(CWNorm(points[i + 1] - points[i]))
        vertexOffset = biProjection(n1, n2)
        if length(vertexOffset) > maxVertexExpansionRatio:
            vertexNormal = calcVertexNormal(points[i - 1], points[i], points[i + 1])
            vertexOffset1 = biProjection(n1, vertexNormal)
            vertexOffset2 = biProjection(n2, vertexNormal)
            shellPoints.append(points[i] + width * vertexOffset1)
            shellPoints.append(points[i] + width * vertexOffset2)
        else:
            shellPoints.append(points[i] + width * vertexOffset)
    return shellPoints


def calcVertexNormal(prevVertex, vertex, nextVertex):
    """
    Calculate the point's normal (average of normals of connected edges) assuming CCW winding
    """
    n1 = unit(CWNorm(vertex - prevVertex))
    n2 = unit(CWNorm(nextVertex - vertex))
    return unit(n1 + n2)


def calcVertexAngle(prevVertex, vertex, nextVertex):
    """
    Calculate the inside angle at a vertex in a polygon assuming CCW winding
    """
    dir1 = unit(vertex - prevVertex)
    dir2 = unit(vertex - nextVertex)
    dir3 = unit(nextVertex - prevVertex)

    cosAngle = np.dot(dir1, dir2)

    angle = math.acos(cosAngle)

    if cross2(dir1, dir3) > 0.0:
        return angle
    else:
        return math.pi * 2 - angle


def woundCCW(points):
    angle = 0
    for i in range(-1, len(points) - 1):
        prevVertex = points[i - 1]
        currVertex = points[i]
        nextVertex = points[i + 1]

        angle += calcVertexAngle(prevVertex, currVertex, nextVertex)
    # If the points are wound CCW then this should given an answer following the standard formula for angle inside a polygon.
    # If wound CW, we will get a much bigger number (4*pi bigger).  We add a small margin of error (0.1) to the calculation.
    if angle < math.pi * (len(points) - 2) + 0.1:
        return True
    else:
        return False


def cross2(v1, v2):
    return v1[0] * v2[1] - v1[1] * v2[0]


def CCWNorm(vec):
    """
    Rotate vec 90 degrees counter-clockwise.
    :param vec:
    :return:
    """
    return np.array([-vec[1], vec[0]], np.double)


def CWNorm(vec):
    """
    Rotate vec 90 degrees clockwise.
    :param vec:
    :return:
    """
    return np.array([vec[1], -vec[0]], np.double)


def dirNorm(vec, rotDirection):
    return np.array([-rotDirection * vec[1], rotDirection * vec[0]], np.double)
    

def rotate2d(point, angle):
    """Rotate CCW by angle"""
    cosAngle = math.cos(angle)
    sinAngle = math.sin(angle)
    return rotate2dtrig(point, cosAngle, sinAngle)


def rotate2dtrig(vec, cosAngle, sinAngle):
    """Rotate CCW by angle"""
    return np.array([vec[0] * cosAngle - vec[1] * sinAngle, vec[1] * cosAngle + vec[0] * sinAngle], np.double)


def relativeAngle(startVec, endVec, rotDirection=1.0):
    """
    How far you would have to turn, to go from startVec to endVec.  This will be in the range (-2*pi, 2*pi).
    :param startVec:
    :param endVec:
    :param rotDirection: CCW = 1, CW = -1
    :return:
    """
    return rotDirection * (angleOfVector(endVec) - angleOfVector(startVec))
    # sinRotate = startVec[0] * endVec[1] - startVec[1] * endVec[0]
    # if np.dot(startVec, endVec) < 0.0:
    #     return math.pi - math.asin(sinRotate)
    # else:
    #     return math.asin(sinRotate)


def modAngle(angle, lowAngle):
    """
    Return angle in the range: [lowAngle, 2*pi+lowAngle)
    :param angle:
    :param lowAngle:
    :return:
    """
    angle = math.fmod(angle, 2.0 * math.pi)
    if angle < lowAngle:
        return angle + 2.0 * math.pi
    elif angle >= lowAngle + 2.0 * math.pi:
        return angle - 2.0 * math.pi
    else:
        return angle


def modAngleSigned(angle):
    """
    Return angle in the range: [-pi,pi)
    :param angle:
    :return:
    """
    return modAngle(angle, -math.pi)


def modAngleUnsigned(angle):
    """
    Return angle in the range: [0,2*pi)
    :param angle:
    :return:
    """
    return modAngle(angle, 0.0)


def isAngleInArcCCW(angle, start, length):
    """
    Is angle within the given CCW arc (start,length)
    :param angle:
    :param start:
    :param length:
    :return:
    """
    return modAngleUnsigned(angle - start) <= length


def clampAngleCCW(angle, start, length):
    """
    Given a CCW arc (start, length), clamp angle to be within the arc.
    :param angle:
    :param start:
    :param length:
    :return:
    """
    if isAngleInArcCCW(angle, start, length):
        return angle

    diff = modAngleSigned(angle - start + length / 2.0)
    if diff > 0.0:
        return start + length
    else:
        return start


def angleOfVector(vector, rotDirection=1.0):
    """
    Angle of a given vector [0,2*pi).  Defaults to CCW.
    :param vector:
    :param rotDirection: CCW = 1, CW = -1
    :return: corresponding unit vector
    """
    return modAngleUnsigned(rotDirection * math.atan2(vector[1], vector[0]))


def unitVectorOfAngle(angle, rotDirection=1.0):
    """
    Unit vector corresponding to a given angle.  Defaults to CCW.
    :param angle:
    :param rotDirection: CCW = 1, CW = -1
    :return: corresponding unit vector
    """
    angle = angle * rotDirection
    return np.array([math.cos(angle), math.sin(angle)], np.double)


def unit(vector):
    length = np.linalg.norm(vector)
    if length == 0.0:
        raise Exception("Norm of 0-length vector!")
    return vector / length


def unitAndLength(vector):
    length = np.linalg.norm(vector)
    if length == 0.0:
        return (vector, length)
    return (vector / length, length)


def length(vector):
    return np.linalg.norm(vector)


def lengthSquared(vector):
    return np.dot(vector, vector)


def changeBasis(vec, x, y):
    return (np.dot(vec, x), np.dot(vec, y))


def clampMag(value, maximum):
    if value < -maximum:
            return -maximum
    elif value > maximum:
            return maximum
    return value