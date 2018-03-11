from utils import quadratic


def hitTargetAtSpeed(projectileStart, projectileSpeed, targetStartPoint, targetVelocity):
    """
    Solves the following problem:
    A projectile starts at a position with a given speed.  A target starts at another position and has a given velocity.

    What is the velocity vector the projectile should follow to hit the moving target?
    Where will the projectile collide with the target?

    Note: We start knowing the speed of the projectile, but not the direction.

    :param projectileStart: where craft starts
    :param projectileSpeed: magnitude of craft's velocity
    :param targetStartPoint: the target's starting location
    :param targetVelocity: the velocity vector of the target
    :return: ((velocityX,velocityY),(collisionX,collisionY)) or None, if speed is insufficient given position
    """

    # Vector heading from start towards point
    towardsX = targetStartPoint[0] - projectileStart[0]
    towardsY = targetStartPoint[1] - projectileStart[1]
    towardsMagSquared = towardsX * towardsX + towardsY * towardsY
    velDotTowards = towardsX * targetVelocity[0] + towardsY * targetVelocity[1]

    # We need a mixture of velocity and the towardsVector.  Its magnitude needs to be projectileSpeed.
    # heading = velocity + towardsFactor * towardsVector.
    # Solve for towardsFactor:

    a = -towardsMagSquared
    b = - 2 * velDotTowards
    c = projectileSpeed * projectileSpeed - (
            targetVelocity[0] * targetVelocity[0] + targetVelocity[1] * targetVelocity[1])

    # We always want the 1st solution
    towardsFactor = quadratic.solveQuad(a, b, c)

    # Error!
    # Note: 0 solutions, when projectileSpeed is too low compared to point's velocity
    # A solution of infinity when projectileSpeed matches velocity exactly
    if len(towardsFactor) == 0 or towardsFactor[0] == 0.0:
        return None

    # We can easily calculate time (t) from towardsFactor
    t = 1.0 / towardsFactor[0]

    velocity = (towardsX * towardsFactor[0] + targetVelocity[0], towardsY * towardsFactor[0] + targetVelocity[1])
    collision = (targetStartPoint[0] + t * targetVelocity[0], targetStartPoint[1] + t * targetVelocity[1])

    return (velocity, collision)


def checkRayIntersectLine(rayStart, rayDir, p1, p2, n, invTan):
    """
    Shapely can't do the whole 1-sided line thing.  I'm figuring the code below will be converted to C, or numpy arrays
    for speed at some point.

    Does a ray intersect a one-sided line segment?
    Only considered an intersection if dir is opposed the normal vector of the line.
    Parallel does not count.
    Intersection with an endpoint does count.

    TODO: Consider
    These decisions more or less give us the desired outcomes when considering tracings around the outside of a polygon.
    The only bad case is when parallel segments of different polygons align exactly.
    We may want to add a tolerance to be sure that we can't ever pick a line exactly through 2 endpoints of a line segments in a polygon

    Shapes should have outward facing normals.
    :param rayStart: (x,y) start position
    :param rayDir: (x,y) direction of the ray (does NOT have to be normalized)
    :param p1: (x,y) 1st point of the line
    :param p2: (x,y) 2nd point of the line
    :param n: (x,y) The line's unit normal. Line is one-sided (ie. normals point outward from a shape)
    :param invTan (x,y) Faces in the direction from p1 to p2, but with magnitude = 1/|p2-p1|
    :return: is there an intersection
    """

    pdiffx = p1[0] - rayStart[0]
    pdiffy = p1[1] - rayStart[1]

    # Distance to line segment in the direction of the normal.  If this is positive then the ray's start position is
    #  "behind" the line-segment and no intersection is possible.
    normalDistance = (n[0] * pdiffx + n[1] * pdiffy)
    if normalDistance > 0.0:
        return False

    # Magnitude of direction in the direction of the normal.  If this is positive then the direction is away from the
    # line segment and no collision is possible.  If this is 0 then direction is parallel and no collision is possible
    # (by our convention)
    normalDirection = (n[0] * rayDir[0] + n[1] * rayDir[1])

    if normalDirection >= 0.0:
        return False

    # time to intersection is based on distance from line and speed in direction of the normal (speed against normal since it faces outwards)
    # dot(n,pdiff) / dot(n,vdiff)
    t = normalDistance / normalDirection

    # While moving towards the line perpendicularly, we also moved along the line tangentially.
    # We don't care how far we moved in absolute space, only in "tangent-unit-space".  In this space p1 is 0, p2 is 1.
    # The provided invTan vector points in the direction from p1 to p2 with the inverse magnitude of the length between them.

    # tan = dot(invTan, rayStart - p1 + rayDir * t)
    tan = (-pdiffx + rayDir[0] * t) * invTan[0] + (-pdiffy + rayDir[1] * t) * invTan[1]

    return tan >= 0 and tan <= 1
