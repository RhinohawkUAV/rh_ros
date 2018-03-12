from utils import quadratic


def hitTargetAtSpeed(projectileStart, projectileSpeed, targetStartPoint, targetVelocity):
    """
    Solves the following problem:
    A projectile starts at a position with a given speed.  A target starts at another position and has a given _velocity.

    What is the _velocity vector the projectile should follow to hit the moving target?
    Where will the projectile collide with the target?

    Note: We start knowing the speed of the projectile, but not the direction.

    :param projectileStart: where craft starts
    :param projectileSpeed: magnitude of craft's _velocity
    :param targetStartPoint: the target's starting location
    :param targetVelocity: the _velocity vector of the target
    :return: ((velocityX,velocityY),(collisionX,collisionY)) or None, if speed is insufficient given position
    """

    # Vector heading from start towards point
    towardsX = targetStartPoint[0] - projectileStart[0]
    towardsY = targetStartPoint[1] - projectileStart[1]
    towardsMagSquared = towardsX * towardsX + towardsY * towardsY
    velDotTowards = towardsX * targetVelocity[0] + towardsY * targetVelocity[1]

    # We need a mixture of _velocity and the towardsVector.  Its magnitude needs to be projectileSpeed.
    # heading = _velocity + towardsFactor * towardsVector.
    # Solve for towardsFactor:

    a = -towardsMagSquared
    b = - 2 * velDotTowards
    c = projectileSpeed * projectileSpeed - (
            targetVelocity[0] * targetVelocity[0] + targetVelocity[1] * targetVelocity[1])

    # We always want the 1st solution
    towardsFactor = quadratic.solveQuad(a, b, c)

    # Error!
    # Note: 0 solutions, when projectileSpeed is too low compared to point's _velocity
    # A solution of infinity when projectileSpeed matches _velocity exactly
    if len(towardsFactor) == 0 or towardsFactor[0] == 0.0:
        return None

    # We can easily calculate time (t) from towardsFactor
    t = 1.0 / towardsFactor[0]

    velocity = (towardsX * towardsFactor[0] + targetVelocity[0], towardsY * towardsFactor[0] + targetVelocity[1])
    collision = (targetStartPoint[0] + t * targetVelocity[0], targetStartPoint[1] + t * targetVelocity[1])

    return (velocity, collision)
