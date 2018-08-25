from engine.geometry.obstacle.target import Target


class CircularTarget(Target):

    def __init__(self, startPosition, velocity, radius, offset):
        Target.__init__(self, startPosition, velocity)
        self.radius = radius + offset / 2.0
        self.outerRadius = radius + offset
