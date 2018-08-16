from engine.geometry.obstacle.target import Target


class CircularTarget(Target):

    def __init__(self, startPosition, velocity, radius):
        Target.__init__(self, startPosition, velocity)
        self.radius = radius
