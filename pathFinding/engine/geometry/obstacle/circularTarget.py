import constants
from engine.geometry.obstacle.target import Target
from gui.draw import DEFAULT_DASH
import gui.draw


class CircularTarget(Target):

    def __init__(self, startPosition, velocity, radius, nfzBufferWidth, nfzTargetOffset):
        Target.__init__(self, startPosition, velocity)
        self.radius = radius + nfzBufferWidth + constants.IDENTICAL_POINT_TOLERANCE
        self.targetRadius = radius + nfzTargetOffset

    def draw(self, canvas, **kwargs):
        gui.draw.drawCircle(canvas, self.position, self.targetRadius, dash=DEFAULT_DASH)
