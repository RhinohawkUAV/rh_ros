import math

from engine.geometry import calcs
from engine.geometry.obstacle.target import Target
from gui.draw import DEFAULT_DASH
import gui.draw
import numpy as np


class VertexTarget(Target):

    def __init__(self, startPosition, velocity, normal, vertexAngle):
        Target.__init__(self, startPosition, velocity)
        self.normal = normal
        self.lowerCosLimit = math.cos(math.pi - vertexAngle / 2.0)
        self.upperCosLimit = math.cos(vertexAngle / 2.0)
    
    def testEntryVelocity(self, entryVelocity):
        """
        Is the given absolute velocity OK?  Illegal velocities are ones which are headed "into" the shape and would cause an immediate collision.
        """
        relativeVelocity = calcs.unit(entryVelocity - self.velocity)
        cosEntryAngle = np.dot(relativeVelocity, self.normal)
        return cosEntryAngle <= self.upperCosLimit and cosEntryAngle >= self.lowerCosLimit 

    def draw(self, canvas, **kwargs):
        gui.draw.drawCircle(canvas, self.position, 4.0, dash=DEFAULT_DASH)
