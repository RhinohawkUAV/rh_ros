import math

from engine.geometry import calcs
from engine.geometry.obstacle.target import Target
import numpy as np


class VertexTarget(Target):

    def __init__(self, startPosition, velocity, normal, vertexAngle):
        Target.__init__(self, startPosition, velocity)
        self.normal = normal
        self.cosLimit = math.cos((math.pi - vertexAngle) / 2.0)
    
    def testEntryVelocity(self, entryVelocity):
        """
        Is the given absolute velocity OK?  Illegal velocities are ones which are headed "into" the shape and would cause an immediate collision.
        """
        relativeVelocity = calcs.unit(entryVelocity - self.velocity)
        return np.dot(relativeVelocity, self.normal) >= self.cosLimit
    
