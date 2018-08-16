import testmod
from engine.geometry.obstacle.arcFinder.arcPathSegment import ArcPathSegment
from engine.geometry.obstacle.intersectionDetector.obstacleLineSegment import ObstacleLineSegment
import numpy as np

# testmod.test_print("print this")
p1 = np.array((1.0, 2.0), np.double)
p2 = np.array((3.0, 4.0), np.double)
velocity = np.array((5.0, 6.0), np.double)
segment = ObstacleLineSegment(p1, p2, velocity)
testmod.readList([segment])

