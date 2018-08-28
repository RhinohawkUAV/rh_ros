from engine.geometry.obstacle.intersectionDetector.lineSegmentObstacle import LineSegmentObstacle
import fastPathIntersect
import numpy as np

fastPathIntersect.test_print("print this")
p1 = np.array((1.0, 2.0), np.double)
p2 = np.array((3.0, 4.0), np.double)
velocity = np.array((5.0, 6.0), np.double)
segment = LineSegmentObstacle(p1, p2, velocity)

