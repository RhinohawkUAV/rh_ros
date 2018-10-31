import numpy as np


class TestScenario():
    """
    Input used in testing.  Defines the problem of finding a PathSegment from 
    a starting point with an initial velocity to a moving target.
    """

    def __init__(self, startPoint=(5.0, 5.0), startVelocity=(1.0, 1.0), targetPoint=(95.0, 95.0), velocityOfTarget=(0.0, 0.0)):
        self.startPoint = np.array(startPoint, np.double)
        self.startVelocity = np.array(startVelocity, np.double)
        self.targetPoint = np.array(targetPoint, np.double)
        self.velocityOfTarget = np.array(velocityOfTarget, np.double)
