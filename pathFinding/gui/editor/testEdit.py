class TestEdit:
    """
    Holds the state of the tester used in the editor gui.  This object is saved/loaded allowing for recovering past tests.
    """

    def __init__(self, startPoint, startVelocity, targetPoint, targetVelocity):
        self._startPoint = startPoint
        self._startVelocity = startVelocity
        self._targetPoint = targetPoint
        self._targetVelocity = targetVelocity

        # self._startPoint = np.array((5, 5), np.double)
        # self._startVelocity = np.array((1, 1), np.double)
        # self._targetPoint = np.array((95, 95), np.double)
        # self._velocityOfTarget = np.array((0, 0), np.double)
