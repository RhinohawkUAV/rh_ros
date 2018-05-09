import numpy as np


class TestEdit:
    """
    Holds the state of the tester used in the editor gui.  This object is saved/loaded allowing for recovering past tests.
    """

    def __init__(self, startPoint, startVelocity, targetPoint, velocityOfTarget):
        self.startPoint = np.array(startPoint, np.double)
        self._startVelocity = np.array(startVelocity, np.double)
        self.targetPoint = np.array(targetPoint, np.double)
        self.velocityOfTarget = np.array(velocityOfTarget, np.double)

    def set(self, testEdit):
        self.startPoint = testEdit.startPoint
        self._startVelocity = testEdit._startVelocity
        self.targetPoint = testEdit.targetPoint
        self.velocityOfTarget = testEdit.velocityOfTarget


def fromJSONDict(objDict):
    return TestEdit(objDict["startPoint"], objDict["_startVelocity"], objDict["targetPoint"],
                    objDict["velocityOfTarget"])
