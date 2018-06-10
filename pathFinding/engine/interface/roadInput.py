import numpy as np

class RoadInput:
    def __init__(self, startPoint, lineEndPoint, width):
        self.startPoint = np.array(startPoint, np.double)
        self.lineEndPoint = np.array(lineEndPoint, np.double)
        self.width = width