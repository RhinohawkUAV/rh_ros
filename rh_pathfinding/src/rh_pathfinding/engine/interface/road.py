import numpy as np


class Road:

    def __init__(self, startPoint, endPoint, width):
        self.startPoint = np.array(startPoint, np.double)
        self.endPoint = np.array(endPoint, np.double)
        self.width = width
