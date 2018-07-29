from engine.interface.fileUtils import SCENARIO_KEY
from polyBuilder import PolyBuilder


class BoundaryBuilder(PolyBuilder):

    def __init__(self):
        PolyBuilder.__init__(self)

    def _polyBuilt(self, points):
        self._inputDict[SCENARIO_KEY].boundaryPoints = points

    def draw(self, canvas, color="red", **kwargs):
        PolyBuilder.draw(self, canvas, color=color)
