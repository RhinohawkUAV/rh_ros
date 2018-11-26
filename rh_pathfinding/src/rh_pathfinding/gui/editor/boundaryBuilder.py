from engine.interface.fileUtils import SCENARIO_KEY
from polyBuilder import PolyBuilder


class BoundaryBuilder(PolyBuilder):

    def __init__(self):
        PolyBuilder.__init__(self)

    def _polyBuilt(self, points):
        self._scenario.boundaryPoints = points

    def draw(self, visualizer, color="red", **kwargs):
        PolyBuilder.draw(self, visualizer, color=color)
