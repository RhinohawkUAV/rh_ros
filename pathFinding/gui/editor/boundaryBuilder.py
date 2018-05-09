from polyBuilder import PolyBuilder
from subGUI import SubGUI


class BoundaryBuilder(PolyBuilder, SubGUI):

    def __init__(self):
        PolyBuilder.__init__(self)

    def _polyBuilt(self, points):
        self._debugInput.scenario.boundaryPoints = points

    def draw(self, canvas, color="red", **kwargs):
        PolyBuilder.draw(self, canvas, color=color)
