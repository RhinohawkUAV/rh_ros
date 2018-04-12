from polyBuilder import PolyBuilder
from subGUI import SubGUI


class BoundaryBuilder(PolyBuilder, SubGUI):
    def __init__(self, staticPathFindingEdit):
        PolyBuilder.__init__(self)
        self._staticPathFindingEdit = staticPathFindingEdit

    def _polyBuilt(self, points):
        self._staticPathFindingEdit.boundaryPoints = points

    def draw(self, canvas, color="red", **kwargs):
        PolyBuilder.draw(self, canvas, color=color)
