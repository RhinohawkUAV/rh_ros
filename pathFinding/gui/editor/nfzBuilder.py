from gui import Drawable
from gui.editor.noFlyZoneEdit import NoFlyZoneEdit
from gui.editor.polyBuilder import PolyBuilder


class NFZBuilder(PolyBuilder, Drawable):
    def __init__(self, initialPathFindingEdit):
        PolyBuilder.__init__(self)
        self._initialPathFindingEdit = initialPathFindingEdit

    def _polyBuilt(self, points):
        self._initialPathFindingEdit.addNoFlyZone(NoFlyZoneEdit(points, [0, 0]))
