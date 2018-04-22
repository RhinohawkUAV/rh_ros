from gui import Drawable
from gui.editor.noFlyZoneEdit import NoFlyZoneEdit
from gui.editor.polyBuilder import PolyBuilder


class NFZBuilder(PolyBuilder, Drawable):
    def __init__(self, obstacleCourseEdit):
        PolyBuilder.__init__(self)
        self._obstacleCourseEdit = obstacleCourseEdit

    def _polyBuilt(self, points):
        self._obstacleCourseEdit.addNoFlyZone(NoFlyZoneEdit(points, [0, 0]))
