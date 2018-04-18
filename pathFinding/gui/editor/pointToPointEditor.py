from gui import Drawable
from pointListEditor import PointListEditor
from subGUI import SubGUI


class PointToPointEditor(Drawable, SubGUI):
    def __init__(self, pointToPointEdit, **kwargs):
        self._pointToPointEdit = pointToPointEdit
        self._pointListEditor = PointListEditor(self._pointToPointEdit.points, **kwargs)

    def onLeftPress(self, point, control=False):
        self._pointListEditor.onLeftPress(point, control)

    def onLeftRelease(self, point, control=False):
        self._pointListEditor.onLeftRelease(point, control)

    def onMotion(self, point, control=False):
        self._pointListEditor.onMotion(point, control)

    def onKey(self, point, key, ctrl=False):
        if key == "v":
            if len(self._pointToPointEdit.points) > 0:
                self._pointToPointEdit.startVelocity = point - self._pointToPointEdit.points[0]
