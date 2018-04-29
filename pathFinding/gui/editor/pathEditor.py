from engine.geometry import calcs
from gui import Drawable
from pointListEditor import PointListEditor
from subGUI import SubGUI


class pathEditor(Drawable, SubGUI):
    def __init__(self, pathEdit, **kwargs):
        self._pathEdit = pathEdit
        self._pointListEditor = PointListEditor(self._pathEdit.points, **kwargs)

    def onLeftPress(self, point, control=False):
        self._pointListEditor.onLeftPress(point, control)

    def onLeftRelease(self, point, control=False):
        self._pointListEditor.onLeftRelease(point, control)

    def onMotion(self, point, control=False):
        self._pointListEditor.onMotion(point, control)

    def onKey(self, point, key, ctrl=False):
        if key == "v":
            if len(self._pathEdit.points) > 0:
                self._pathEdit.startVelocity = point - self._pathEdit.points[0]
        if key == "Delete":
            if len(self._pathEdit.points) > 0:
                (distance, index) = calcs.findClosestPoint(point, self._pathEdit.points)
                self._pathEdit.points.pop(index)
