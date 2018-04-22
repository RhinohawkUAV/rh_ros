import numpy as np

from gui import Drawable
from subGUI import SubGUI


class NoFlyMover(Drawable, SubGUI):
    def __init__(self, obstacleCourseEdit):
        self._startPoint = None
        self._obstacleCourseEdit = obstacleCourseEdit
        self._dragZones = []
        self._previewZones = []

    def onLeftPress(self, point, control=False):
        self._startPoint = point
        if control:
            self._dragZones = self._obstacleCourseEdit.findInsideNoFlyZones(self._startPoint)
        else:
            self._dragZones = self._obstacleCourseEdit.removeInsideNoFlyZones(self._startPoint)
            offset = np.array([0, 0], np.double)
            for dragZone in self._dragZones:
                self._previewZones.append(dragZone.getTranslatedCopy(offset))

    def onLeftRelease(self, point, control=False):
        self.onMotion(point, control)
        self._startPoint = None
        del self._dragZones[:]
        if not control:
            self._obstacleCourseEdit.addNoFlyZones(self._previewZones)

            del self._previewZones[:]

    def onMotion(self, point, control=False):
        if self._startPoint is not None:
            offset = point - self._startPoint
            if control:
                for dragZone in self._dragZones:
                    dragZone.velocity = offset
            else:

                del self._previewZones[:]
                for dragZone in self._dragZones:
                    self._previewZones.append(dragZone.getTranslatedCopy(offset))

    def onKey(self, point, key, ctrl=False):
        if key == "Delete":
            self._obstacleCourseEdit.removeInsideNoFlyZones(point)

    def draw(self, canvas, **kwargs):
        for previewZone in self._previewZones:
            previewZone.draw(canvas, **kwargs)
