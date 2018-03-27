import numpy as np

from gui import Drawable
from subGUI import SubGUI


class NoFlyMover(Drawable, SubGUI):
    def __init__(self, obstacleCourse):
        self._startPoint = None
        self._obstacleCourse = obstacleCourse
        self._dragZones = []
        self._previewZones = []

    def onLeftPress(self, point, control=False):
        self._startPoint = point
        self._dragZones = self._obstacleCourse.removeInsideNoFlyZones(self._startPoint)
        offset = np.array([0, 0], np.double)
        for dragZone in self._dragZones:
            self._previewZones.append(dragZone.getTranslatedCopy(offset))

    def onLeftRelease(self, point, control=False):
        self.onMotion(point, control)
        self._obstacleCourse.addNoFlyZones(self._previewZones)
        self._startPoint = None
        del self._dragZones[:]
        del self._previewZones[:]

    def onMotion(self, point, control=False):
        if self._startPoint is not None:
            offset = point - self._startPoint
            del self._previewZones[:]
            for dragZone in self._dragZones:
                self._previewZones.append(dragZone.getTranslatedCopy(offset))

    def onKey(self, point, key):
        if key == "Delete":
            self._obstacleCourse.removeInsideNoFlyZones(point)

    def draw(self, canvas, **kwargs):
        for previewZone in self._previewZones:
            previewZone.draw(canvas, **kwargs)
