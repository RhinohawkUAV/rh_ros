from gui import Drawable
import numpy as np


class NoFlyMover(Drawable):

    def __init__(self):
        self._startPoint = None
        self._dragZones = None
        self._previewZones = []

    def onPress(self, point, nfzEdit):
        self._startPoint = point
        self._dragZones = nfzEdit.removeInsideNoFlyZones(self._startPoint)
        offset = np.array([0, 0], np.double)
        self._previewZones = []
        for dragZone in self._dragZones:
            self._previewZones.append(dragZone.getTranslatedCopy(offset))

    def onRelease(self, point, nfzEdit):
        self.onMotion(point, nfzEdit)
        nfzEdit.addNoFlyZones(self._previewZones)
        self._startPoint = None
        self._dragZones = None
        self._previewZones = []

    def onMotion(self, point, nfzEdit):
        if self._startPoint is not None:
            offset = point - self._startPoint
            self._previewZones = []
            for dragZone in self._dragZones:
                self._previewZones.append(dragZone.getTranslatedCopy(offset))

    def draw(self, canvas, **kwargs):
        for previewZone in self._previewZones:
            previewZone.draw(canvas, **kwargs)
