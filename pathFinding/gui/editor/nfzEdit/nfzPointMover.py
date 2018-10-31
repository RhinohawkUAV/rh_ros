from gui import Drawable


class NFZPointMover(Drawable):

    def __init__(self):
        self._pointIndex = None
        self._noFlyZone = None
        self._alteredNoFlyZone = None

    def onPress(self, point, nfzEdit):
        (self._pointIndex, self._noFlyZone) = nfzEdit.findClosestPointIndex(point)
        nfzEdit.removeNoFlyZone(self._noFlyZone)
        self._alteredNoFlyZone = self._noFlyZone.getPointTranslatedCopy(self._pointIndex, point)

    def onRelease(self, point, nfzEdit):
        self._alteredNoFlyZone = self._noFlyZone.getPointTranslatedCopy(self._pointIndex, point)
        nfzEdit.addNoFlyZone(self._alteredNoFlyZone)
        self._noFlyZone = None
        self._alteredNoFlyZone = None

    def onMotion(self, point, nfzEdit):
        if self._noFlyZone is None:
            pass
        else:
            self._alteredNoFlyZone = self._noFlyZone.getPointTranslatedCopy(self._pointIndex, point)

    def draw(self, visualizer, **kwargs):
        if not self._alteredNoFlyZone is None:
            self._alteredNoFlyZone.draw(visualizer, **kwargs)

