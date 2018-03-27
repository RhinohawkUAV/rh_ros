from gui import Drawable
from subGUI import SubGUI


class NFZPointMover(Drawable, SubGUI):
    def __init__(self, parent, obstacleCourse):
        self._parent = parent
        self._obstacleCourse = obstacleCourse
        self._pointIndex = None
        self._noFlyZone = None
        self._alteredNoFlyZone = None

    def onLeftPress(self, point, control=False):
        (self._pointIndex, self._noFlyZone) = self._obstacleCourse.findClosest(point)
        self._obstacleCourse.removeNoFlyZone(self._noFlyZone)
        self._alteredNoFlyZone = self._noFlyZone.getPointTranslatedCopy(self._pointIndex, point)

    def onLeftRelease(self, point, control=False):
        self._alteredNoFlyZone = self._noFlyZone.getPointTranslatedCopy(self._pointIndex, point)
        self._obstacleCourse.addNoFlyZone(self._alteredNoFlyZone)
        self._noFlyZone = None

    def onMotion(self, point, control=False):
        if self._noFlyZone is None:
            pass
        else:
            self._alteredNoFlyZone = self._noFlyZone.getPointTranslatedCopy(self._pointIndex, point)

    def draw(self, canvas, **kwargs):
        if not self._alteredNoFlyZone is None:
            self._alteredNoFlyZone.draw(canvas, **kwargs)
