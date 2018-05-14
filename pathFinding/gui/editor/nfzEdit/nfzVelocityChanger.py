from gui import Drawable
from gui.draw import VELOCITY_SCALE
import numpy as np


class NFZVelocityChanger(Drawable):

    def __init__(self):
        self._dragZones = None
        self._startPoint = None

    def onPress(self, point, nfzEdit):
        self._startPoint = point
        self._dragZones = nfzEdit.findInsideNoFlyZones(self._startPoint)
        self.onMotion(point, nfzEdit)

    def onRelease(self, point, nfzEdit):
        self.onMotion(point, nfzEdit)
        self._startPoint = None
        self._dragZones = None

    def onMotion(self, point, nfzEdit):
        if self._startPoint is not None:
            offset = point - self._startPoint
            for dragZone in self._dragZones:
                dragZone.velocity = offset / VELOCITY_SCALE

    def draw(self, canvas, **kwargs):
        pass
