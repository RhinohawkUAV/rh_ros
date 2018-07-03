from engine.geometry import  calcs
from engine.interface.dynamicNoFlyZone import DynamicNoFlyZone
from engine.interface.fileUtils import SCENARIO_KEY
from gui import Drawable
import gui
from gui.editor.subGUI import SubGUI
import numpy as np


class DNFZBuilder(Drawable, SubGUI):

    def __init__(self):
        self._previewCenter = None
        self._previewRadius = 0.0
        self._previewVelocity = np.array((0, 0), np.double)
        self._inputMode = 0
        
    def onLeftRelease(self, point, control=False):
        if self._inputMode == 0:
            self._previewCenter = point
            self._inputMode += 1
        elif self._inputMode == 1:
            self._previewRadius = np.linalg.norm(point - self._previewCenter)
            self._inputMode += 1
        else:
            dnfz = DynamicNoFlyZone(self._previewCenter, self._previewRadius, self._previewVelocity)
            self._inputDict[SCENARIO_KEY].dynamicNoFlyZones.append(dnfz)
            self._previewCenter = None
            self._previewRadius = 0.0
            self._previewVelocity = np.array((0, 0), np.double)
            self._inputMode = 0

    def onMotion(self, point, control=False):
        if self._inputMode == 0:
            self._previewCenter = point
            self._previewRadius = 0.0
        elif self._inputMode == 1:
            self._previewRadius = np.linalg.norm(point - self._previewCenter)
        else:
            self._previewVelocity = (point - self._previewCenter) / gui.draw.VELOCITY_SCALE

    def onKey(self, point, key, ctrl=False):
        if key == "Delete":
            keepDNFZs = []
            for dnfz in self._inputDict[SCENARIO_KEY].dynamicNoFlyZones:
                if not calcs.isPointInCircle(dnfz.center, dnfz.radius, point):
                    keepDNFZs.append(dnfz)
            self._inputDict[SCENARIO_KEY].dynamicNoFlyZones = keepDNFZs
    
    def draw(self, canvas, **kwargs):
        if self._previewCenter is not None:
            gui.draw.drawCircle(canvas, self._previewCenter, self._previewRadius)
            gui.draw.drawVelocity(canvas, self._previewCenter, self._previewVelocity)

