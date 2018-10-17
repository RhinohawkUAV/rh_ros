from engine.geometry import  calcs
from engine.interface.dynamicNoFlyZone import DynamicNoFlyZone
import gui
from gui.draw import VELOCITY_TO_PIXEL
from gui.editor.subGUI import SubGUI
import numpy as np


class DNFZBuilder(SubGUI):

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
            self._scenario.dynamicNoFlyZones.append(dnfz)
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
            self._previewVelocity = self._visualizer.scaleVecToPixels(point - self._previewCenter) / VELOCITY_TO_PIXEL

    def onKey(self, point, key, ctrl=False):
        if key == "Delete":
            keepDNFZs = []
            for dnfz in self._scenario.dynamicNoFlyZones:
                if not calcs.isPointInCircle(dnfz.center, dnfz.radius, point):
                    keepDNFZs.append(dnfz)
            self._scenario.dynamicNoFlyZones = keepDNFZs
    
    def draw(self, visualizer, **kwargs):
        SubGUI.draw(self, visualizer, **kwargs)
        if self._previewCenter is not None:
            gui.draw.drawCircle(visualizer, self._previewCenter, self._previewRadius)
            gui.draw.drawVelocity(visualizer, self._previewCenter, self._previewVelocity)

