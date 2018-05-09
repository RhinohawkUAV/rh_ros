from gui import Drawable
from gui.editor.polyBuilder import PolyBuilder
from engine.interface.noFlyZoneInput import NoFlyZoneInput


class NFZBuilder(PolyBuilder, Drawable):
    def __init__(self):
        PolyBuilder.__init__(self)

    def _polyBuilt(self, points):
        self._debugInput.scenario.noFlyZones.append(NoFlyZoneInput(points, (0.0,0.0)))

