from editableNoFlyZoneList import EditableNoFlyZoneList
from engine.interface.fileUtils import SCENARIO_KEY
from gui.editor.subGUI import SubGUI
from nfzPointMover import NFZPointMover
from nfzVelocityChanger import NFZVelocityChanger
from noFlyMover import NoFlyMover


class NFZEditor(SubGUI):

    def __init__(self):
        self._nfzEdit_nfzEdit = None
        self._pressVelocity = False
        self._pointMover = NFZPointMover()
        self._mover = NoFlyMover()
        self._velocityChanger = NFZVelocityChanger()

    def onSwitch(self, params, scenario, vehicle, testInput, visualizer):
        SubGUI.onSwitch(self, params, scenario, vehicle, testInput, visualizer)
        self._velocityChanger._visualizer = visualizer
        self._nfzEdit = EditableNoFlyZoneList(self._scenario.noFlyZones)
        self._scenario.noFlyZones = []
    
    def onExit(self):
        self._scenario.noFlyZones = self._nfzEdit.asInput()
        
    def onLeftPress(self, point, control=False):
        if control:
            self._pointMover.onPress(point, self._nfzEdit)
        else:
            self._mover.onPress(point, self._nfzEdit)

    def onLeftRelease(self, point, control=False):
        if control:
            self._pointMover.onRelease(point, self._nfzEdit)
        else:
            self._mover.onRelease(point, self._nfzEdit)

    def onMotion(self, point, control=False):
        if self._pressVelocity:
            self._velocityChanger.onMotion(point, self._nfzEdit)
        elif control:
            self._pointMover.onMotion(point, self._nfzEdit)
        else:
            self._mover.onMotion(point, self._nfzEdit)
    
    def onKey(self, point, key, ctrl=False):
        if key == "Delete":
            self._nfzEdit.removeInsideNoFlyZones(point)
        if key == "v":
            if not self._pressVelocity:
                self._velocityChanger.onPress(point, self._nfzEdit)
            else:
                self._velocityChanger.onRelease(point, self._nfzEdit)
            self._pressVelocity = not self._pressVelocity
            
    def draw(self, visualizer, **kwargs):
        SubGUI.draw(self, visualizer, **kwargs)

        self._nfzEdit.draw(visualizer, **kwargs)
        self._pointMover.draw(visualizer, **kwargs)
        self._mover.draw(visualizer, **kwargs)
