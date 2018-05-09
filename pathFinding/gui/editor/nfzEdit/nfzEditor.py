from editableNoFlyZoneList import EditableNoFlyZoneList
from gui.core import Drawable
from gui.editor.subGUI import SubGUI
from nfzPointMover import NFZPointMover
from nfzVelocityChanger import NFZVelocityChanger
from noFlyMover import NoFlyMover


class NFZEditor(Drawable, SubGUI):

    def __init__(self):
        self._nfzEdit_nfzEdit = None
        self._pressVelocity = False
        self._pointMover = NFZPointMover()
        self._mover = NoFlyMover()
        self._velocityChanger = NFZVelocityChanger()

    def onSwitch(self, debugInput):
        SubGUI.onSwitch(self, debugInput)
        self._nfzEdit = EditableNoFlyZoneList(debugInput.scenario.noFlyZones)
        debugInput.scenario.noFlyZones = []
    
    def onExit(self):
        self._debugInput.scenario.noFlyZones = self._nfzEdit.asInput()
        
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
            
    def draw(self, canvas, **kwargs):
        self._nfzEdit.draw(canvas, **kwargs)
        self._pointMover.draw(canvas, **kwargs)
        self._mover.draw(canvas, **kwargs)
