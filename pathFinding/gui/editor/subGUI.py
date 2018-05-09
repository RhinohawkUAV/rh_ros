class SubGUI:
    """Depending on mode of GeometryCreator, a subGUI will be signalled to take action in response to input events."""

    def __init__(self):
        self._debugInput = None

    def onLeftPress(self, point, control=False):
        pass

    def onLeftRelease(self, point, control=False):
        pass

    def onMotion(self, point, control=False):
        pass

    def onKey(self, point, key, ctrl=False):
        pass

    def onSwitch(self, debugInput):
        self._debugInput = debugInput
    
    def onExit(self):
        pass