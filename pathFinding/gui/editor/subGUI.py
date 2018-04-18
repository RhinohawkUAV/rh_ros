class SubGUI:
    """Depending on mode of GeometryCreator, a subGUI will be signalled to take action in response to input events."""

    def onLeftPress(self, point, control=False):
        pass

    def onLeftRelease(self, point, control=False):
        pass

    def onMotion(self, point, control=False):
        pass

    def onKey(self, point, key):
        pass

    def onSwitch(self):
        pass
