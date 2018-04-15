from threading import Thread

from gui.visualizer import Visualizer


class PathFindingVisualizer(Visualizer):
    """
    Visualizes a Geometry object during the path finding process.  Installs itself as a listener and gets a drawable
    copy of the pathFinding.findPathStaticLegacy object on a regular basis.
    """

    def __init__(self, pathFinder, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self._pathFinder = pathFinder
        self._pathFinder.drawListener = self
        self._startedAlgorithm = False

    def startPathFinding(self):
        thread = Thread(None, lambda: self._pathFinder.findPath(), "Path Finding Thread")
        thread.start()

    def onResize(self, event):
        if not self._startedAlgorithm:
            self._startedAlgorithm = True
            self.startPathFinding()
