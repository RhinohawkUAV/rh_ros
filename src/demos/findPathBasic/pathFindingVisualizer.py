from threading import Thread

from gui.visualizer import Visualizer


class PathFindingVisualizer(Visualizer):
    """
    Visualizes a Geometry object during the path finding process.  Installs itself as a listener and gets a drawable
    copy of the findPath object on a regular basis.
    """

    def __init__(self, pathFinder, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self._pathFinder = pathFinder
        self._pathFinder.drawListener = self

    # TODO: Move to an onVisible or equivalent method
    def leftClick(self, event):
        thread = Thread(None, lambda: self._pathFinder.findPath(), "Path Finding Thread")
        thread.start()
