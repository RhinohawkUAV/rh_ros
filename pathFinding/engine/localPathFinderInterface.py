from engine.pathFinder import PathFinder
from gui.pathFinder.pathFinderInterface import PathFinderInterface


class LocalPathFinderInterface(PathFinderInterface):
    """
    Manages a local path finder for use in PathFindViewer.
    """

    def __init__(self):
        self._activePathFinder = None
    
    def initiate(self, scenario, vehicle):
        """
        Start a new path finding process.  Will wipe out previous process.
        """
        self._activePathFinder = PathFinder(scenario, vehicle)
    
    def step(self):
        """
        Perform one step of the path finding process.
        """
        if self._activePathFinder is None or self._activePathFinder.isDone():
            return
        if self._activePathFinder.step():
            solutionPathSegments = self._activePathFinder.getSolution()
            self._listener.triggerSolution(solutionPathSegments, self._activePathFinder.isDone())
        else:
            debugData = self._activePathFinder.getDebugData()
            self._listener.triggerDebug(*debugData)

