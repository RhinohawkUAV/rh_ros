from engine.pathFinder import PathFinder
from gui.pathFinder.pathFinderInterface import PathFinderInterface


class LocalPathFinderInterface(PathFinderInterface):
    """
    Manages a local path finder for use in PathFindViewer.
    """

    def __init__(self):
        self._pathFinder = None
    
    def initiate(self, scenario, vehicle):
        """
        Start a new path finding process.  Will wipe out previous process.
        """
        self._pathFinder = PathFinder(scenario, vehicle)
    
    def step(self):
        """
        Perform one step of the path finding process.
        """
        if self._pathFinder is None or self._pathFinder.isDone():
            return
        if self._pathFinder.step():
            solutionPathSegments = self._pathFinder.getSolution()
            self._listener.triggerSolution(solutionPathSegments, self._pathFinder.isDone())
        else:
            debugData = self._pathFinder.getDebugData()
            self._listener.triggerDebug(*debugData)

