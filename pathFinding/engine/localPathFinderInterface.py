from engine.pathFinderManager import PathFinderManager
from gui.pathFinder.pathFinderInterface import PathFinderInterface


class LocalPathFinderInterface(PathFinderManager, PathFinderInterface):
    """
    Manages a local path finder for use in PathFindViewer.
    """

    def __init__(self):
        PathFinderManager.__init__(self)
    
    def submitProblem(self, scenario, vehicle):
        """
        Start a new path finding process.  Will wipe out previous process.
        """
        PathFinderManager.submitProblem(self, scenario, vehicle)
    
    def stepProblem(self, numSteps=1):
        """
        Perform one step of the path finding process.
        """
        PathFinderManager.stepProblem(self, numSteps)

    def publishSolution(self, solutionPathSegments, finished):
        self._listener.fireSolutionInGuiThread(solutionPathSegments, finished)
    
    def publishDebug(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        self._listener.fireDebugInGuiThread(pastPathSegments, futurePathSegments, filteredPathSegments)
