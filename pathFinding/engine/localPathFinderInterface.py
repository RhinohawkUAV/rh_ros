from engine.pathFinderManager import PathFinderManager
from gui.pathFinder.pathFinderInterface import PathFinderInterface


class LocalPathFinderInterface(PathFinderManager, PathFinderInterface):
    """
    Manages a local path finder for use in PathFindViewer.
    """

    def __init__(self):
        PathFinderManager.__init__(self)
    
    def submitProblem(self, scenario, vehicle):
        PathFinderManager.submitProblem(self, scenario, vehicle)
    
    def stepProblem(self, numSteps=1):
        PathFinderManager.stepProblem(self, numSteps)
        
    def solveProblem(self, timeout):
        pass

    def publishSolution(self, solutionPathSegments, finished):
        self._listener.fireSolutionInGuiThread(solutionPathSegments, finished)
    
    def publishDebug(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        self._listener.fireDebugInGuiThread(pastPathSegments, futurePathSegments, filteredPathSegments)
