from engine.pathFinderManager import PathFinderManager
from gui.pathFinder.pathFinderInterface import PathFinderInterface


class LocalPathFinderInterface(PathFinderInterface):
    """
    Manages a local path finder for use in PathFindViewer.
    """

    def __init__(self):
        self._pathFinderManager = PathFinderManager()
        self._pathFinderManager.setListeners(self._fireInputAccepted, self._fireStepPerformed)
        
    def submitProblem(self, params, scenario, vehicle):
        self._pathFinderManager.submitProblem(params, scenario, vehicle)
    
    def stepProblem(self, numSteps=1):
        self._pathFinderManager.stepProblem(numSteps)
        
    def solveProblem(self, timeout):
        pass
