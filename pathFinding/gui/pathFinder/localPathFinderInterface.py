from threading import Thread

from engine.pathFinderManager import PathFinderManager
from gui.pathFinder.pathFinderInterface import PathFinderInterface


class LocalPathFinderInterface(PathFinderManager, PathFinderInterface):
    """
    Manages a local path finder for use in PathFindViewer.
    """

    def __init__(self):
        PathFinderManager.__init__(self)
    
    def submitProblem(self, params, scenario, vehicle):
        PathFinderManager.submitProblem(self, params, scenario, vehicle)
    
    def stepProblem(self, numSteps=1):
        PathFinderManager.stepProblem(self, numSteps)
        
    def solveProblem(self, timeout):
        pass

    def publishInput(self, params, scenario, vehicle):
        Thread(target=self._doPublishInput, args=(params, scenario, vehicle)).start()

    def publishDebug(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        Thread(target=self._doPublishDebug, args=(pastPathSegments, futurePathSegments, filteredPathSegments)).start()

    def publishSolution(self, solutionWaypoints, solutionPathSegments, finished):
        Thread(target=self._doPublishSolution, args=(solutionWaypoints, solutionPathSegments, finished)).start()

# These functions are necessray due to the failings of the TK library:
# Tk's event processing thread, allows queueing and execution of events like any GUI library.  However, when it executes an event
# it holds onto some kind of internal lock which blocks calls to queue new events via tk.after_idle().  This can cause deadlock as
# these publishing methods bring another lock into play to guarantee order or events.  The result is we can't guarantee order of events,
# thanks TK...

    def _doPublishInput(self, params, scenario, vehicle):
        self._listener.fireInputInGuiThread(params, scenario, vehicle)

    def _doPublishDebug(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        self._listener.fireDebugInGuiThread(pastPathSegments, futurePathSegments, filteredPathSegments)
        
    def _doPublishSolution(self, solutionWaypoints, solutionPathSegments, finished):
        self._listener.fireSolutionInGuiThread(solutionWaypoints, solutionPathSegments, finished)
        
