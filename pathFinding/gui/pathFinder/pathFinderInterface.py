import gui


class PathFinderInterface:
    """
    Provides an interface to a path finding process for use in the GUI.  
    This allows a local path-finder or remote, ROS, path finder to use the same GUI.
    """

    def __init__(self):
        self._inputAcceptedListeners = []
        self._stepPerformedListeners = []
        self._solvedListeners = []
        
    def addListeners(self, inputAcceptedListener, stepPerformedListener, solvedListener):
        """
        Sets listeners.  These will notified in GUI thread.
        """
        self._inputAcceptedListeners.append(inputAcceptedListener)
        self._stepPerformedListeners.append(stepPerformedListener)
        self._solvedListeners.append(solvedListener)
        
    def submitProblem(self, params, scenario, vehicle):
        """
        Start a new path finding process.  Will wipe out previous process.
        """
        pass
    
    def stepProblem(self, numSteps=1):
        """
        Queue up steps to perform towards finding a path.  Steps are ignored, if path finder has concluded its search.
        """
        pass

    def solveProblem(self, timeout):
        """
        Perform steps until problem is solved or timeout occurs.
        """
        pass

    def _doInputAccepted(self, params, scenario, vehicle):
        for listener in self._inputAcceptedListeners:
            listener(params, scenario, vehicle)

    def _doStepPerformed(self, isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments):
        for listener in self._stepPerformedListeners:
            listener(isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments)

    def _doSolved(self, bestPath):
        for listener in self._solvedListeners:
            listener(bestPath)

    def _fireInputAccepted(self, params, scenario, vehicle):
        """
        In GUI thread: Inform listener of input being accepted for processing.
        """
        gui.inGUIThread(self._doInputAccepted, params, scenario, vehicle)
        
    def _fireStepPerformed(self, isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments):
        """
        In GUI thread: Inform listener of latest step result.
        """
        gui.inGUIThread(self._doStepPerformed, isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments)

    def _fireSolved(self, bestPath):
        """
        In GUI thread: Inform listener of latest step result.
        """
        gui.inGUIThread(self._doSolved, bestPath)
