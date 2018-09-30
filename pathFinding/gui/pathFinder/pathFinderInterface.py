import gui


class PathFinderInterface:
    """
    Provides an interface to a path finding process for use in the GUI.  
    This allows a local path-finder or remote, ROS, path finder to use the same GUI.
    """

    def setListeners(self, inputAcceptedListener, stepPerformedListener):
        """
        Sets listeners.  These will notified in GUI thread.
        """
        self._inputAcceptedListener = inputAcceptedListener
        self._stepPerformedListener = stepPerformedListener
        
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

    def _fireInputAccepted(self, params, scenario, vehicle):
        """
        In GUI thread: Inform listener of input being accepted for processing.
        """
        gui.inGUIThread(self._inputAcceptedListener, params, scenario, vehicle)
        
    def _fireStepPerformed(self, isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments):
        """
        In GUI thread: Inform listener of latest step result.
        """
        gui.inGUIThread(self._stepPerformedListener, isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments)
